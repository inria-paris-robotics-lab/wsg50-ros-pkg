/*
 * WSG 50 ROS NODE
 * Copyright (c) 2012, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Marc Benetó (mbeneto@robotnik.es)
 * \brief WSG-50 ROS driver.
 */


//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------


#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <thread>
#include <chrono>


#include "wsg_50_driver/common.h"
#include "wsg_50_driver/cmd.h"
#include "wsg_50_driver/msg.h"
#include "wsg_50_driver/functions.h"

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "wsg_50_common/msg/status.hpp"
#include "wsg_50_common/srv/conf.hpp"

#include "wsg_50_common/action/cmd.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"



#define GRIPPER_MIN_OPEN 2.0 // unit: mm
#define GRIPPER_MAX_OPEN 110.0
#define GRIPPER_MIN_SPEED 0.0
#define GRIPPER_MAX_SPEED 420.0
#define GRIPPER_WIDTH_THRESHOLD 1.0

#define MODE_MOVE 0
#define MODE_GRASP 1
#define MODE_RELEASE 2

class WSG50Node : public rclcpp::Node{
    public:
        WSG50Node():Node("wsg_50"){
            this->declare_parameter<std::string>("ip", "192.168.1.20");
            this->declare_parameter<int>("port", 1000);
            this->declare_parameter<std::string>("protocol", "tcp");
            // Warning to the user if rate is too low the driver could not work properly !!
            this->declare_parameter<double>("rate", 100.0);
            this->declare_parameter<double>("grasping_force", 0.0);
            this->declare_parameter<bool>("finger_sensors", false);

            this->get_parameter("ip", ip_);
            this->get_parameter("port", port_);
            this->get_parameter("protocol", protocol_);
            this->get_parameter("rate", rate_);
            this->get_parameter("grasping_force", grasping_force_);
            this->get_parameter("finger_sensors", finger_sensors_);

            std::cout << "IP: " << ip_ << std::endl;
            std::cout << "Port: " << port_ << std::endl;
            std::cout << "Protocol: " << protocol_ << std::endl;
            std::cout << "Rate: " << rate_ << std::endl;
            std::cout << "Grasping force: " << grasping_force_ << std::endl;

            

            RCLCPP_INFO(this->get_logger(), "Connecting to %s:%d; communication mode: %s ...",
                        ip_.c_str(), port_, protocol_.c_str());
            
            int res_con=cmd_connect_tcp(ip_.c_str(), port_);

            if (res_con!=0){
                RCLCPP_ERROR(this->get_logger(), "Unable to connect, please check the port and address used.");
                rclcpp::shutdown();
            }
            else{
                RCLCPP_INFO(this->get_logger(), "Connected to WSG-50 gripper.");

                // Publishers
                g_pub_state = this->create_publisher<wsg_50_common::msg::Status>("status", 1000);
                g_pub_joint = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
                g_pub_moving = this->create_publisher<std_msgs::msg::Bool>("moving", 10);

                // Services
                homing_srv_ = this->create_service<std_srvs::srv::Trigger>("homing", std::bind(&WSG50Node::homingSrv, this, std::placeholders::_1, std::placeholders::_2));
                stop_srv_ = this->create_service<std_srvs::srv::Trigger>("stop", std::bind(&WSG50Node::stopSrv, this, std::placeholders::_1, std::placeholders::_2));
                set_force_srv_ = this->create_service<wsg_50_common::srv::Conf>("set_force", std::bind(&WSG50Node::setForceSrv, this, std::placeholders::_1, std::placeholders::_2));
                set_acc_srv_ = this->create_service<wsg_50_common::srv::Conf>("set_acc", std::bind(&WSG50Node::setAccSrv, this, std::placeholders::_1, std::placeholders::_2));
                ack_srv_ = this->create_service<std_srvs::srv::Trigger>("ack", std::bind(&WSG50Node::ackSrv, this, std::placeholders::_1, std::placeholders::_2));

                // Actions
                move_action_server_ = rclcpp_action::create_server<wsg_50_common::action::Cmd>(
                    this,
                    "wsg_50_cmd",
                    std::bind(&WSG50Node::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                    std::bind(&WSG50Node::handle_cancel, this, std::placeholders::_1),
                    std::bind(&WSG50Node::handle_accepted, this, std::placeholders::_1));

                // Initialize gripper
                RCLCPP_INFO(this->get_logger(), "Ready to use. Homing and taring gripper, stand away...");
                RCLCPP_INFO(this->get_logger(), "Homing gripper...");
                if (homing()){
                    RCLCPP_ERROR(this->get_logger(), "Error while homing gripper");
                    RCLCPP_INFO(this->get_logger(), "Trying to ack fault the gripper...");
                    ack_fault();
                    RCLCPP_INFO(this->get_logger(), "Retrying homing...");
                    homing();
                }
                else{
                    RCLCPP_INFO(this->get_logger(), "Gripper homed.");
                }
                rclcpp::sleep_for(std::chrono::milliseconds(500));  // Pause de 5 secondes
                if (this->finger_sensors_){
                    RCLCPP_INFO(this->get_logger(), "Taring gripper...");
                    doTare();
                    do_tare_srv_ = this->create_service<std_srvs::srv::Trigger>("do_tare", std::bind(&WSG50Node::doTareSrv, this, std::placeholders::_1, std::placeholders::_2));
                }

                if (grasping_force_ > 0.0) {
                    RCLCPP_INFO(this->get_logger(), "Setting grasping force limit to %f", grasping_force_);
                    setGraspingForceLimit(grasping_force_);
                }
                RCLCPP_INFO(this->get_logger(), "Gripper ready.");

                auto_update_thread_ = std::thread(std::bind(&WSG50Node::read_thread, this, (int)(1000.0 / rate_)));

            }
        }

        ~WSG50Node() {
            if (auto_update_thread_.joinable()) {
                auto_update_thread_.join(); // Attend la fin du thread
            }
            cmd_disconnect();
            RCLCPP_INFO(this->get_logger(), "Disconnected from WSG-50 gripper.");
        }
    private:
        std::string ip_, protocol_;
        int port_, local_port_;
        double rate_, grasping_force_;
        float increment, g_goal_position = NAN, g_goal_speed = NAN, g_speed = 10.0;
        float current_width_ = 0.0;
        bool g_ismoving = false, finger_sensors_ = false;
        bool object_grasped = false;
        rclcpp::TimerBase::SharedPtr timer_;
        std::thread auto_update_thread_;

        // Publishers
        rclcpp::Publisher<wsg_50_common::msg::Status>::SharedPtr g_pub_state;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr g_pub_joint;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr g_pub_moving;


        // Services
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr homing_srv_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr do_tare_srv_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
        rclcpp::Service<wsg_50_common::srv::Conf>::SharedPtr set_force_srv_;
        rclcpp::Service<wsg_50_common::srv::Conf>::SharedPtr set_acc_srv_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ack_srv_;

        // Actions
        rclcpp_action::Server<wsg_50_common::action::Cmd>::SharedPtr move_action_server_;
        
        void homingSrv(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,const std::shared_ptr<std_srvs::srv::Trigger::Response> res){
            RCLCPP_INFO(this->get_logger(), "Received homing command");
            RCLCPP_INFO(this->get_logger(), "Homing gripper...");
            res->success=homing(true);
            if(res->success==0){
                RCLCPP_INFO(this->get_logger(), "Homing completed");
                res->message="Homing completed";
            }
            else{
                RCLCPP_ERROR(this->get_logger(), "Error while homing gripper");
                res->success="Homing failed";
            }
        }
        // revoir do tare marche pas 
        void doTareSrv(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, const std::shared_ptr<std_srvs::srv::Trigger::Response> res){
            RCLCPP_INFO(this->get_logger(), "Received tare command");
            RCLCPP_INFO(this->get_logger(), "Taring gripper...");
            res->success=doTare(true);
            if(res->success==0){
                RCLCPP_INFO(this->get_logger(), "Taring completed");
                res->message="Taring completed";
            }
            else{
                RCLCPP_ERROR(this->get_logger(), "Error while taring gripper");
                res->success="Taring failed";
            }
        }

        // revoir
        void stopSrv(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,const std::shared_ptr<std_srvs::srv::Trigger::Response> res){
            RCLCPP_INFO(this->get_logger(), "Received stop command");
            RCLCPP_INFO(this->get_logger(), "Stopping gripper...");
            res->success=stop(true);
            if(res->success==0){
                RCLCPP_INFO(this->get_logger(), "Gripper stopped");
                res->message="Gripper stopped";
            }
            else{
                RCLCPP_ERROR(this->get_logger(), "Error while stopping gripper");
                res->success="Stop failed";
            }
        }

        void setForceSrv(const std::shared_ptr<wsg_50_common::srv::Conf::Request> req,const std::shared_ptr<wsg_50_common::srv::Conf::Response> res){
            RCLCPP_INFO(this->get_logger(), "Received set force command");
            if(req->val>0.0 && req->val<=80.0){
                RCLCPP_INFO(this->get_logger(), "Setting grasping force to %f", req->val);
                res->error=setGraspingForceLimit(req->val,true);
                if(res->error == 0){
                    RCLCPP_INFO(this->get_logger(), "Grasping force set to %f", req->val);
                    grasping_force_ = req->val;
                }
                else{
                    RCLCPP_ERROR(this->get_logger(), "Error while setting grasping force");
                }
            }
            else{
                RCLCPP_ERROR(this->get_logger(), "Grasping force values are outside the gripper's physical limits (Force values: [0.0 - 80.0]");
                res->error = 255;
            }
        }

        void setAccSrv(const std::shared_ptr<wsg_50_common::srv::Conf::Request> req,const std::shared_ptr<wsg_50_common::srv::Conf::Response> res){
            RCLCPP_INFO(this->get_logger(), "Received set acceleration command");
            if (req->val>100.0 && req->val<=5000.0){
                RCLCPP_INFO(this->get_logger(), "Setting acceleration to %f", req->val);
                res->error=setAcceleration(req->val,true);
                if(res->error == 0){
                    RCLCPP_INFO(this->get_logger(), "Acceleration set to %f", req->val);
                }
                else{
                    RCLCPP_ERROR(this->get_logger(), "Error while setting acceleration");
                }
            }
            else{
                RCLCPP_ERROR(this->get_logger(), "Acceleration values are outside the gripper's physical limits (Acceleration values: [100.0 - 5000.0] mm/s²");
                res->error = 255;
            }
        }

        void ackSrv(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,const std::shared_ptr<std_srvs::srv::Trigger::Response> res){
            RCLCPP_INFO(this->get_logger(), "Received ack command");
            res->success=ack_fault(true);
            if(res->success==0){
                RCLCPP_INFO(this->get_logger(), "Ack completed");
                res->message="Ack completed";
            }
            else{
                RCLCPP_ERROR(this->get_logger(), "Error while acking gripper");
                res->success="Ack failed";
            }
        }

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const wsg_50_common::action::Cmd::Goal> goal){
            // Check if the goal is valid
            if(goal->mode<0 || goal->mode>2){
                RCLCPP_ERROR(this->get_logger(), "Invalid command: %d. Must be 0 (move), 1 (grasp) or 2 (release)", goal->mode);
                return rclcpp_action::GoalResponse::REJECT;
            }
            // Check if the width is within the valid range
            if(goal->width < GRIPPER_MIN_OPEN || goal->width > GRIPPER_MAX_OPEN){
                RCLCPP_ERROR(this->get_logger(), "Invalid width: %f. Must be between %f and %f", goal->width, GRIPPER_MIN_OPEN, GRIPPER_MAX_OPEN);
                return rclcpp_action::GoalResponse::REJECT;
            }
            // Check if the speed is within the valid range
            if(goal->speed <= GRIPPER_MIN_SPEED || goal->speed > GRIPPER_MAX_SPEED){
                RCLCPP_ERROR(this->get_logger(), "Invalid speed: %f. Must be between %f and %f", goal->speed, GRIPPER_MIN_SPEED, GRIPPER_MAX_SPEED);
                return rclcpp_action::GoalResponse::REJECT;
            }
            // Accept the goal
            RCLCPP_INFO(this->get_logger(), "Received command request");
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<wsg_50_common::action::Cmd>> goal_handle){
            RCLCPP_INFO(this->get_logger(), "Received cancel request");
            // Accept the cancel request
            (void)goal_handle;
            stop(true);
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<wsg_50_common::action::Cmd>> goal_handle){
            using namespace std::placeholders;
            std::thread{std::bind(&WSG50Node::move_exec, this,_1), goal_handle}.detach();
        }

     
        void move_exec(const std::shared_ptr<rclcpp_action::ServerGoalHandle<wsg_50_common::action::Cmd>> goal_handle){
            auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<wsg_50_common::action::Cmd::Feedback>();
            auto result = std::make_shared<wsg_50_common::action::Cmd::Result>();
            int error = -1;
            switch (goal->mode){
                case MODE_MOVE:
                    RCLCPP_INFO(this->get_logger(), "Moving gripper to width: %f at speed: %f", goal->width, goal->speed);
                    error = move(goal->width, goal->speed, false, true);
                    break;
                case MODE_GRASP:
                    RCLCPP_INFO(this->get_logger(), "Grasping object with width: %f at speed: %f", goal->width, goal->speed);
                    error = grasp(goal->width, goal->speed, true);
                    break;
                case MODE_RELEASE:
                    RCLCPP_INFO(this->get_logger(), "Releasing object with width: %f at speed: %f", goal->width, goal->speed);
                    error = release(goal->width, goal->speed, true);
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Invalid command: %d. Must be 0 (move), 1 (grasp) or 2 (release)", goal->mode);
                    result->result = false;
                    goal_handle->abort(result);
                    return;
            }
            // Update feedback during the movement and handle cancel requests
            while (g_ismoving){
                if (goal_handle->is_canceling()) {
                    result->result = false;
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal cancel confirmation sent");
                    return;
                }
                // Update feedback
                feedback->position = current_width_;
                goal_handle->publish_feedback(feedback);
            }
            // Check the result of the movement
            if (rclcpp::ok()) {
                if (error<0){
                    result->result = false;
                    RCLCPP_ERROR(this->get_logger(), "Fail to send command to gripper");
                    goal_handle->abort(result);
                }
                switch (goal->mode){
                    case MODE_MOVE:
                        if (goal->width >= current_width_ - GRIPPER_WIDTH_THRESHOLD && goal->width <= current_width_ + GRIPPER_WIDTH_THRESHOLD){
                            result->result = true;
                        }
                        else{
                            result->result = false;
                        }
                        break;
                    case MODE_GRASP:
                        if (object_grasped){
                            result->result = true;
                        }
                        else{
                            result->result = false;
                        }
                        break;
                    case MODE_RELEASE:
                        if (!object_grasped){
                            result->result = true;
                        }
                        else{
                            result->result = false;
                        }
                        break;
                    default:
                        break;
                }
                if (result->result){
                   goal_handle->succeed(result);
                }
                else{
                    goal_handle->abort(result);
                }
            }
           
        }


        /** \brief Reads gripper responses in auto_update mode. The gripper pushes state messages in regular intervals. */
        void read_thread(int interval_ms){
            RCLCPP_INFO(this->get_logger(), "Starting read thread with interval %d ms", interval_ms);

            status_t status;
            int res;

            // double rate_exp = 1000.0 / (double)interval_ms;
            std::string names[3] = { "opening", "speed", "force" };

            // Prepare messages
            wsg_50_common::msg::Status status_msg;
            status_msg.status = "UNKNOWN";

            sensor_msgs::msg::JointState joint_states;
            joint_states.header.frame_id = "wsg50_base_link";
            joint_states.name.push_back("wsg50_finger_left_joint");
            joint_states.name.push_back("wsg50_finger_right_joint");
            joint_states.position.resize(2);
            joint_states.velocity.resize(2);
            joint_states.effort.resize(2);

            // Request automatic updates (error checking is done below)
            getOpening(interval_ms);
            getSpeed(interval_ms);
            getForce(interval_ms);

            msg_t msg; msg.id = 0; msg.data = 0; msg.len = 0;
            int cnt[3] = {0,0,0};
            auto time_start = std::chrono::system_clock::now();


            while (rclcpp::ok()) {
                // Receive gripper response
                msg_free(&msg);
                res = msg_receive( &msg );
                if (res < 0 || msg.len < 2) {
                    RCLCPP_ERROR(this->get_logger(), "Gripper response failure");
                    continue;
                }

                float val = 0.0;
                status = cmd_get_response_status(msg.data);

                // Decode float for opening/speed/force
                if (msg.id >= 0x43 && msg.id <= 0x45 && msg.len == 6) {
                    if (status != E_SUCCESS) {
                        RCLCPP_ERROR(this->get_logger(), "Gripper response failure for opening/speed/force");
                        continue;
                    }
                    val = convert(&msg.data[2]);
                }
               
                // Handle response types
                int motion = -1;
                switch (msg.id) {
                /*** Opening ***/
                case 0x43:
                    status_msg.width = val;
                    cnt[0]++;
                    break;

                /*** Speed ***/
                case 0x44:
                    status_msg.speed = val;
                    cnt[1]++;
                    break;

                /*** Force ***/
                case 0x45:
                    status_msg.force = val;
                    cnt[2]++;
                    break;
                
                /***Homing ***/
                case 0x20:
                    if (status == E_SUCCESS) {
                        RCLCPP_INFO(this->get_logger(), "Homing completed");
                        status_msg.status = "Homing completed";
                        motion = 0;
                    } else if (status == E_CMD_PENDING) {
                        RCLCPP_INFO(this->get_logger(), "Homing in progress");
                        status_msg.status = "Homing in progress";
                        motion = 1;
                    } else if (status == E_CMD_ABORTED) {
                        RCLCPP_ERROR(this->get_logger(), "Homing aborted");
                        status_msg.status = "Homing aborted";
                        motion = 0;
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Homing error");
                        status_msg.status = "Homing error";
                        motion = 0;
                    }
                    break;
                /*** Move ***/
                // Move commands are sent from outside this thread
                case 0x21:
                    if (status == E_SUCCESS) {
                        RCLCPP_INFO(this->get_logger(), "Position reached");
                        status_msg.status = "Position reached";
                        motion = 0;
                    } else if (status == E_AXIS_BLOCKED) {
                        RCLCPP_ERROR(this->get_logger(), "Axis blocked");
                        status_msg.status = "Axis blocked";
                        motion = 0;
                    } else if (status == E_CMD_PENDING) {
                        RCLCPP_INFO(this->get_logger(), "Gripper moving");
                        if(object_grasped){
                            object_grasped = false;
                        }
                        status_msg.status = "Gripper moving";
                        motion = 1;
                    } else if (status == E_ALREADY_RUNNING) {
                        RCLCPP_ERROR(this->get_logger(), "Gripper is already moving");
                        status_msg.status = "Gripper is already moving";
                    } else if (status == E_CMD_ABORTED) {
                        RCLCPP_INFO(this->get_logger(), "Movement aborted");
                        status_msg.status = "Movement aborted";
                        motion = 0;
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Movement error");
                        status_msg.status = "Movement error";
                        motion = 0;
                    }
                    break;

                /*** Stop ***/
                // Stop commands are sent from outside this thread
                case 0x22:
                    if(status == E_SUCCESS) {
                        RCLCPP_INFO(this->get_logger(), "Gripper stopped");
                        status_msg.status = "Gripper stopped";
                        motion = 0;
                    } else if (status == E_CMD_PENDING) {
                        RCLCPP_INFO(this->get_logger(), "Gripper is stopping");
                        status_msg.status = "Gripper stopping";
                    } else if (status == E_CMD_ABORTED) {
                        RCLCPP_INFO(this->get_logger(), "Stop command aborted");
                        status_msg.status = "Stop command aborted";
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Stop command error");
                        status_msg.status = "Stop command error";
                    }
                    break;
                /*** ack ***/
                // Ack commands are sent from outside this thread
                case 0x24:
                    if(status == E_SUCCESS) {
                        RCLCPP_INFO(this->get_logger(), "Fault acknowledged");
                        status_msg.status = "Fault acknowledged";
                    } else if (status == E_CMD_PENDING) {
                        RCLCPP_INFO(this->get_logger(), "Acknowledging fault");
                        status_msg.status = "Acknowledging fault";
                    } else if (status == E_CMD_ABORTED) {
                        RCLCPP_INFO(this->get_logger(), "Ack command aborted");
                        status_msg.status = "Ack command aborted";
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Ack command error");
                        status_msg.status = "Ack command error";
                    }
                    break;
                /*** Grasp ***/
                // Grasp commands are sent from outside this thread
                case 0x25:
                    if(status == E_SUCCESS) {
                        RCLCPP_INFO(this->get_logger(), "Grasp completed");
                        status_msg.status = "Grasp completed";
                        if (!object_grasped) {
                            RCLCPP_INFO(this->get_logger(), "Object grasped");
                            object_grasped = true;
                        }
                        motion = 0;
                    } else if (status == E_CMD_PENDING) {
                        RCLCPP_INFO(this->get_logger(), "Grasp in progress");
                        status_msg.status = "Grasp in progress";
                        motion = 1;
                    } else if (status == E_CMD_ABORTED) {
                        RCLCPP_INFO(this->get_logger(), "Grasp command aborted");
                        status_msg.status = "Grasp command aborted";
                        motion = 0;
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Grasp command error");
                        status_msg.status = "Grasp command error";
                        motion = 0;
                    }
                    break;
                /*** Release ***/
                // Release commands are sent from outside this thread
                case 0x26:
                    if(status == E_SUCCESS) {
                        RCLCPP_INFO(this->get_logger(), "Release completed");
                        status_msg.status = "Release completed";
                        if (object_grasped) {
                            RCLCPP_INFO(this->get_logger(), "Object released");
                            object_grasped = false;
                        }
                        motion = 0;
                    } else if (status == E_CMD_PENDING) {
                        RCLCPP_INFO(this->get_logger(), "Release in progress");
                        status_msg.status = "Release in progress";
                        motion = 1;
                    } else if (status == E_CMD_ABORTED) {
                        RCLCPP_INFO(this->get_logger(), "Release command aborted");
                        status_msg.status = "Release command aborted";
                        motion = 0;
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Release command error");
                        status_msg.status = "Release command error";
                        motion = 0;
                    }
                    break;
                /*** Set Acceleration ***/
                // Set acceleration commands are sent from outside this thread
                case 0x30:
                    if(status == E_SUCCESS) {
                        RCLCPP_INFO(this->get_logger(), "Acceleration set");
                        status_msg.status = "Acceleration set";
                    } else if (status == E_CMD_PENDING) {
                        RCLCPP_INFO(this->get_logger(), "Setting acceleration");
                        status_msg.status = "Setting acceleration";
                    } else if (status == E_CMD_ABORTED) {
                        RCLCPP_INFO(this->get_logger(), "Set acceleration command aborted");
                        status_msg.status = "Set acceleration command aborted";
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Set acceleration command error");
                        status_msg.status = "Set acceleration command error";
                    }
                    break;
                /*** Set Grasping Force ***/
                // Set force commands are sent from outside this thread
                case 0x32:
                    if(status == E_SUCCESS) {
                        RCLCPP_INFO(this->get_logger(), "Force set");
                        status_msg.status = "Force set";
                    } else if (status == E_CMD_PENDING) {
                        RCLCPP_INFO(this->get_logger(), "Setting force");
                        status_msg.status = "Setting force";
                    } else if (status == E_CMD_ABORTED) {
                        RCLCPP_INFO(this->get_logger(), "Set force command aborted");
                        status_msg.status = "Set force command aborted";
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Set force command error");
                        status_msg.status = "Set force command error";
                    }
                    break;
                /*** Do Tare ***/
                // Do tare commands are sent from outside this thread
                case 0x38:
                    if(status == E_SUCCESS) {
                        RCLCPP_INFO(this->get_logger(), "Tare completed");
                        status_msg.status = "Tare completed";
                    } else if (status == E_CMD_PENDING) {
                        RCLCPP_INFO(this->get_logger(), "Taring in progress");
                        status_msg.status = "Taring in progress";
                    } else if (status == E_CMD_ABORTED) {
                        RCLCPP_INFO(this->get_logger(), "Tare command aborted");
                        status_msg.status = "Tare command aborted";
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Tare command error");
                        status_msg.status = "Tare command error";
                    }
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown response from gripper: 0x%02x (%2dB)", msg.id, msg.len);
                    break;
                }

                // ***** PUBLISH motion message
                if (motion == 0 || motion == 1) {
                    std_msgs::msg::Bool moving_msg;
                    moving_msg.data = motion;
                    g_pub_moving->publish(moving_msg);
                    
                }

                if(motion == 0){
                    g_ismoving = false;
                }
                else{
                    g_ismoving = true;
                }
                
                // ***** PUBLISH state message & joint message
                status_msg.object_grasped = object_grasped;
                g_pub_state->publish(status_msg);
                current_width_ = status_msg.width;

                joint_states.header.stamp = rclcpp::Clock().now();
                joint_states.position[0] = -status_msg.width/2000.0;
                joint_states.position[1] = status_msg.width/2000.0;
                joint_states.velocity[0] = status_msg.speed/1000.0;
                joint_states.velocity[1] = status_msg.speed/1000.0;
                joint_states.effort[0] = status_msg.force;
                joint_states.effort[1] = status_msg.force;
                g_pub_joint->publish(joint_states);

                // Check # of received messages regularly
                std::chrono::duration<float> t = std::chrono::system_clock::now() - time_start;
                double t_ = t.count();
                if (t_ > 5.0) {
                    time_start = std::chrono::system_clock::now();
                    //printf("Infos for %5.1fHz, %5.1fHz, %5.1fHz\n", (double)cnt[0]/t_, (double)cnt[1]/t_, (double)cnt[2]/t_);

                    std::string info = "Rates for ";
                    for (int i=0; i<3; i++) {
                        double rate_is = (double)cnt[i]/t_;
                        info += names[i] + ": " + std::to_string((int)rate_is) + "Hz, ";
                        if (rate_is == 0.0)
                            RCLCPP_ERROR(this->get_logger(), "Did not receive data for %s", names[i].c_str());
                    }
                    // RCLCPP_DEBUG_STREAM((info + " expected: " + std::to_string((int)rate_exp) + "Hz").c_str());
                    cnt[0] = 0; cnt[1] = 0; cnt[2] = 0;
                }


            }

            // // Disable automatic updates
            // // TODO: The functions will receive an unexpected response
            // getOpening(0);
            // getSpeed(0);
            // getForce(0);

            RCLCPP_INFO(this->get_logger(), "Thread ended");
        }

};



/**
 * The main function
 */

int main( int argc, char **argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WSG50Node>());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down WSG-50 node...");
    rclcpp::shutdown();
    return 0;
}


//------------------------------------------------------------------------
// Testing functions
//------------------------------------------------------------------------