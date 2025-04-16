#include "wsg_50_control/wsg_50.hpp"
#include <rclcpp/rclcpp.hpp>


WSG50Driver::WSG50Driver(){}

WSG50Driver::~WSG50Driver(){}


bool WSG50Driver::connect(){
            
    int res_con=cmd_connect_tcp(ip_.c_str(), port_);

    if (res_con!=0){
        return false;
    }
    else{
        connected_ = 1;
        return true;
    }
}

void WSG50Driver::disconnect(){
    connected_ = 0;
    if (auto_update_thread_.joinable()) {
        auto_update_thread_.join(); // Attend la fin du thread
    }
    cmd_disconnect();
}

bool WSG50Driver::setup(){
    if (homing()){
        ack_fault();
        homing();
    }
    rclcpp::sleep_for(std::chrono::milliseconds(500));  // Pause de 5 secondes
    if (grasping_force_ > 0.0) {
        std::cout << "Setting grasping force limit to " << grasping_force_ << std::endl;
        setGraspingForceLimit(grasping_force_);
    }
    auto_update_thread_ = std::thread(std::bind(&WSG50Driver::read_thread, this, (int)(1000.0 / rate_)));
    return true;
}

int WSG50Driver::cmd(double pos, double speed, int mode){
    float goal_pos = pos * 1000.0; // Convert to mm
    float goal_speed = speed;
    switch(mode){
        case 0: // Move
            return move(goal_pos, goal_speed, false, false);
        case 1: // Grasp
            return grasp(goal_pos, goal_speed,true);
        case 2: // Release
            return release(goal_pos, goal_speed,true);
        default:
            RCLCPP_ERROR(rclcpp::get_logger("WSG50Driver"), "Invalid mode");
            return -1;
    }
}

void WSG50Driver::read_thread(int interval_ms){
   // Request automatic updates (error checking is done below)
    getOpening(interval_ms);
    getSpeed(interval_ms);
    getForce(interval_ms);
    int res;
    msg_t msg; msg.id = 0; msg.data = 0; msg.len = 0;

    while(connected_==1){
        msg_free(&msg);
        res = msg_receive( &msg );
        if (res < 0 || msg.len < 2) {
            continue;
        }

        float val = 0.0;
        status_t status = cmd_get_response_status(msg.data);

        // Decode float for opening/speed/force
        if (msg.id >= 0x43 && msg.id <= 0x45 && msg.len == 6) {
            if (status != E_SUCCESS) {
                continue;
            }
            val = convert(&msg.data[2]);
        }
        // Handle response types
        int motion = -1;
        switch (msg.id) {
            /*** Opening ***/
            case 0x43:
                width_ = val/1000.0; // Convert to m
                break;

            /*** Speed ***/
            case 0x44:
                speed_ = val/1000.0; // Convert to m/s
                break;

            /*** Force ***/
            case 0x45:
                force_ = val;
                break;
        }
    }
}