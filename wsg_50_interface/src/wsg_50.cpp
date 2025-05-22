#include "wsg_50_interface/wsg_50.hpp"
#include <rclcpp/rclcpp.hpp>

/***
 * @brief Constructor for the WSG50Driver class
 */
WSG50Driver::WSG50Driver(){}

/***
 * @brief Destructor for the WSG50Driver class
 */
WSG50Driver::~WSG50Driver(){}

/***    
 * @brief Connecting to the real WSG50 gripper
 * @return true if the connection is successful
 */
bool WSG50Driver::connect(){
    int res_con;
    if(protocol_ == "tcp"){
        RCLCPP_INFO(rclcpp::get_logger("WSG50Driver"), "Connecting to %s:%d", ip_.c_str(), port_);
        res_con=cmd_connect_tcp(ip_.c_str(), port_);
    }
    else if(protocol_ == "udp"){
        RCLCPP_INFO(rclcpp::get_logger("WSG50Driver"), "Connecting to %s:%d:%d", ip_.c_str(), port_, local_port_);
        res_con = cmd_connect_udp(local_port_, ip_.c_str(), port_);
    }
    else{
        RCLCPP_ERROR(rclcpp::get_logger("WSG50Driver"), "Invalid protocol");
        return false;
    }

    if (res_con!=0){
        return false;
    }
    connected_ = 1;
    return true;
}
/***
 * @brief Disconnecting from the WSG50 gripper
 * @return true if the disconnection is successful
 * @note This function will wait for the auto-update thread to finish before disconnecting
 */
bool WSG50Driver::disconnect(){
    connected_ = 0; // Stop the auto-update thread
    if (auto_update_thread_.joinable()) {
        auto_update_thread_.join(); // Wait for the thread to finish
    }
    else{
        return false;
    }
    cmd_disconnect(); // Disconnect from the device
    return true;
}
/***
 * @brief Setup the WSG50 gripper
 * @return true if the setup is successful
 * @note This function will homming or enable the gripper and set the grasping force limit and tare the finger sensors
 */
bool WSG50Driver::setup(){
    if (homing()){// Homing if it is not done that means the gripper is in error case
        ack_fault(); // Acknowledge the fault
        homing();   // Homing again
    }
    rclcpp::sleep_for(std::chrono::milliseconds(500));  // Wait for the gripper to be ready
    if (grasping_force_ > 0.0) { // Set the grasping force limit if it is greater than 0
        RCLCPP_INFO(rclcpp::get_logger("WSG50Driver"), "Setting grasping force limit to %f", grasping_force_);
        setGraspingForceLimit(grasping_force_);
    }
    if (finger_sensors_){ // Tare the finger sensors if it is equipped on the gripper
        if(doTare()==0)
            RCLCPP_INFO(rclcpp::get_logger("WSG50Driver"), "Tare done");
        else
            RCLCPP_ERROR(rclcpp::get_logger("WSG50Driver"), "Tare failed");
    }
    auto_update_thread_ = std::thread(std::bind(&WSG50Driver::read_thread, this, (int)(1000.0 / rate_))); // Start the auto-update thread
    return true;
}
/***
 * @brief Move the gripper to a specific position
 * @param pos The position to move to in m
 * @param speed The speed to move at in mm/s
 * @param mode The mode to use (0: move, 1: grasp, 2: release)
 * @return 0 if successful, -1 if failed
 */
int WSG50Driver::cmd(double pos, double speed, int mode){
    float goal_pos = pos * 1000.0; // Convert to mm
    float goal_speed = speed; // mm/s
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
/***
 * @brief Get the current informartion transmitted from the gripper
 * @param interval_ms The interval in ms to update the information
 * @note This function will start as a thread to update the information
 * @note The information will be updated every interval_ms ms
 */
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
    // Disconnect from the device
    // TODO: cause some weird response message but not very important
    // getOpening(0);
    // getSpeed(0);
    // getForce(0);
    // std::cout << "Thread stopped" << std::endl;
}