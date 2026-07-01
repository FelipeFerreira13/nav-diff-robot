#include "robot_control/controller/controller.hpp"

Controller::Controller() : 
    Node("controller_node") {

    this->declare_parameter<bool> ("diff_drive.enabled", false);
    this->declare_parameter<float>("diff_drive.wheel_radius",0);
    this->declare_parameter<float>("diff_drive.robot_radius",0);
    this->declare_parameter<int>("diff_drive.front_left",0);
    this->declare_parameter<int>("diff_drive.front_right",0);
    this->declare_parameter<int>("diff_drive.rear_left",0);
    this->declare_parameter<int>("diff_drive.rear_right",0);
    this->declare_parameter<float>("diff_drive.max_motor_speed",0);

    bool diff_drive_enabled = this->get_parameter("diff_drive.enabled" ).as_bool();
    differential_kim.R = this->get_parameter("diff_drive.wheel_radius" ).as_double();
    differential_kim.L = this->get_parameter("diff_drive.robot_radius" ).as_double() * 2;
    differential_kim.fl_ = this->get_parameter("diff_drive.front_left" ).as_int();
    differential_kim.fr_ = this->get_parameter("diff_drive.front_right" ).as_int();
    differential_kim.rl_ = this->get_parameter("diff_drive.rear_left" ).as_int();
    differential_kim.rr_ = this->get_parameter("diff_drive.rear_right" ).as_int();
    differential_kim.max_motor_speed_ = this->get_parameter("diff_drive.max_motor_speed" ).as_double();

    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "Diff Drive Parameters:\n"
        "  enabled: %s\n"
        "  wheel_radius R: %.4f\n"
        "  wheel_base L: %.4f\n"
        "  front_left: %d\n"
        "  front_right: %d\n"
        "  rear_left: %d\n"
        "  rear_right: %d\n"
        "  max_motor_speed: %.4f",
        diff_drive_enabled ? "true" : "false",
        differential_kim.R,
        differential_kim.L,
        differential_kim.fl_,
        differential_kim.fr_,
        differential_kim.rl_,
        differential_kim.rr_,
        differential_kim.max_motor_speed_
    );



    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, 
        std::bind(&Controller::cmdVelCallback, this, std::placeholders::_1));

    enc_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("motor/get_encoder", 10, 
        std::bind(&Controller::encCallback, this, std::placeholders::_1));

    stop_button_sub_  = this->create_subscription<std_msgs::msg::Bool>("/robot/button/stop/raw", 10, 
        std::bind(&Controller::stop_Callback, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("robot/twist", 10);

    speed_client_ = this->create_client<robot_control::srv::SetSpeed>  ("motor/set_motor_speed");

    pid_service_ = this->create_service<robot_control::srv::Pid>(
        "motor/pid", 
        std::bind(&Controller::pidCallback, this, std::placeholders::_1, std::placeholders::_2));

    timer_ = this->create_wall_timer( 
        std::chrono::milliseconds(40), 
        std::bind(&Controller::publish_odometry, this));

    previous_time_cmd  = this->now().seconds();
    previous_time_odom = this->now().seconds();

    pid_wheels.resize(4);
    wheels_cmd.resize(4);


    for (int i=0; i<4; i++) { pid_wheels[i].initPid(0.2, 0.4, 0.0, 0.2, -0.2); }

    RCLCPP_INFO(this->get_logger(), "Controller started using %s kinematics", kinematics_.c_str());
}

void Controller::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {

    double current_time = this->now().seconds();
    double dt = current_time - previous_time_cmd;
    previous_time_cmd = current_time;

    // RCLCPP_INFO(this->get_logger(), "dvx=%.3f dvy=%.3f dvth=%.2f cvx=%.3f cvy=%.3f cvth=%.2f dt=%.2f", 
    //     msg->linear.x, msg->linear.y, msg->angular.z, 
    //     robot_twist.linear.x, robot_twist.linear.y, robot_twist.angular.z,
    //     dt
    // );

    if (dt < 0.0001 || dt > 0.5) return;

    Twist2D twist{msg->linear.x, msg->linear.y, msg->angular.z};

    bool stop_motors = false;
    if( !stop_state || ( twist.vx == 0 && twist.vy == 0 && twist.omega == 0 ) ){ 
        twist.vx = 0; twist.vy = 0; twist.omega = 0;
        for( auto & c : pid_wheels ){ c.reset(); }
        stop_motors = true;
    }

    DifferentialSpeeds wheel_vels = differential_kim.inverse(twist);

    int fl_ = differential_kim.fl_;
    int fr_ = differential_kim.fr_;
    int rl_ = differential_kim.rl_;
    int rr_ = differential_kim.rr_;

    double error_fl = wheel_vels.front_left  - differential_vel.front_left;
    double error_fr = wheel_vels.front_right - differential_vel.front_right;
    double error_rl = wheel_vels.rear_left  - differential_vel.rear_left;
    double error_rr = wheel_vels.rear_right - differential_vel.rear_right;

    wheels_cmd[fl_] = pid_wheels[fl_].computeCommand( error_fl, (uint64_t)(dt * 1e9) );
    wheels_cmd[fr_] = pid_wheels[fr_].computeCommand( error_fr, (uint64_t)(dt * 1e9) );
    wheels_cmd[rl_] = pid_wheels[rl_].computeCommand( error_rl, (uint64_t)(dt * 1e9) );
    wheels_cmd[rr_] = pid_wheels[rr_].computeCommand( error_rr, (uint64_t)(dt * 1e9) );

    if( stop_motors ){ 
        wheels_cmd[fl_] = 0;
        wheels_cmd[fr_] = 0;
        wheels_cmd[rl_] = 0;
        wheels_cmd[rr_] = 0;
    }
    
    auto wheels = std::make_shared<robot_control::srv::SetSpeed::Request>();

    // RCLCPP_INFO(this->get_logger(), "dl=%.3f cl=%.3f pid=%.2f dt=%.2f", wheel_vels.front_right, differential_vel.front_right, wheels_cmd[fr_], dt);

    wheels->motor = fl_;
    wheels->speed = wheels_cmd[fl_] /= differential_kim.max_motor_speed_;
    speed_client_->async_send_request( wheels );

    wheels->motor = fr_;
    wheels->speed = wheels_cmd[fr_] /= differential_kim.max_motor_speed_;
    speed_client_->async_send_request( wheels );

    wheels->motor = rl_;
    wheels->speed = wheels_cmd[rl_] /= differential_kim.max_motor_speed_;
    speed_client_->async_send_request( wheels );

    wheels->motor = rr_;
    wheels->speed = wheels_cmd[rr_] /= differential_kim.max_motor_speed_;
    speed_client_->async_send_request( wheels );
    
}

void Controller::publish_odometry() {

    double current_time = this->now().seconds();
    double dt = current_time - previous_time_odom;
    previous_time_odom = current_time;

    if (dt < 0.0001 || dt > 0.5) return;
    
    Twist2D twist;


    differential_vel.front_left  = differential_kim.WheelSpeed(enc[differential_kim.fl_] - previous_encoder[differential_kim.fl_], dt);
    differential_vel.front_right = differential_kim.WheelSpeed(enc[differential_kim.fr_] - previous_encoder[differential_kim.fr_], dt);
    differential_vel.rear_left   = differential_kim.WheelSpeed(enc[differential_kim.rl_] - previous_encoder[differential_kim.rl_], dt);
    differential_vel.rear_right  = differential_kim.WheelSpeed(enc[differential_kim.rr_] - previous_encoder[differential_kim.rr_], dt);


    twist = differential_kim.forward( (differential_vel.front_left  + differential_vel.rear_left ) / 2.0, 
                                      (differential_vel.front_right + differential_vel.rear_right) / 2.0);
    

    robot_twist.linear.x  = twist.vx;
    robot_twist.linear.y  = twist.vy;
    robot_twist.angular.z = twist.omega;

    twist_pub_->publish( robot_twist );

    // RCLCPP_INFO(this->get_logger(), "vx=%.2f vy=%.2f vth=%.2f", twist.vx, twist.vy, twist.omega);
    // RCLCPP_INFO(this->get_logger(), "fl=%.2f fr=%.2f rl=%.2f rr=%.2f", 
    //     differential_vel.front_left,
    //     differential_vel.front_right,
    //     differential_vel.rear_left,
    //     differential_vel.rear_right
    // );



    for (int i=0; i<4; i++) { previous_encoder[i] = enc[i]; }

}

void Controller::encCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    for (int i=0; i<4; i++) { enc[i] = msg->data[i]; }
}

void Controller::pidCallback( std::shared_ptr<robot_control::srv::Pid::Request> request, std::shared_ptr<robot_control::srv::Pid::Response> response ){

    for (int i=0; i<4; i++) { 
        pid_wheels[i].initPid(
            request->p, 
            request->i, 
            request->d, 
            request->i_max, 
            request->i_min
        ); 
    }

}
