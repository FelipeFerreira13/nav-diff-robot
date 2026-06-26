#include "robot_control/controller/controller.hpp"

Controller::Controller() : 
    Node("controller_server") {

    this->declare_parameter<bool> ("diff_drive.enabled", false);
    this->declare_parameter<float>("diff_drive.wheel_radius",0);
    this->declare_parameter<float>("diff_drive.wheelbase",0);
    this->declare_parameter<int>("diff_drive.left",0);
    this->declare_parameter<int>("diff_drive.right",0);
    this->declare_parameter<float>("diff_drive.max_motor_speed",0);

    bool diff_drive_enabled = this->get_parameter("diff_drive.enabled" ).as_bool();
    differential_kim.R = this->get_parameter("diff_drive.wheel_radius" ).as_double();
    differential_kim.L = this->get_parameter("diff_drive.wheelbase" ).as_double();
    differential_kim.l_ = this->get_parameter("diff_drive.left" ).as_int();
    differential_kim.r_ = this->get_parameter("diff_drive.right" ).as_int();
    differential_kim.max_motor_speed_ = this->get_parameter("diff_drive.max_motor_speed" ).as_double();



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

    RCLCPP_INFO(this->get_logger(), "dvx=%.3f dvy=%.3f dvth=%.2f cvx=%.3f cvy=%.3f cvth=%.2f dt=%.2f", 
        msg->linear.x, msg->linear.y, msg->angular.z, 
        robot_twist.linear.x, robot_twist.linear.y, robot_twist.angular.z,
        dt
    );

    if (dt < 0.0001 || dt > 0.5) return;

    Twist2D twist{msg->linear.x, msg->linear.y, msg->angular.z};

    bool stop_motors = false;
    if( !stop_state || ( twist.vx == 0 && twist.vy == 0 && twist.omega == 0 ) ){ 
        twist.vx = 0; twist.vy = 0; twist.omega = 0;
        for( auto & c : pid_wheels ){ c.reset(); }
        stop_motors = true;
    }

    DifferentialSpeeds wheel_vels = differential_kim.inverse(twist);

    int l_ = differential_kim.l_;
    int r_ = differential_kim.r_;

    double error_l = wheel_vels.left  - differential_vel.left;
    double error_r = wheel_vels.right - differential_vel.right;

    wheels_cmd[l_] = pid_wheels[l_].computeCommand( error_l, (uint64_t )(dt * 1e9) );
    wheels_cmd[r_] = pid_wheels[r_].computeCommand( error_r, (uint64_t )(dt * 1e9) );

    if( stop_motors ){ 
        wheels_cmd[l_] = 0;
        wheels_cmd[r_] = 0;
    }
    
    auto wheels = std::make_shared<robot_control::srv::SetSpeed::Request>();

    // RCLCPP_INFO(this->get_logger(), "dl=%.3f cl=%.3f pid=%.2f dt=%.2f", wheel_vels.left, differential_vel.left, wheels_cmd[l_], dt);

    wheels->motor = l_;
    wheels->speed = wheels_cmd[l_] /= differential_kim.max_motor_speed_;
    speed_client_->async_send_request( wheels );

    wheels->motor = r_;
    wheels->speed = wheels_cmd[r_] /= differential_kim.max_motor_speed_;
    speed_client_->async_send_request( wheels );
    
}

void Controller::publish_odometry() {

    double current_time = this->now().seconds();
    double dt = current_time - previous_time_odom;
    previous_time_odom = current_time;

    if (dt < 0.0001 || dt > 0.5) return;
    
    Twist2D twist;


    differential_vel.left  = differential_kim.WheelSpeed(enc[differential_kim.l_] - previous_encoder[differential_kim.l_], dt);
    differential_vel.right = differential_kim.WheelSpeed(enc[differential_kim.r_] - previous_encoder[differential_kim.r_], dt);

    twist = differential_kim.forward( differential_vel.left, differential_vel.right);
    

    robot_twist.linear.x = twist.vx;
    robot_twist.linear.y = twist.vy;
    robot_twist.angular.z = twist.omega;

    twist_pub_->publish( robot_twist );

    // RCLCPP_INFO(this->get_logger(), "cvx=%.3f cvy=%.3f cvth=%.2f dt=%.2f", twist.vx, twist.vy, twist.omega, dt);

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
