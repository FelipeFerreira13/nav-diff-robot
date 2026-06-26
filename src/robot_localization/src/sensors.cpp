#include "robot_localization/sensors.hpp"

Sensor::Sensor() : Node("sensor_server") {

    this->declare_parameter<float> ("sensor.baseline_distance", 0);
    this->declare_parameter<int> ("sensor.left_ang",  0);
    this->declare_parameter<int> ("sensor.right_ang", 0);
    this->declare_parameter<int> ("sensor.front_ang", 0);

    baseline_distance = this->get_parameter("sensor.baseline_distance" ).as_double();
    left_ang  = this->get_parameter("sensor.left_ang"  ).as_int();
    right_ang = this->get_parameter("sensor.right_ang" ).as_int();
    front_ang = this->get_parameter("sensor.front_ang" ).as_int();

    right_sharp_sub_ = this->create_subscription<sensor_msgs::msg::Range>("robot/sensor/sharp/right/raw", 10, 
        std::bind(&Sensor::right_sharpCallback, this, std::placeholders::_1));
    left_sharp_sub_ = this->create_subscription<sensor_msgs::msg::Range>("robot/sensor/sharp/left/raw", 10, 
        std::bind(&Sensor::left_sharpCallback, this, std::placeholders::_1));
    arm_sharp_sub_ = this->create_subscription<sensor_msgs::msg::Range>("robot/sensor/sharp/arm/raw", 10, 
        std::bind(&Sensor::arm_sharpCallback, this, std::placeholders::_1));
    right_ultrasonic_sub_ = this->create_subscription<sensor_msgs::msg::Range>("robot/sensor/ultrasonic/right/raw", 10, 
        std::bind(&Sensor::right_ultrasonicCallback, this, std::placeholders::_1));
    left_ultrasonic_sub_ = this->create_subscription<sensor_msgs::msg::Range>("robot/sensor/ultrasonic/left/raw", 10, 
        std::bind(&Sensor::left_ultrasonicCallback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 
        rclcpp::SensorDataQoS(),    // Best Effort
        std::bind(&Sensor::scanCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
        std::bind(&Sensor::odom_callback, this, std::placeholders::_1));

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    action_server_ = rclcpp_action::create_server<SensorCommand>( this, "robot/sensor/control",
        std::bind(&Sensor::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Sensor::handle_cancel, this, std::placeholders::_1),
        std::bind(&Sensor::handle_accepted, this, std::placeholders::_1)
    );


    RCLCPP_INFO(this->get_logger(), "sensor_node Started");

}

rclcpp_action::GoalResponse Sensor::handle_goal( const rclcpp_action::GoalUUID &, std::shared_ptr<const SensorCommand::Goal> goal){
    if (goal->command > SensorCommand::Goal::LINEAR_ALIGN)
        return rclcpp_action::GoalResponse::REJECT;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Sensor::handle_cancel(const std::shared_ptr<GoalHandleSensor>){
    RCLCPP_WARN(this->get_logger(), "Sensor goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Sensor::handle_accepted(const std::shared_ptr<GoalHandleSensor> goal_handle){
    std::thread{std::bind(&Sensor::execute, this, goal_handle)}.detach();
}

/* ---------------- EXECUTE ---------------- */

void Sensor::execute(const std::shared_ptr<GoalHandleSensor> goal_handle){
    
    auto result = std::make_shared<SensorCommand::Result>();

    if( goal_handle->get_goal()->command == SensorCommand::Goal::GET_SENSOR ){
        get_sensor(goal_handle, result);
    }else if( goal_handle->get_goal()->command == SensorCommand::Goal::GET_ANGLE ){
        get_angle(goal_handle, result);
    }else if( goal_handle->get_goal()->command == SensorCommand::Goal::LINEAR_ALIGN ){
        linear_align(goal_handle, result);
    }
}

void Sensor::get_sensor( const std::shared_ptr<GoalHandleSensor> goal_handle, std::shared_ptr<SensorCommand::Result> result){

    // PrintScan();

    auto feedback = std::make_shared<SensorCommand::Feedback>();

    std::string sensor = goal_handle->get_goal()->sensor;

    double dist = -1;
    if     ( sensor == "sharp_right" )     { dist = sensor_mean(sharp_right_dist, 5); }
    else if( sensor == "sharp_left" )      { dist = sensor_mean(sharp_left_dist, 5); }
    else if( sensor == "sharp_arm" )       { dist = sensor_mean(sharp_arm_dist, 5); }
    else if( sensor == "ultrasonic_right" ){ dist = sensor_mean(us_right_dist, 5); }
    else if( sensor == "ultrasonic_left" ) { dist = sensor_mean(us_left_dist, 5); }
    else if( sensor == "laser_back" )      { dist = sensor_mean(front_scan, 5); }
    else if( sensor == "laser_left" )      { dist = sensor_mean(left_scan,  5); }
    else if( sensor == "laser_right" )     { dist = sensor_mean(right_scan, 5); }

    // RCLCPP_INFO(this->get_logger(), "Get sensor %s: %f", sensor.c_str(), dist);

    result->success = true;
    result->value = dist;
    goal_handle->succeed(result);

}

float Sensor::get_angle_wall( int sample ){

    float right = sensor_mean(sharp_right_dist, sample);
    float left  = sensor_mean(sharp_left_dist,  sample);

    float sensor_difference = right - left;
    float angle_diff = atan(sensor_difference / baseline_distance);
    angle_diff = angle_diff * (180.0 / M_PI);

    return angle_diff;
}

void Sensor::get_angle( const std::shared_ptr<GoalHandleSensor> goal_handle, std::shared_ptr<SensorCommand::Result> result ){

    double angle_diff = get_angle_wall( 10 );

    double angle_wall = 0;

    double angle = goal_handle->get_goal()->value;

    angle_wall = Utils::straight_ang( angle );

    double minDiff = 0;
    for (int i = 0; i < 5; i++){
        double diff = std::abs( angle - (i * 90));
        if (i == 0 || ( diff < minDiff)){
            angle_wall = i * 90;
            minDiff = diff;
        }
    }
    float estimated_angle_degrees = angle_wall - angle_diff;
    estimated_angle_degrees = Utils::Quotient_Remainder( estimated_angle_degrees, 360 );

    result->success = true;
    result->value = estimated_angle_degrees;
    goal_handle->succeed(result);
}

void Sensor::linear_align( const std::shared_ptr<GoalHandleSensor> goal_handle, std::shared_ptr<SensorCommand::Result> result ){
    
    // PrintScan();
    
    double dist = goal_handle->get_goal()->value;
    std::string direction = goal_handle->get_goal()->sensor;

    int count = 0;
    int count_zero = 0;
    float prev_speed = 0;
    float sensor_dist = 0;

    double des_ang = Utils::straight_ang( robot_position.theta );

    geometry_msgs::msg::Twist cmd;

    rclcpp::Rate rate(20);
    while( count < 5 ){

        if     ( direction.compare( "front"  ) == 0 ){ sensor_dist = front_scan; }
        else if( direction.compare( "right"  ) == 0 ){ sensor_dist = right_scan; }
        else if( direction.compare( "left"   ) == 0 ){ sensor_dist = left_scan;  }
        else if( direction.compare( "back"   ) == 0 ){ sensor_dist = sharp_right_dist; }


        double linear_dist_offset = 0.075; // [m]
        double max_linear_speed   = 0.10;  // [m/s]
        double min_linear_speed   = 0.05;  // [m/s]
        float linear_tolerance    = 0.02;  // [m]

        double desired_v = (-(dist - sensor_dist) / linear_dist_offset) * max_linear_speed;
        desired_v =  std::max( std::min( desired_v, max_linear_speed ), -1 * max_linear_speed );
        if( std::abs(desired_v) < min_linear_speed ){
          desired_v > 0 ? desired_v = min_linear_speed : desired_v = -min_linear_speed; }

        if( std::abs((dist - sensor_dist)) < linear_tolerance / 2.0 && sensor_dist != 0 ){ desired_v = 0; }

        if( sensor_dist <= 0 ){
            desired_v = prev_speed;
            count_zero++;
            count = 0;
        }else{
            count_zero = 0;
        }

        if( count_zero > 5 ){
            desired_v = -min_linear_speed;
        }

        if(       direction.compare( "front" ) == 0 ){
            cmd.linear.x = desired_v;
            cmd.linear.y  = 0;
        }else if( direction.compare( "left" )  == 0 ){
            cmd.linear.x = 0;
            cmd.linear.y  = desired_v;
        }else if( direction.compare( "right" ) == 0 ){
            cmd.linear.x = 0;
            cmd.linear.y  = -desired_v;
        }else if( direction.compare( "back" ) == 0 ){
            cmd.linear.x = -desired_v;
            cmd.linear.y = 0;
        }
        float th_diff = des_ang - robot_position.theta;

        if      ( th_diff < -180 ) { th_diff = th_diff + 360; }
        else if ( th_diff >  180 ) { th_diff = th_diff - 360; }

        double max_ang_speed = 0.75;         // rad/s
        double min_ang_speed = 0.25;         // rad/s
        double angular_dist_offset = 10.0;   // [degrees]
        double angular_tolerance = 1.5;

        double desired_vth = (th_diff / angular_dist_offset) * max_ang_speed; 
        desired_vth =  std::max(  std::min( desired_vth, max_ang_speed ), -1 * max_ang_speed );
        if( std::abs(desired_vth) < min_ang_speed ){
          desired_vth > 0 ? desired_vth = min_ang_speed : desired_vth = -min_ang_speed; }
        if( std::abs(th_diff) < angular_tolerance ){ desired_vth = 0; }

        cmd.angular.z  = desired_vth;

        prev_speed = desired_vth;

        cmd_vel_publisher_->publish(cmd);

        // RCLCPP_INFO(this->get_logger(), "theta: %f", robot_position.theta);
        // RCLCPP_INFO(this->get_logger(), "sensor_dist: %f dist: %f, theta: %f", sensor_dist, dist, robot_position.theta);
        // RCLCPP_INFO(this->get_logger(), "vx: %f vy: %f vth: %f", cmd.linear.x, cmd.linear.y, cmd.angular.z);

        if( std::abs((dist - sensor_dist)) < linear_tolerance && desired_vth == 0 )
            { count++; }
        else if( sensor_dist > 0 )
            { count = 0; }

        rate.sleep();
    }

    geometry_msgs::msg::Twist cmd_;
    cmd_vel_publisher_->publish(cmd_);

    result->success = true;
    goal_handle->succeed(result);
}


float Sensor::sensor_mean( double & sensor_dist, int samples ){
    int count = 0;
    int count_out = 0;

    float average = 0;

    rclcpp::Rate rate(10);
    while ( count < samples && count_out < 30 ){

        double dist;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            dist = sensor_dist;
        }

        if( dist > 0.02 && dist < 10 && dist != 0 ){
            average += ( dist / samples);
            count++;
        }

        // RCLCPP_INFO(this->get_logger(), "dist: %f", dist );

        count_out++;
        rate.sleep();
    }
    process_done = true;
    return average;
}
