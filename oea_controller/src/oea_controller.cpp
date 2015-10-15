#include "oea_controller/oea_controller.h"
#include <mission_ctrl_msgs/mission_ctrl_defines.h>

#define _USE_MATH_DEFINES

namespace oea_controller{

TOEAController::TOEAController(ros::NodeHandle nodeHandle) 
{

    nodeHandle.param("debug", debug_, false);
    nodeHandle.param("debug_velocities", debug_vel_, false);
    nodeHandle.param("always_show_protective_laser", always_show_protective_laser_, true);

    if (debug_) //output debug
    {
        if(ros::console::set_logger_level("ros.oea_controller", ros::console::levels::Debug))
            ros::console::notifyLoggerLevelsChanged();
    }

    std::string name = ROSCONSOLE_DEFAULT_NAME; //ros.carlos_motion_action_server
    name = name  + ".control";
    logger_name_ = "control";
    //logger is ros.carlos_motion_action_server.control

    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn)) //to avoid to much debug msg from actions
        ros::console::notifyLoggerLevelsChanged();

    if (debug_)
    {
        // if we use ROSCONSOLE_DEFAULT_NAME we'll get a ton of debug messages from actionlib which is annoying!!!
        // so for debug we'll use a named logger
        if(ros::console::set_logger_level(name, ros::console::levels::Debug)) //name
            ros::console::notifyLoggerLevelsChanged();
    }
    else // if not DEBUG we want INFO
    {
        if(ros::console::set_logger_level(name, ros::console::levels::Info)) //name
            ros::console::notifyLoggerLevelsChanged();
    }

    name = ROSCONSOLE_DEFAULT_NAME;
    name = name  + ".velocities";

    if (debug_vel_)
    {
        if(ros::console::set_logger_level(name, ros::console::levels::Debug)) //name
            ros::console::notifyLoggerLevelsChanged();
    }
    else
    {
        if(ros::console::set_logger_level(name, ros::console::levels::Info)) //name
            ros::console::notifyLoggerLevelsChanged();
    }

    ROS_DEBUG_STREAM_NAMED(logger_name_, "Starting oea_controller");


    // get parameters
    nodeHandle.param<std::string>("global_frame_id", global_frame_id, "map");
    nodeHandle.param<std::string>("base_frame_id", base_frame_id, "base_footprint");
    nodeHandle.param<std::string>("plan_topic", plan_topic, "/oea_planner/oea_plan");

    // maximum and minimum (absolute) velocities allowed
    nodeHandle.param("max_linear_vel", max_linear_vel, 0.15); // 15 cm/s
    nodeHandle.param("max_angular_vel", max_angular_vel, 0.15); // 8.5 degrees/sec
    nodeHandle.param("max_angular_vel_hard", max_angular_vel_hard, 0.15); // 2.9 degrees/sec
    nodeHandle.param("min_linear_vel", min_linear_vel, 0.01);
    nodeHandle.param("min_angular_vel", min_angular_vel, 0.01); // 0.58 degrees/sec
    nodeHandle.param("keep_ratio", keep_ratio, true);
    nodeHandle.param("k_r", k_r, 0.08);

    // tolerance
      // middle points
    nodeHandle.param("tolerance_yaw_deg", tolerance_yaw, 5.0);
    tolerance_yaw = to_radians(tolerance_yaw);
    nodeHandle.param("tolerance_d", tolerance_d, 0.1); //1*resolution
    nodeHandle.param("tolerance_erro_theta2", tolerance_erro_theta2, 85.0);
    tolerance_erro_theta2=to_radians(tolerance_erro_theta2);
      // last point and points close to obstacles
    nodeHandle.param("tolerance_d_last_point", tolerance_d_last_point, 0.05);
    nodeHandle.param("tolerance_yaw_deg_last_point", tolerance_yaw_last_point, 5.0);
    tolerance_yaw_last_point = to_radians(tolerance_yaw_last_point);


    // controller gains
    nodeHandle.param("Kp", Kp, 1.0); //Proportional Gain
    nodeHandle.param("Kd", Kd, 0.0); //Derivative Gain
    // nodeHandle.param("Ki", Ki, 0.0); //Integral Gain - not implemented
    // nodeHandle.param("Kff", Kff, 0.1); //Feedfprward Gain // not used anymore (since grouping points)

    nodeHandle.param("limit_velocity", limit_velocity, false);
    nodeHandle.param("next_target_timeout", next_target_timeout, 10);
    nodeHandle.param("obstacle_detection", obstacle_detection, true);
    nodeHandle.param("error_deviation_deg", error_deviation, 5.0);
    error_deviation = to_radians(error_deviation);
    nodeHandle.param("scale_on_curves", scale_on_curves, 0.3);

    // because lasers are not symmetrical and we want a symmetrical safety area around lasers
    // this is hardcoded for the CARLoS robot - make this more generic
    front_laser_offset = 0.08;
    back_laser_offset = 0.13;

    double tmp;
    nodeHandle.param("forward_protective_distance", tmp, 0.325);
    FRONT_FWD_DIST = tmp + front_laser_offset; //offset laser-track
    BACK_FWD_DIST = tmp + back_laser_offset; //offset laser-track

    nodeHandle.param("lateral_protective_distance", LAT_DIST, 0.3);
    nodeHandle.param("send_markers", send_markers, false);
    nodeHandle.param("stop_with_joystick", stop_with_joystick, true);
    nodeHandle.param("use_yaw_on_middle_points", use_yaw_on_middle_points_, true);

    FRONT_CORNER_ANG = atan(LAT_DIST/FRONT_FWD_DIST);
    BACK_CORNER_ANG = atan(LAT_DIST/BACK_FWD_DIST);

	double tmp2;
    nodeHandle.param("forward_warning_distance", tmp2, 0.325);
    FRONT_WARN_FWD_DIST = tmp2 + front_laser_offset; //offset laser-track
    BACK_WARN_FWD_DIST = tmp2 + back_laser_offset; //offset laser-track

    nodeHandle.param("lateral_warning_distance", WARN_LAT_DIST, 0.3);
    FRONT_WARN_CORNER_ANG = atan(WARN_LAT_DIST/FRONT_WARN_FWD_DIST);
    BACK_WARN_CORNER_ANG = atan(WARN_LAT_DIST/BACK_WARN_FWD_DIST);


    nodeHandle.param<std::string>("front_laser_link", front_laser_link_, "hokuyo_front_laser_link");
    nodeHandle.param<std::string>("back_laser_link", back_laser_link_, "hokuyo_back_laser_link");

    nodeHandle.param("deaccel_distance_thresh", deaccel_distance_thresh, 0.18); //3*6 cm

    nodeHandle.param("delta_angle_stuck_th", delta_angle_stuck_th, 5);
    nodeHandle.param("erro_theta2_stuck_th_low", erro_theta2_stuck_th_low, 80);
    nodeHandle.param("erro_theta2_stuck_th_sup", erro_theta2_stuck_th_sup, 100);

    nodeHandle.param("obstacle_timeout", obstacle_timeout, 20);

    marker_id_zones=0;
    marker_id_laser=0;

    FMainTimer=new QTimer;
    FTfPoseSub=new qt_ros_interface::TQrosTfPoseSub(global_frame_id, base_frame_id);
    FCommandVelPub=new qt_ros_interface::TQrosGeometryMsgsTwistPub("/cmd_vel");
    FFrontLaserScanSub=new qt_ros_interface::TQrosSensorMsgsLaserScanSub("/front_laser_scan");
    FBackLaserScanSub=new qt_ros_interface::TQrosSensorMsgsLaserScanSub("/back_laser_scan");
    FPoseGoalSub=new qt_ros_interface::TQrosGeometryMsgsPoseStampedSub("/pose_goal");
    FControllerStatePub=new qt_ros_interface::TQrosStdMsgsStringPub("controller_state", "~"); //, true);
    FControllerCommandSub=new qt_ros_interface::TQrosStdMsgsStringSub("controller_command", "~");
    connect(FControllerCommandSub, SIGNAL(DataReceived()), this, SLOT(controller_command_call_back()));
    FDebugNextTargetPub=new qt_ros_interface::TQrosGeometryMsgsPoseStampedPub("next_target", "~");
    FDebugErrorThetaPub=new qt_ros_interface::TQrosStdMsgsFloat64Pub("error_theta", "~");
    FDebugErrorTheta2Pub=new qt_ros_interface::TQrosStdMsgsFloat64Pub("error_theta2", "~");
    FDebugErrorDistancePub=new qt_ros_interface::TQrosStdMsgsFloat64Pub("error_distance", "~");
    Path_sub_ = nodeHandle.subscribe(plan_topic, 1, &TOEAController::planCallback, this);
    FControllerZoneStatePub = new qt_ros_interface::TQrosStdMsgsStringPub("zone_state", "~"); //, true);

    if (send_markers)
    {
        marker_protective_laser_pub_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("protective_laser_markers", 1);
        marker_warning_laser_pub_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("warning_laser_markers", 1);
        markers_zone_pub_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("zones_markers", 1, true);
        markers_deaccel_pub_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("deaccel_markers", 1);
    }

    if (stop_with_joystick)
        joy_sub_ = nodeHandle.subscribe<sensor_msgs::Joy>("/joy", 10, &TOEAController::joyCallback, this);

    server_sub_ = nodeHandle.subscribe<mission_ctrl_msgs::hardware_state>(CARLOS_BASE_STATE_MSG,5,&TOEAController::serverCB, this);

    connect(FMainTimer, SIGNAL(timeout()), this, SLOT(MainTimerCallBack()));
    connect(FPoseGoalSub, SIGNAL(DataReceived()), this, SLOT(PoseGoalSubCallBack()));

    if (obstacle_detection)
    {
        connect(FFrontLaserScanSub, SIGNAL(DataReceived()), this, SLOT(FrontLaserScanSubCallBack()));
        connect(FBackLaserScanSub, SIGNAL(DataReceived()), this, SLOT(BackLaserScanSubCallBack()));
    }

    timer_msecs_ = 100;
    FMainTimer->start(timer_msecs_);
    state_goto_ = STOP;
    pub_state_ = state_goto_;
    flag_target_received_ = false;
    enable_pub_ = false;
    plan_received_ = false;
    compute_next_target_ = false;
    is_last_target_ = false;
    pose_index = 0;
    pub_emergency_errors = true;
    next_target_is_in_front = true;

    flag_obst_front = flag_obst_front_left = flag_obst_front_right = flag_obst_back = flag_obst_back_left =flag_obst_back_right = false;
    obstacle_in_warning_zone_ = false;
    obstacle_in_warning_front_ = false;
    obstacle_in_warning_back_ = false;
    show_laser_front_, show_laser_back_ = false;

    //TODO - print this to rosout!!!
 /*   std::cout << BOLDMAGENTA << "Parameters: " << RESET << std::endl;
    std::cout << BOLDWHITE   << "  tolerance_d: " << RESET << CYAN << tolerance_d << " (meters)" << RESET << std::endl;
    std::cout << BOLDWHITE   << "  tolerance_yaw: " << RESET << CYAN << to_degrees(tolerance_yaw) << " (degrees)"  << RESET << std::endl;

    std::cout << BOLDWHITE   << "  * tolerance_d_last_point: " << RESET << CYAN << tolerance_d_last_point << " (meters)" << RESET << std::endl;
    std::cout << BOLDWHITE   << "  * tolerance_yaw_last_point: " << RESET << CYAN << to_degrees(tolerance_yaw_last_point) << " (degrees)"  << RESET << std::endl;

    std::cout << BOLDWHITE   << "  max_linear_vel: " << RESET << CYAN << max_linear_vel << " (m/s)" << RESET << std::endl;
    std::cout << BOLDWHITE   << "  max_angular_vel: " << RESET << CYAN << max_angular_vel << " (rad/s) <-> " << to_degrees(max_angular_vel) << " (deg/s) " << RESET << std::endl;
    std::cout << BOLDMAGENTA << "PD parameters: " << RESET << std::endl;
    std::cout << BOLDWHITE   << "    Kp: " << BOLDCYAN << Kp << RESET << std::endl;
    std::cout << BOLDWHITE   << "    Kd: " << BOLDCYAN << Kd << RESET << std::endl;
    //std::cout << BOLDWHITE   << "    Kff: " << BOLDCYAN << Kff << RESET << std::endl;
    std::cout << BOLDWHITE   << "limit_velocity: "<< CYAN << limit_velocity   << RESET << std::endl;
    std::cout << BOLDWHITE   << "keep_ratio: " << CYAN << keep_ratio  << RESET << std::endl;
    std::cout << BOLDWHITE   << "next_target_timeout: "<< CYAN << next_target_timeout << RESET << std::endl;
    std::cout << BOLDMAGENTA << "Laser Parameters: " << RESET << std::endl;
    std::cout << BOLDRED   << "  forward_protective_distance: " << RESET << CYAN << tmp << " (meters)" << RESET << std::endl;
    //std::cout << BOLDRED   << "  FRONT_FWD_DIST: " << RESET << CYAN << FRONT_FWD_DIST << " (meters)" << RESET << std::endl;
    //std::cout << BOLDRED   << "  BACK_FWD_DIST: " << RESET << CYAN << BACK_FWD_DIST << " (meters)" << RESET << std::endl;

    std::cout << BOLDRED   << "  lateral_protective_distance: " << RESET << CYAN << LAT_DIST << " (meters)" << RESET << std::endl;
    std::cout << BOLDYELLOW   << "  forward_warning_distance: " << RESET << CYAN << tmp2 << " (meters)" << RESET << std::endl;
    std::cout << BOLDYELLOW   << "  lateral_warning_distance: " << RESET << CYAN << WARN_LAT_DIST << " (meters)" << RESET << std::endl;
    std::cout << BOLDWHITE   << "  send laser markers: " << RESET << CYAN << send_markers << RESET << std::endl;

    std::cout << BOLDMAGENTA << "Other parameters: " << RESET << std::endl;
    std::cout << BOLDWHITE   << "  stop with joystick: " << RESET << CYAN << stop_with_joystick << RESET << std::endl;
    std::cout << BOLDWHITE   << "  error_deviation: " << RESET << CYAN << to_degrees(error_deviation) << RESET << std::endl;
    std::cout << BOLDWHITE   << "  scale_on_curves: " << RESET << CYAN << scale_on_curves << RESET << std::endl;
    std::cout << BOLDWHITE   << "  k_r: " << RESET << CYAN << k_r << RESET << std::endl;

    std::cout << BOLDGREEN   << "  use_yaw_on_middle_points: " << RESET << CYAN << use_yaw_on_middle_points_ << RESET << std::endl;
    std::cout << BOLDGREEN   << "  deaccel_distance_thresh: " << RESET << CYAN << deaccel_distance_thresh << RESET << std::endl;

    std::cout << BOLDGREEN   << "  delta_angle_stuck_th: " << RESET << CYAN << delta_angle_stuck_th << RESET << std::endl;
    std::cout << BOLDGREEN   << "  erro_theta2_stuck_th_low: " << RESET << CYAN << erro_theta2_stuck_th_low << RESET << std::endl;
    std::cout << BOLDGREEN   << "  erro_theta2_stuck_th_sup: " << RESET << CYAN << erro_theta2_stuck_th_sup << RESET << std::endl;

    std::cout << BOLDCYAN   << "  obstacle_timeout: " << RESET << CYAN << obstacle_timeout << RESET << std::endl;
*/

    if (send_markers)
        pub_zones_markers_ = true;
    else
        pub_zones_markers_ = false;

    f = boost::bind(&TOEAController::paramsCB, this,  _1, _2);
    params_server.setCallback(f);

    control_done_ = false;
    obstacle_timer_ = nodeHandle.createTimer(ros::Duration(obstacle_timeout),&TOEAController::obstacle_timerCB, this, false, false);

    running_action_ = false; // is true when receives and is following an action goal, false if following pose or topic plan

    service_server = nodeHandle.advertiseService("stop_robot", &TOEAController::force_stop_robot, this);
    robot_pose_pub_ = nodeHandle.advertise<geometry_msgs::PoseStamped>("robot",1);

    obstacle_timer_started_ = false;

}


TOEAController::~TOEAController()
{
    // driver keeps last sent vel - send 0 vel to stop platform
    geometry_msgs::Twist commandToSend;
    commandToSend.linear.x=0;
    commandToSend.angular.z=0;
    FCommandVelPub->SendData(commandToSend);

    if(FDebugErrorDistancePub) delete FDebugErrorDistancePub;
    if(FDebugErrorTheta2Pub) delete FDebugErrorTheta2Pub;
    if(FDebugErrorThetaPub) delete FDebugErrorThetaPub;
    if(FDebugNextTargetPub) delete FDebugNextTargetPub;
    if(FControllerStatePub) delete FControllerStatePub;
    if(FPoseGoalSub) delete FPoseGoalSub;
    if(FFrontLaserScanSub) delete FFrontLaserScanSub;
    if(FBackLaserScanSub) delete FBackLaserScanSub;
    if(FCommandVelPub) delete FCommandVelPub;
    if(FTfPoseSub) delete FTfPoseSub;
    if(FMainTimer) delete FMainTimer;
    if(FControllerZoneStatePub) delete FControllerZoneStatePub;

}



double TOEAController::CalcDist(double x1, double y1, double x2, double y2)
{
    double result = sqrt(std::pow(x1-x2,2)+std::pow(y1-y2,2));
    return result;
}

double DifAng(double ang1, double ang2)
{
    return NormalizeAngle(ang1-ang2);
}

/*double ShortestAng(double ang1, double ang2)
{
    return angles::shortest_angular_distance(ang1, ang2);
}*/

bool TOEAController::cancel_action(std::string str)
{
    ROS_WARN_STREAM_NAMED(logger_name_, "Canceling action: " << str);
    stop_robot("action was cancelled");
    control_done_ = true;
    result_str_ = str;
    result_state_ = false;
    running_action_ = false;

}

void TOEAController::MainTimerCallBack()
{
    if (obstacle_in_warning_front_ || obstacle_in_warning_back_)
        obstacle_in_warning_zone_ = true;
    else
        obstacle_in_warning_zone_ = false;

    // losing move server is only problematic if moving.
    // if running on topic/poses is not needed
    // if on action, but stopped, can get up again with no prob
    if (server_sub_.getNumPublishers()==0 and running_action_)
    {
        cancel_action("Lost connection to move_platform_server while moving");
        ROS_ERROR_NAMED(logger_name_, "Lost connection to move_platform_server while moving - please relaunch it");
    }

    double theta2target, erro_theta2,erro_dist; //, erro_theta_f,
    double linearVel, angularVel;
    linearVel = angularVel = 0;
    float v_sign = 0; // w_sign = 0;
    float scale = 1;
    double delta_angle; //diff between theta_current and target_theta
  //  float k_vw=0;
    bool pub_velocities = false; //, print = false;

    if(FTfPoseSub->GetPose(current_x, current_y, current_theta))
    {

        geometry_msgs::PoseStamped t;
        t.header.frame_id = "map";
        t.header.stamp = ros::Time::now();
        t.pose.position.x = current_x;
        t.pose.position.y = current_y;
        t.pose.orientation = tf::createQuaternionMsgFromYaw(current_theta);
        robot_pose_pub_.publish(t);

        FControllerStatePub->SendData(QString::number(pub_state_).toStdString()); // send to topic which state was chosen
        hw_state_ = pub_state_;

        if (send_markers)
        {
            if (show_laser_front_ || show_laser_back_)
            {
                if (marker_protective_laser_pub_.getTopic()!="")
                {
                    send_cube_array(current_x, current_y, marker_array_protective_laser, front_laser_link_, 1);
                    send_cube_array(current_x, current_y, marker_array_protective_laser, back_laser_link_, 1);
                    marker_protective_laser_pub_.publish(marker_array_protective_laser);
                }
            }

            if (obstacle_in_warning_zone_)
            {
                //ROS_DEBUG_NAMED(logger_name_, "Obstacle in warning zone");
                if (marker_warning_laser_pub_.getTopic()!="")
                {
                    send_cube_array(current_x, current_y, marker_array_warning_laser, front_laser_link_, 2);
                    send_cube_array(current_x, current_y, marker_array_warning_laser, back_laser_link_, 2);
                    marker_warning_laser_pub_.publish(marker_array_warning_laser);
                }
            }

            if (pub_zones_markers_)
            {
                 if (markers_zone_pub_.getTopic()!="")
                     markers_zone_pub_.publish(marker_zones_array_);
                pub_zones_markers_ = false;
            }
        }

        enable_pub_ = true; //enable pub of velocities at the end of cycle
        //print = false;

        bool robot_is_stuck = false; //robot is stuck parallel to a goal

        if (plan_received_ || flag_target_received_) // plan or single pose received
        {

            ros::Duration time_elapsed = ros::Time::now()- time_next_target_; // to know when the robot gets stuck

            // if robot is stuck
            // pode passar este tempo se estiver parado por causa de obstaculos -> alterar
            if (time_elapsed > ros::Duration(next_target_timeout) && plan_received_) //TODO: and not stoped because of an obstacle
            {
                if (is_last_target_) //last goal of the global plan
                {
                    // if there are no more targets, stop the robot where it is.
                    state_goto_ = STOP; //
                    robot_is_stuck = true;
                    compute_next_target_ = false;
                }
                else // if there are more targets, send next target to unstuck the robot from that point
                {
                    compute_next_target_ = true;
                    ROS_WARN_NAMED(logger_name_, "Robot is stuck - trying next target");
                }
            }


            if (compute_next_target_)
            {
                time_next_target_ = ros::Time::now(); //start new time

                compute_next_target_ = false;
                GetNextTargetFromPlan(pose_index); //sets x y and target_theta
                pose_index++; // inc index in plan
                state_goto_ = CHOOSE_STATE; // choose which state to go next

                if (pose_index == n_poses) //because it's incremented after calling GetNextTargetFromPlan()
                    is_last_target_ = true;

                theta2target = atan2(target2.y-current_y, target2.x-current_x);
                erro_theta2 = NormalizeAngle(current_theta-theta2target);
                next_target_is_in_front = target_is_in_front(erro_theta2); // know if next target is in front or not of the position the robot is NOW
            }

            delta_angle = DifAng(current_theta, target2.yaw); //heading diff
            theta2target = atan2(target2.y-current_y, target2.x-current_x);
            erro_theta2 = NormalizeAngle(current_theta-theta2target);
            erro_dist = CalcDist(current_x,current_y, target2.x,target2.y);

            if ((fabs(delta_angle) < to_radians(delta_angle_stuck_th)) &&
               ((fabs(erro_theta2) > to_radians(erro_theta2_stuck_th_low)) && (fabs(erro_theta2) < to_radians(erro_theta2_stuck_th_sup))) &&
               (fabs(erro_dist) > tolerance_d_last_point) &&
               is_last_target_)
            {
                state_goto_ = STOP; // choose which state to go next
                robot_is_stuck =  true;
                ROS_DEBUG_NAMED(logger_name_, "Robot is stuck parallel");
            }

/*
              (Target#1)                    (Target#2)                      (LastTarget)
   state:          ||   PID  |    PID  |   PID  ||   PID  |  DE_ACCEL | STOP  ||
   dist2target:             (2d)      (d)       (0)      (2d)         (d)     (0)
                                send next_target
*/
            //choose which state to go
            switch(state_goto_)
            {
                case CHOOSE_STATE:
                if (state_goto_ != EMERGENCY) //only choose state if not in emergency state
                {
                    pub_emergency_errors = true; //can publish emergency messages (is set to false when enters emergency)

                    if (is_last_target_) // only gets here if robot is not stuck
                    {                   // if stuck goes directly to STOP state
                        compute_next_target_ = false;

                       /* if (erro_dist > 2*tolerance_d_last_point)
                        {
                            state_goto = PID;
                        }
                        else
                        {*/

                        if (d_to_next_target(target2) < deaccel_distance_thresh) //deaccel_before_target(target2, target3) ||  // deacel because we're about to change orientation (change layer)
                        {
                            if ((erro_dist < tolerance_d_last_point) && (fabs(delta_angle) < tolerance_yaw_last_point))
                            {
                                state_goto_ = STOP;
                            }
                            else
                            {
                                state_goto_ = DE_ACCEL;
                            }
                        }
                        else // still far from goal, don't deaccel
                        {
                            state_goto_ = PID;
                        }

                      // }
                    }
                    else // is not last target
                    {
                        if (go_next_target(erro_dist, erro_theta2)) //close to target
                        {
                            compute_next_target_ = true;
                            state_goto_ = CHOOSE_STATE;  // if should go to next target, choose state again
                            is_first_target_ = false;
                        }
                        else
                        {
                            if (d_to_next_target(target2) < deaccel_distance_thresh) //deaccel_before_target(target2, target3) ||  // deacel because we're about to change orientation (change layer)
                            {
                                state_goto_ = DE_ACCEL;
                            }
                            else
                            {
                                state_goto_ = PID;
                            }
                        }
                    }

                    if ((obstacle_in_warning_zone_) && (state_goto_ != STOP))
                    {
                        ROS_DEBUG_NAMED(logger_name_, "DEACCEL because of obstacle in warning zone");
                        state_goto_ = DE_ACCEL;
                    }
                }
            } //CHOOSE STATE switch()

             // FControllerStatePub->SendData(QString::number(state_goto).toStdString()); // send to topic which state was chosen
            pub_state_ = state_goto_;

            switch(state_goto_)
            {
            case EMERGENCY:
                ROS_WARN_STREAM_COND_NAMED(pub_emergency_errors, logger_name_, "STOPPED WITH JOYSTICK");
                pub_emergency_errors = false; //only pub msg once every touch
                if (running_action_)
                {
                    cancel_action("Robot was stopped with joystick");
                }
                else
                    stop_robot("Robot was stopped with joystick");
                break;
            case STOP:
                if (!go_next_target(erro_dist, erro_theta2) && !robot_is_stuck) // if not close to the goal, shouldn't be stoped
                {
                    state_goto_ = CHOOSE_STATE;
                }
                else // if within tolerance or stuck close to goal
                {
                    linearVel = 0;
                    angularVel = 0;

                    if (robot_is_stuck)
                    {
                        result_str_ = "Robot is stuck - didn't reach goal";
                        result_state_ = false;
                        ROS_WARN_NAMED(logger_name_, "Robot is stuck close to goal. Stoping without reaching it... :(");
                    }
                    else
                    {
                        result_str_ = "Reached goal with SUCCESS!!!!";
                        result_state_ = true;
                        ROS_INFO_NAMED(logger_name_, "Reached goal with SUCCESS :) ");
                    }

                    ROS_INFO_STREAM_NAMED(logger_name_, "erro_dist is: " << erro_dist << " meters | tolerance: " << current_tolerance_d); // use tolerance_d_last_point
                    ROS_INFO_STREAM_NAMED(logger_name_, "erro_angle is: " << to_degrees(delta_angle) << " degrees | tolerance: " << to_degrees(current_tolerance_yaw)); //use tolerance_yaw_last_point
                    flag_target_received_ = false;
                    plan_received_ = false;
                    //print = false;
                    pub_velocities = true; //publish velocity

                    control_done_ = true;
                    running_action_ = false;
                    //ROS_DEBUG_STREAM("Control finished: " << control_done_);
                }
                break; //STOP

            case PID:
                if (fabs(erro_dist) > current_tolerance_d) // not within tolerance
                {
                    double u_cmd = compute_control_comands(current_theta, target2.yaw);

                    if (target_is_in_front(erro_theta2))
                    {
                        v_sign = 1; //go forward
                    }
                    else // if target is in the back
                    {
                        v_sign = -1; //go backwards
                    }

                    if ((fabs(NormalizeAngle(erro_theta2)) < error_deviation) || (fabs(NormalizeAngle(erro_theta2-M_PI)) < error_deviation)) //if going straight - "Full speed"
                    {
                        scale = 1;
                    }
                    else //if on "curves", scale down speed
                    {
                        scale = scale_on_curves;
                    }
                    linearVel = max_linear_vel*v_sign*scale;

                    //limit v aqui (tem de ser maior que o min)
                    angularVel = u_cmd;
                    pub_velocities = true; //publish velocity
                }
                else // within tolerance //(fabs(erro_dist) < tolerance_d)
                {
                    compute_next_target_ = true;
                    // state_goto = CHOOSE_STATE; //(no need - already does this in compute_next_target
                    flag_target_received_ = false;
                }
                break; //PID

            case DE_ACCEL: // robot is close to goal or is about to get to a "curve"
                if (((fabs(erro_dist) >current_tolerance_d) || (fabs(delta_angle) > current_tolerance_yaw)) || obstacle_in_warning_zone_  )
                {
                    double u_cmd = compute_control_comands(current_theta, target2.yaw);

                    if (target_is_in_front(erro_theta2))
                    {
                        v_sign = 1; //go forward
                    }
                    else
                    {
                        v_sign = -1; //go backwards
                    }

                    if (fabs(NormalizeAngle(erro_theta2)) < error_deviation ||(fabs(NormalizeAngle(erro_theta2-M_PI)) < error_deviation))// || deaccel_before_target(target2, target3)) //if going straight - "Full speed"
                    {
                        scale = 0.5;
                    }
                    else //if on "curves", scale down speed
                    {
                        scale = scale_on_curves; //como achar este valor?
                    }

                    /* int K=1;
                        linearVel = max_linear_vel*v_sign*scale*K*fabs(erro_dist);*/
                    linearVel = max_linear_vel*v_sign*scale;
                    angularVel = u_cmd;
                    pub_velocities = true; //publish velocity
                }
                else //reached last goal
                {
                    //compute_next_target_ = true;
                    state_goto_ = CHOOSE_STATE;
                    flag_target_received_ = false;
                }
                break; //DE_ACCEL

                /*case IDLE: //ToDo
                    pub = false; // don't publish velocity
                    break; //IDLE:*/
            }
        } // if plan or pose received


        //if too close to an obstacle
        if(   (flag_obst_front && (linearVel > 0)) ||
              (flag_obst_back && (linearVel < 0))  ||
              ((flag_obst_front_left || flag_obst_back_right) && ((angularVel > 0) && fabs(angularVel) > 0.05)) ||
              ((flag_obst_front_right || flag_obst_back_left) && ((angularVel < 0) && fabs(angularVel) > 0.05)))
        {
            ROS_WARN_NAMED(logger_name_, "Obstacle detected - setting velocity to 0!!");
            pub_state_ = OBSTACLE;

            linearVel = 0;
            angularVel = 0;

            if (!obstacle_timer_started_)
            {
                obstacle_timer_.start();
                obstacle_timer_started_ = true;
            }
        }
        else
            if (obstacle_timer_started_)
            {
                obstacle_timer_.stop();
                obstacle_timer_started_ = false;
            }

        if (limit_velocity && pub_velocities)
        {
            if ( fabs(angularVel) > max_angular_vel) // limit w
            {
                ROS_DEBUG_NAMED("velocities", "limiting velocity");
                limit_w(angularVel);
                linearVel = k_r*angularVel;
            }
           // else
             //   std::cout << "*Vel: (" << linearVel << " | " << angularVel << ")" << RESET << std::endl;
        }


        if (pub_velocities) //publish velocity - prevents it from always publishing
        {
            geometry_msgs::Twist commandToSend;
            commandToSend.linear.x=linearVel;
            if(std::fabs(angularVel)>max_angular_vel_hard){
                angularVel=max_angular_vel_hard*angularVel/fabs(angularVel);
            }
            commandToSend.angular.z=angularVel;
            FCommandVelPub->SendData(commandToSend);
            state_goto_ = CHOOSE_STATE;
            ROS_DEBUG_STREAM_NAMED("velocities", "pub v: " << linearVel << " | w: " << angularVel);

            if(true){
                FDebugNextTargetPub->SendData(target2.x, target2.y, target2.yaw, global_frame_id);
                double delta_angle_aux=delta_angle*180.0/M_PI;
                FDebugErrorThetaPub->SendData(delta_angle_aux);
                double erro_theta2_aux=erro_theta2*180.0/M_PI;
                FDebugErrorTheta2Pub->SendData((erro_theta2_aux));
                FDebugErrorDistancePub->SendData(erro_dist);
            }
        }
    }
    else{
        FControllerStatePub->SendData(QString::number(NO_LOC).toStdString()); // send to topic which state was chosen
        ROS_WARN_NAMED(logger_name_, "Get pose failed!");
        if (enable_pub_){ //send a cmd to stop - prevent from sending continuosly when robot is lost
            linearVel = 0;
            angularVel = 0;

            geometry_msgs::Twist commandToSend;
            commandToSend.linear.x=linearVel;
            commandToSend.angular.z=angularVel;
            FCommandVelPub->SendData(commandToSend);
            enable_pub_ = 0;
            ROS_WARN_NAMED(logger_name_, "Stoped publishing velocities until pose is restored");
        }
    }

}



//front scan callback
void TOEAController::FrontLaserScanSubCallBack()
{
    sensor_msgs::LaserScan laserScan;
    FFrontLaserScanSub->GetData(laserScan);

    double ang_step = (laserScan.angle_max - laserScan.angle_min)/laserScan.ranges.size();
    int i =0;
    double act_ang=laserScan.angle_min + i*ang_step;
    double dist_test;

    flag_obst_front = flag_obst_front_left = flag_obst_front_right = false;
    obstacle_in_warning_front_ = false;

    if (always_show_protective_laser_)
        show_laser_front_ = true;
    else
        show_laser_front_ = false;

    while (act_ang < laserScan.angle_max - ang_step/2)
    {
        // PROTECTIVE LASERS
        if (fabs(act_ang) > FRONT_CORNER_ANG) // lateral distance
        {
            dist_test = fabs(LAT_DIST/sin(act_ang));
        }
        else // forward distance
        {
            dist_test = fabs(FRONT_FWD_DIST/cos(act_ang));
        }

        if (laserScan.ranges[i] < dist_test)
        {
            if (fabs(act_ang) < FRONT_CORNER_ANG)
            {
                flag_obst_front = true;
                show_laser_front_ = true;
            }
            else
            {
                if (act_ang > 0)
                {
                    flag_obst_front_left = true;
                    show_laser_front_ = true;
                }
                else
                {
                    flag_obst_front_right = true;
                    show_laser_front_ = true;
                }
            }
        }

        // WARM AREA LASERS (CLOSE TO OBSTACLES)

        if (fabs(act_ang) > FRONT_WARN_CORNER_ANG) // lateral distance
        {
            dist_test = fabs(WARN_LAT_DIST/sin(act_ang));
        }
        else // forward distance
        {
            dist_test = fabs(FRONT_WARN_FWD_DIST/cos(act_ang));
        }

        if (laserScan.ranges[i] < dist_test)
        {
            obstacle_in_warning_front_ = true;
        }

        i++;
        act_ang=laserScan.angle_min + i*ang_step;
    }
}

//back scan callback
void TOEAController::BackLaserScanSubCallBack()
{
    sensor_msgs::LaserScan laserScan;
    FBackLaserScanSub->GetData(laserScan);

    double ang_step = (laserScan.angle_max - laserScan.angle_min)/laserScan.ranges.size();
    int i =0;
    double act_ang=laserScan.angle_min + i*ang_step;
    double dist_test;

    flag_obst_back = flag_obst_back_left = flag_obst_back_right = false;
    obstacle_in_warning_back_ = false;

    if (always_show_protective_laser_)
        show_laser_back_ = true;
    else
        show_laser_back_ = false;

    while (act_ang < laserScan.angle_max - ang_step/2)
    {
        // PROTECTIVE LASERS
        if (fabs(act_ang) > BACK_CORNER_ANG) //lateral distance
        {
            dist_test = fabs(LAT_DIST/sin(act_ang));
        }
        else  // forward distance
        {
            dist_test = fabs(BACK_FWD_DIST/cos(act_ang));
        }

        if (laserScan.ranges[i] < dist_test)
        {
                if (fabs(act_ang) < BACK_CORNER_ANG)
                {
                    flag_obst_back = true;
                    show_laser_back_ = true;
                }
                else
                {
                    if (act_ang > 0)
                    {
                        flag_obst_back_right = true;
                        show_laser_back_ = true;
                    }
                    else
                    {
                        flag_obst_back_left = true;
                        show_laser_back_ = true;
                    }
                }
        }

        // WARM AREA LASERS (CLOSE TO OBSTACLES)

        if (fabs(act_ang) > BACK_WARN_CORNER_ANG) //lateral distance
        {
            dist_test = fabs(WARN_LAT_DIST/sin(act_ang));
        }
        else  // forward distance
        {
            dist_test = fabs(BACK_WARN_FWD_DIST/cos(act_ang));
        }

        if (laserScan.ranges[i] < dist_test)
        {
            if (laserScan.ranges[i] < dist_test)
            {
                obstacle_in_warning_back_ = true;
            }
        }

        i++;
        act_ang=laserScan.angle_min + i*ang_step;
    }
}

// algorithm works with poseStamped (1) or nav_msgs/Path (2):
//(1) poseStamped:
void TOEAController::PoseGoalSubCallBack()
{
    running_action_ = false;
    flag_target_received_ = true; //received new target (one pose only)
    geometry_msgs::PoseStamped poseGoal;
    FPoseGoalSub->GetData(poseGoal); //get pose

    tf::Quaternion quat(poseGoal.pose.orientation.x, poseGoal.pose.orientation.y, poseGoal.pose.orientation.z, poseGoal.pose.orientation.w);
    double yaw = getYaw(quat);

    ROS_INFO_NAMED(logger_name_, "[TOPIC] Pose Goal Received: x=%f, y=%f, th=%f (degrees)", poseGoal.pose.position.x, poseGoal.pose.position.y, to_degrees(yaw));

    target2.x = poseGoal.pose.position.x;
    target2.y = poseGoal.pose.position.y;
    target2.yaw = yaw;
    flag_theta = true;

    state_goto_ = CHOOSE_STATE;
    previous_error = 0;

    is_last_target_ = true; // to test DE_ACCEL
    compute_next_target_ = false;
}

//(2) nav_msgs/Path:
void TOEAController::planCallback(const oea_msgs::Oea_path path_msg)
{
    running_action_ = false;
    // always stop the robot when received another path
    stop_robot("a new plan was received");

    /*if (send_markers)
    {
        if (n_poses>0)
        {
            delete_zones_array(2*n_poses); //because we're printing 2 zones (red and green)
            markers_zone_pub.publish(marker_zones_array_);
        }
    }*/

    //update n_poses for new path
    n_poses = path_msg.path.poses.size();

    if (n_poses == 0)
    {
        ROS_WARN_NAMED(logger_name_, "[TOPIC] Path received has 0 poses!");
        plan_received_ = false;
        return;
    }

    ROS_INFO_STREAM_NAMED(logger_name_, "[TOPIC] New Global plan received with " << n_poses << " poses");

    global_plan = path_msg; // copy msg to global variable

    plan_received_ = true; //received new plan (n poses)
    pose_index = 0;
    compute_next_target_ = true;
    is_last_target_ = false;
    is_first_target_ = true;
    state_goto_ = CHOOSE_STATE;

    time_next_target_ = ros::Time::now();

    if (send_markers)
     marker_zones_array_.markers.clear();

    for (int i=0; i<n_poses; i++)
    {
        geometry_msgs::PoseStamped target;
        target = global_plan.path.poses[i];

        tf::Quaternion quat(target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w);
        double yaw = getYaw(quat);

        target2.x = target.pose.position.x;
        target2.y = target.pose.position.y;
        target2.yaw = yaw;

        uint8_t cost;
        cost = global_plan.cost[i];
        double tol = tolerance_d - cost * ((tolerance_d-tolerance_d_last_point)/4);


        if (send_markers) //@planCallback
        {
            if (i==n_poses-1)
            {
                send_zones(target2.x, target2.y, tolerance_d_last_point, 0xff0000 , marker_zones_array_, "/map");
                send_zones(target2.x, target2.y, 2*tolerance_d_last_point, 0xff0000, marker_zones_array_, "/map");
            }
            else
            {
                int col;
                if (cost == 0)
                    col = 0x00ff00; // green
                if (cost == 1)
                    col = 0xFFFF66; // yellow
                if (cost == 2)
                    col = 0xFF6600; // orange
                if (cost == 3)
                    col = 0xFF0066; // hot pink
                if (cost == 4)
                    col = 0xff0000; // red

                send_zones(target2.x, target2.y, tol, 0x3399FF, marker_zones_array_, "/map");
                send_zones(target2.x, target2.y, 2*tol, col , marker_zones_array_, "/map");
            }
        }
    }
    pub_zones_markers_ = true;
    marker_deaccel_array_.markers.clear();
}

//retrieve next target to follow from the original plan
void TOEAController::GetNextTargetFromPlan(int next)
{

    if ((pose_index > n_poses-1) ||(next > n_poses-1))
    	return;

    ROS_DEBUG_STREAM_NAMED(logger_name_, "Getting next target: next is #" << next);

    geometry_msgs::PoseStamped target;
    target = global_plan.path.poses[next];

    tf::Quaternion quat(target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w);
    double yaw = getYaw(quat);

    //target2 is immediate next target to follow
    target2.x = target.pose.position.x;
    target2.y = target.pose.position.y;
    target2.yaw = yaw;

    //target_n is a generic next target, somewhere in the path
    pose target_n;
    target_n = target2; //now target_n is immediate next

    if (next < n_poses-1) // only group same orientation goals if not close to walls
    {
        while(target_n.yaw == target2.yaw) //while the next goals have the same orientatin
        {
            // lets check the goal after maintains the orientation
            next++;

            target = global_plan.path.poses[next];
            tf::Quaternion quat_n(target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w);
            yaw = getYaw(quat_n);
            target_n.x = target.pose.position.x;
            target_n.y = target.pose.position.y;
            target_n.yaw = yaw;

            if(target_n.yaw == target2.yaw)
            {
                target2 = target_n;

                if(pose_index == n_poses-1)
                {
                    return;
                }
                pose_index++;
                target3 = target2;

                if (pose_index >= n_poses-1)
                {
                    break;
                }
            }
            else
            {
                target3 = target2;
                break;
            }
        }
    }

   ROS_DEBUG_STREAM_NAMED(logger_name_, "Next target is: #" << pose_index << ": (" << target2.x << ", " << target2.y << ", " << to_degrees(target2.yaw)<< ")");

   /* //for "feedfoward"
    if (pose_index < n_poses-1)
    {
        target = global_plan.poses[next+1];

        tf::Quaternion quat2(target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w);
        yaw = getYaw(quat2);

        target3.x = target.pose.position.x;
        target3.y = target.pose.position.y;
        target3.yaw = yaw;
    }
*/
    if (send_markers)
    {

        if ((pose_index==n_poses-1) || obstacle_in_warning_zone_ )// is last target
        {
            current_tolerance_d = tolerance_d_last_point;
            current_tolerance_yaw = tolerance_yaw_last_point;
        }
        else
        {
            current_tolerance_d = tolerance_d;
            current_tolerance_yaw = tolerance_yaw;
        }

        if (obstacle_in_warning_zone_)
        {
            if (markers_zone_pub_.getTopic()!="")
            {
                int id = 2*pose_index+1;
             //   update_zone(id,target2.x, target2.y, current_tolerance_d, 0x509ddd, marker_zones_array_, "/map");
              //  update_zone(id+1,target2.x, target2.y, 2*current_tolerance_d, 0xffff00, marker_zones_array_, "/map");
               // markers_zone_pub_.publish(marker_zones_array_);
            }
        }
    }
}

//PD controller
bool TOEAController::deaccel_before_target(pose t2, pose t3)
{
    double angle = NormalizeAngle(t3.yaw-t2.yaw);

    if (angle != 0)
    {
        //send_zones(current_x, current_y, 0.02, 0xFF3300, marker_deaccel_array_, "/map"); //orange
        if (markers_deaccel_pub_.getTopic()!="")
            markers_deaccel_pub_.publish(marker_deaccel_array_);

        return TRUE;
    }
    else
        return FALSE;

}

double TOEAController::d_to_next_target(pose next_target)
{
    double d = CalcDist(current_x,current_y, next_target.x,next_target.y);

    if (d < deaccel_distance_thresh )
    {
        // send yellow markers when d to next target < distance_threshold
        if (markers_deaccel_pub_.getTopic()!="")
        {
            //send_zones(current_x, current_y, 0.02, 0xFFFF00, marker_deaccel_array_, "/map"); //yellow
            markers_deaccel_pub_.publish(marker_deaccel_array_);
        }
    }

    return d;


}

//PD controller
double TOEAController::compute_control_comands(float theta_curr, float theta_des)
{
    double theta2target = atan2(target2.y-current_y, target2.x-current_x);
    double erro_theta2 = NormalizeAngle(theta2target-current_theta);
    double  error, erro_dist; // = NormalizeAngle(erro_theta2);*/

    if (!(target_is_in_front(erro_theta2))) // target is behind current pose
    {
        theta2target = atan2(current_y-target2.y, current_x-target2.x);
        erro_theta2 = NormalizeAngle(theta2target-current_theta);
        // error = NormalizeAngle(erro_theta2);
    }

  /*  // TARGET3 is target after "next_target" -> For feedforward
    double theta2target3 = atan2(target3.y-current_y, target3.x-current_x);
    double erro_theta3 = NormalizeAngle(theta2target3-current_theta);

    if (!(target_is_in_front(erro_theta3))) // target is behind current pose
    {
        theta2target3 = atan2(current_y-target3.y, current_x-target3.x);
        erro_theta3 = NormalizeAngle(theta2target3-current_theta);
    }
*/
    erro_dist = CalcDist(current_x,current_y, target2.x,target2.y);

    if ((use_yaw_on_middle_points_) || is_last_target_)
    {
        if (fabs(erro_dist) < 2*tolerance_d)
        {
            error = NormalizeAngle(theta_des - theta_curr);
            FControllerZoneStatePub->SendData("orientation"); // send to topic which zone
        }
        else
        {
            error = NormalizeAngle(erro_theta2);
            FControllerZoneStatePub->SendData("distance"); // send to topic which zone
        }
    }
    else
        error = NormalizeAngle(erro_theta2);


    double d_error = error - previous_error;
    previous_error = error;

    double u = Kp * error + Kd * d_error;// + Kff * erro_theta3;

    return u;
}


bool TOEAController::target_is_in_front(double angle)
{
    if (fabs(angle) < M_PI_2) //target is on front of current point
        return true;
    else                        //target is behind the current point
        return false;
}

// choose if should go to next target based on the errors
bool TOEAController::go_next_target(double erro_dist, double erro_theta2)
{
    bool result_dist = erro_dist < tolerance_d; // if within distance tolerance
    erro_theta2=std::fabs(NormalizeAngle(erro_theta2));

    bool result_theta2=(erro_theta2 > tolerance_erro_theta2) && (erro_theta2 < (M_PI-tolerance_erro_theta2));

    // if already passed the target in the orientation of the motion
    bool passed_target = (next_target_is_in_front && erro_theta2 > M_PI_2 ) || (!next_target_is_in_front && erro_theta2 < M_PI_2 );
    return (result_dist || result_theta2 || passed_target);
}

void TOEAController::limit_v(double& v)
{
    if ( (fabs(v)) > max_linear_vel)
    {
        v = sign(v)*max_linear_vel;
    }
    else
    {
        if ((fabs(v)) < min_linear_vel)
        {
            v = sign(v)*min_linear_vel;
        }
    }
}

void TOEAController::limit_w(double& w)
{
    if ((fabs(w)) > max_angular_vel)
    {
        w = sign(w)*max_angular_vel;
    }
}

void TOEAController::send_zones(float wx, float wy, float threshold, int color, visualization_msgs::MarkerArray& marker_array,  const std::string  frame)
{

    visualization_msgs::Marker marker;

    marker.header.frame_id = frame.c_str();
    marker.header.stamp = ros::Time::now();

    marker.ns = "zones_namespace";
    marker.id = ++marker_id_zones;

    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = threshold; // diameter in x direction
    marker.scale.y = threshold; // diameter in y direction
    marker.scale.z = 0.1;//height

    float r,g,b;
    r = ((color >> 16) & 0xFF) / 255.0;
    g = ((color >> 8) & 0xFF) / 255.0;;
    b = ((color) & 0xFF) / 255.0;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.8;

    marker.pose.position.x = wx;
    marker.pose.position.y = wy;
    marker.pose.position.z = 0;
    // marker.lifetime = ros::Duration(10);
    marker_array.markers.push_back(marker);
}

void TOEAController::update_zone(int id, float wx, float wy, float threshold, int color, visualization_msgs::MarkerArray& marker_array,  const std::string  frame)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame.c_str();
    marker.header.stamp = ros::Time::now();

    marker.ns = "zones_namespace";
    marker.id = id;

    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = threshold; // diameter in x direction
    marker.scale.y = threshold; // diameter in y direction
    marker.scale.z = 0.1;//height

    float r,g,b;
    r = ((color >> 16) & 0xFF) / 255.0;
    g = ((color >> 8) & 0xFF) / 255.0;;
    b = ((color) & 0xFF) / 255.0;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.3;

    marker.pose.position.x = wx;
    marker.pose.position.y = wy;
    marker.pose.position.z = 0;
    // marker.lifetime = ros::Duration(10);
    marker_array.markers.push_back(marker);
}

void TOEAController::send_cube_array(float wx, float wy, visualization_msgs::MarkerArray& marker_array,  const std::string  frame, int color)
{

    double fwd_dist, lat_dist, corner_ang;


    if (color == 1) //protective
    {
        if (frame == front_laser_link_)
        {
            fwd_dist = FRONT_FWD_DIST;
            corner_ang = FRONT_CORNER_ANG;
        }
        else
        {
            fwd_dist = BACK_FWD_DIST;
            corner_ang = BACK_CORNER_ANG;
        }

        lat_dist = LAT_DIST;

    }
    else //close
    {
        if (frame == front_laser_link_)
        {
            fwd_dist = FRONT_WARN_FWD_DIST;
            corner_ang = FRONT_WARN_CORNER_ANG;
        }
        else
        {
            fwd_dist = BACK_WARN_FWD_DIST;
            corner_ang = BACK_WARN_CORNER_ANG;
        }

        lat_dist = WARN_LAT_DIST;
    }


    if (count > 100)
    {
        count = 0;
        marker_array.markers.clear();
    }

    visualization_msgs::Marker points;

    points.header.frame_id = frame.c_str();
    points.header.stamp = ros::Time::now();

    points.ns = "laser_namespace";
    points.id = ++marker_id_laser;
    points.type = visualization_msgs::Marker::POINTS; // shape;
    points.action = visualization_msgs::Marker::ADD;

    points.scale.x = 0.01;
    points.scale.y = 0.01;

    switch (color)
    {
    case 1:
        points.color.r = 1.0;
        points.color.g = 0.0;
        points.color.b = 0.0;
        break;
    case 2:
        points.color.r = 1.0;
        points.color.g = 1.0;
        points.color.b = 0.0;
        break;
    case 3:
        points.color.r = 0.0;
        points.color.g = 1.0;
        points.color.b = 1.0;
        break;
    }


    points.color.a = 1.0;

    //float ang_max = M_PI_2-CORNER_ANG;
    float ang_max = corner_ang;

    float x = wx;
    float y = wy;

    geometry_msgs::Point p;
    for (float i=-ang_max; i< (ang_max); i=i+0.2)
    {
        x = fwd_dist;
        y = fwd_dist*tan(i);

        p.x = x;
        p.y = y;
        p.z = 0;
        points.points.push_back(p); //start point

        points.lifetime = ros::Duration(0.2);

        marker_array.markers.push_back(points);
        count++;
    }

    float x1 = -lat_dist*tan(to_radians(30));

    for (float i=x1; i< fwd_dist; i=i+0.1)
    {
        x = i;
        y = lat_dist;

        p.x = x;
        p.y = y;
        p.z = 0;
        points.points.push_back(p); //start point

        p.x = x;
        p.y = -y;
        p.z = 0;
        points.points.push_back(p); //start point

        points.lifetime = ros::Duration(0.2);
        marker_array.markers.push_back(points);
        count++;
    }



 }

void TOEAController::controller_command_call_back()
{
     std::string data;
     FControllerCommandSub->GetData(data);
     if(data=="idle"){
         //condicao para parar
         state_goto_ = EMERGENCY;
     }
}


void TOEAController::serverCB(const mission_ctrl_msgs::hardware_stateConstPtr& server_msg)
{
    //want to know if server is alive when running actions
}


void TOEAController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if (    (joy->axes[0] == 1) || (joy->axes[0] == -1) ||
            (joy->axes[1] == 1) || (joy->axes[1] == -1) ||
            (joy->axes[2] == 1) || (joy->axes[2] == -1) ||
            (joy->axes[3] == 1) || (joy->axes[3] == -1) ||
            (joy->axes[4] == 1) || (joy->axes[4] == -1) ||
            (joy->axes[5] == 1) || (joy->axes[5] == -1) ||
            (joy->buttons[0] == 1) || (joy->buttons[0] == -1) ||
            (joy->buttons[1] == 1) || (joy->buttons[1] == -1) ||
            (joy->buttons[2] == 1) || (joy->buttons[2] == -1) ||
            (joy->buttons[3] == 1) || (joy->buttons[3] == -1) ||
            (joy->buttons[4] == 1) || (joy->buttons[4] == -1) ||
            (joy->buttons[5] == 1) || (joy->buttons[5] == -1) ||
            (joy->buttons[6] == 1) || (joy->buttons[6] == -1) ||
            (joy->buttons[7] == 1) || (joy->buttons[7] == -1) ||
            (joy->buttons[8] == 1) || (joy->buttons[8] == -1) ||
            (joy->buttons[9] == 1) || (joy->buttons[9] == -1) ||
            (joy->buttons[10] == 1) || (joy->buttons[10] == -1) ||
            (joy->buttons[11] == 1) || (joy->buttons[11] == -1))
    {
        state_goto_ = EMERGENCY;
        //ROS_DEBUG("Robot stopped with joystick");
    }
 }


 void TOEAController::paramsCB(oea_controller::ctrl_paramsConfig &config, uint32_t level)
 {
     ROS_INFO_NAMED(logger_name_, "Changed parameters!");

     //update parameters:
//GAINS:
     Kp = config.Kp;
     Kd = config.Kd;
     //Kff = config.Kff;
//VELOCITY
     max_linear_vel = config.max_linear_vel;
     max_angular_vel = config.max_angular_vel;
//TOLERANCE
     tolerance_d = config.tolerance_d;
     tolerance_yaw = to_radians(config.tolerance_yaw_deg);

     use_yaw_on_middle_points_ = config.use_yaw_on_middle_points;
//protective/PROTECTIVE LASER
     double tmp;
     tmp = config.forward_protective_distance;
     FRONT_FWD_DIST = tmp + front_laser_offset;
     BACK_FWD_DIST = tmp + back_laser_offset;
     FRONT_CORNER_ANG = atan(LAT_DIST/FRONT_FWD_DIST);
     BACK_CORNER_ANG = atan(LAT_DIST/BACK_FWD_DIST);
     LAT_DIST = config.lateral_protective_distance;
//WARNING LASER
     tmp  = config.forward_warning_distance;
     FRONT_WARN_FWD_DIST = tmp + front_laser_offset;
     BACK_WARN_FWD_DIST = tmp + back_laser_offset;
     FRONT_WARN_CORNER_ANG = atan(WARN_LAT_DIST/FRONT_WARN_FWD_DIST);
     BACK_WARN_CORNER_ANG = atan(WARN_LAT_DIST/BACK_WARN_FWD_DIST);
	 WARN_LAT_DIST = config.lateral_warning_distance;

     //timer_msecs = config.timer_rate;
     //FMainTimer->setInterval(timer_msecs); // change timer period

     tolerance_d_last_point = config.tolerance_d_last_point;
     tolerance_yaw_last_point = to_radians(config.tolerance_yaw_deg_last_point);

     k_r = config.k_r;

     error_deviation = to_radians(config.error_deviation_deg);
     next_target_timeout = config.next_target_timeout;
     deaccel_distance_thresh = config.deaccel_distance_thresh;

     delta_angle_stuck_th = config.delta_angle_stuck_th;
     erro_theta2_stuck_th_low = config.erro_theta2_stuck_th_low;
     erro_theta2_stuck_th_sup = config.erro_theta2_stuck_th_sup;

     obstacle_timeout = config.obstacle_timeout;
     obstacle_timer_.setPeriod(ros::Duration(obstacle_timeout));

     always_show_protective_laser_ = config.always_show_protective_laser;
     //TODO:rosout all this

 }



 void TOEAController::PoseGoalAction(const geometry_msgs::PoseStamped poseGoal)
 {
     flag_target_received_ = true; //received new target (one pose only)

     tf::Quaternion quat(poseGoal.pose.orientation.x, poseGoal.pose.orientation.y, poseGoal.pose.orientation.z, poseGoal.pose.orientation.w);
     double yaw = getYaw(quat);

     ROS_INFO_NAMED(logger_name_, "[ACTION] Pose Goal Received: x=%f, y=%f, th=%f (degrees)", poseGoal.pose.position.x, poseGoal.pose.position.y, to_degrees(yaw));

     target2.x = poseGoal.pose.position.x;
     target2.y = poseGoal.pose.position.y;
     target2.yaw = yaw;
     flag_theta = true;

     state_goto_ = CHOOSE_STATE;
     previous_error = 0;

     is_last_target_ = true; // to test DE_ACCEL
     compute_next_target_ = false;
     control_done_ = false;

}

bool TOEAController::processActionPlan(const oea_msgs::Oea_path path_msg)
{
     running_action_ = true;
     // always stop the robot when received another path
     //stop_robot("a new plan was received"); //no need, already stopped when goal received

     if (send_markers)
    {
        if (n_poses>0)
        {
            delete_zones_array(2*n_poses); //because we're printing 2 zones (red and green)
            //markers_zone_pub.publish(marker_zones_array_);
        }
    }

     n_poses = path_msg.path.poses.size();

     if (n_poses == 0)
     {
         ROS_WARN_NAMED(logger_name_, "[Action] Path received has 0 poses!");
         plan_received_ = false;
         return false;
     }

     ROS_INFO_STREAM_NAMED(logger_name_, "[ACTION] New Global plan received with " << n_poses << " poses" );

     global_plan = path_msg; // copy msg to global variable
     pose_index = 0;
     compute_next_target_ = true;
     is_last_target_ = false;
     is_first_target_ = true;
     state_goto_ = CHOOSE_STATE;

     time_next_target_ = ros::Time::now();

     if (send_markers)
         marker_zones_array_.markers.clear();

     for (int i=0; i<n_poses; i++)
     {
         geometry_msgs::PoseStamped target;
         target = global_plan.path.poses[i];

         tf::Quaternion quat(target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w);
         double yaw = getYaw(quat);

         target2.x = target.pose.position.x;
         target2.y = target.pose.position.y;
         target2.yaw = yaw;

         uint8_t cost;
         cost = global_plan.cost[i];
         double tol = tolerance_d - cost * ((tolerance_d-tolerance_d_last_point)/4);

         if (send_markers) //@processActionPlan
         {
             if (i == n_poses-1)
             {
                 send_zones(target2.x, target2.y, tolerance_d_last_point, 0x00ff00, marker_zones_array_, "/map");
                 send_zones(target2.x, target2.y, 2*tolerance_d_last_point, 0xff0000, marker_zones_array_, "/map");
             }
             else
             {
                 int col;
                 if (cost == 0)
                     col = 0x00ff00; // green
                 if (cost == 1)
                     col = 0xFFFF66; // yellow
                 if (cost == 2)
                     col = 0xFF6600; // orange
                 if (cost == 3)
                     col = 0xFF0066; // hot pink
                 if (cost == 4)
                     col = 0xff0000; // red

                 send_zones(target2.x, target2.y, tol, 0x3399FF, marker_zones_array_, "/map");
                 send_zones(target2.x, target2.y, 2*tol, col , marker_zones_array_, "/map");
             }
         }
     }
     pub_zones_markers_ = true;
     plan_received_ = true; //received new plan (n poses)
     control_done_ = false;
     marker_deaccel_array_.markers.clear();
     return true;
}

void TOEAController::stop_robot(std::string str)
{
    plan_received_ = false;
    flag_target_received_ = false;
    ROS_WARN_STREAM_NAMED(logger_name_, "Stopping the robot because: " << str);

    geometry_msgs::Twist commandToSend;
    commandToSend.linear.x=0;
    commandToSend.angular.z=0;
    FCommandVelPub->SendData(commandToSend);
    pub_state_ = STOP;

}

bool TOEAController::force_stop_robot(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    //cancel goal
    if (running_action_)
    {
        cancel_action("Called service to stop the robot");
    }
    else
    {
        ROS_WARN_STREAM_NAMED(logger_name_, "Called service to stop the robot");
        stop_robot("service was called");
    }

    return true;
}


void TOEAController::delete_zones_array(int n_zones)
{
   // marker_zones_array_.markers.clear();
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time().now();

    marker.ns = "zones_namespace";

    marker.type = visualization_msgs::Marker::CYLINDER; // shape;

    for (int i=0; i< n_zones; i++)
    {
        marker.id = i+1; //markers start with 1;
        marker.action = visualization_msgs::Marker::DELETE;
        marker_zones_array_.markers.push_back(marker);
    }

    marker_id_zones = 0;
}

void TOEAController::obstacle_timerCB(const ros::TimerEvent& event)
{
    if (running_action_)
    {
        obstacle_timer_.stop();
        cancel_action("Immovable obstacle in the way of the robot!");
    }
    else
        stop_robot("there's an immovable obstacle in the way of the robot");
}

}// namespace

