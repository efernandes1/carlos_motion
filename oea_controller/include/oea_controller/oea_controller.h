#ifndef OEA_CONTROLLER_H
#define OEA_CONTROLLER_H

#include <QObject>
#include <qt_ros_interface/qros_tf.h>
#include <qt_ros_interface/qros_geometry_msgs_twist.h>
#include <qt_ros_interface/qros_sensor_msgs_laser_scan.h>
#include <qt_ros_interface/qros_geometry_msgs_pose_stamped.h>
#include <qt_ros_interface/qros_std_msg_string.h>
#include <qt_ros_interface/qros_std_msg_float64.h>
#include <QTimer>
#include <math.h>
//#include <angles/angles.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <dynamic_reconfigure/server.h>
#include <oea_controller/ctrl_paramsConfig.h>
#include <mission_ctrl_msgs/hardware_state.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Empty.h>
#include <ros/console.h>
#include <oea_msgs/Oea_path.h>


#define ROTATE 0
#define ROTATE_AND_GO 0
#define GO_STRAIGHT 1


//states
#define CHOOSE_STATE 0
#define PID 1
#define DE_ACCEL 2
#define STOP 5
#define EMERGENCY 10
//#define IDLE 3
#define NO_LOC 30 // robot not localized
#define OBSTACLE 20 // obstacle in front of the robot

#define deg_5 0.0872664626
#define deg_10 0.174532925
#define deg_170 2.96705973


#define color_path 0x0000FF //blue

inline double NormalizeAngle(double val){return std::atan2(std::sin(val),std::cos(val));}
inline double to_degrees(double radians) {return radians*(180.0/M_PI);}
inline double to_radians(double degs) {return degs*(M_PI/180.0);}

//Compute x, y, theta position based on velocity.
inline double computeNewXPosition(double xi, double vx, double vy, double theta, double dt)
    { return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;}

inline double computeNewYPosition(double yi, double vx, double vy, double theta, double dt)
    { return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;}

inline double computeNewThetaPosition(double thetai, double vth, double dt)
    { return thetai + vth * dt;}

//compute velocity based on acceleration
inline double computeNewVelocity(double vg, double vi, double a_max, double dt)
{   if((vg - vi) >= 0)
        return std::min(vg, vi + a_max * dt);
    return std::max(vg, vi - a_max * dt);}

inline int sign(float a) {
    if (a < 0)
        return -1;
    else return 1;
}

struct point {
    double x, y, z; //e: coord for each grid cell
};

struct pose {
    double x, y, yaw;
};

namespace oea_controller{

class TOEAController : public QObject
{
    Q_OBJECT
public:
    TOEAController(ros::NodeHandle nodeHandle);
    ~TOEAController();
    void PoseGoalAction(const geometry_msgs::PoseStamped poseGoal);
    bool processActionPlan(const oea_msgs::Oea_path path_msg);
    void stop_robot(std::string str);

    bool control_done_, result_state_;
    std::string result_str_;
    uint8_t hw_state_;
    //std_msgs::UInt8 hw_state;
    //depois substituir pelo tipo de msg certo?
    ros::Timer obstacle_timer_;
    bool running_action_;
    std::string front_laser_link_, back_laser_link_;
    double deaccel_distance_thresh;
    int delta_angle_stuck_th, erro_theta2_stuck_th_low, erro_theta2_stuck_th_sup;
    double front_laser_offset, back_laser_offset;
    std::string logger_name_;
private:
    QTimer* FMainTimer;
    qt_ros_interface::TQrosTfPoseSub* FTfPoseSub;
    qt_ros_interface::TQrosGeometryMsgsTwistPub* FCommandVelPub;
    qt_ros_interface::TQrosSensorMsgsLaserScanSub* FFrontLaserScanSub;
    qt_ros_interface::TQrosSensorMsgsLaserScanSub* FBackLaserScanSub;
    //qt_ros_interface::TQrosGeometryMsgsPoseStampedSub* FPositionGoalSub;
    qt_ros_interface::TQrosGeometryMsgsPoseStampedSub* FPoseGoalSub;
    qt_ros_interface::TQrosStdMsgsStringPub* FControllerStatePub;
    qt_ros_interface::TQrosStdMsgsStringSub* FControllerCommandSub;
    qt_ros_interface::TQrosGeometryMsgsPoseStampedPub* FDebugNextTargetPub;
    qt_ros_interface::TQrosStdMsgsFloat64Pub* FDebugErrorThetaPub;
    qt_ros_interface::TQrosStdMsgsFloat64Pub* FDebugErrorTheta2Pub;
    qt_ros_interface::TQrosStdMsgsFloat64Pub* FDebugErrorDistancePub;
    qt_ros_interface::TQrosStdMsgsStringPub* FControllerZoneStatePub;
    ros::Subscriber Path_sub_; //TODO
    ros::Publisher marker_protective_laser_pub_; //markers for protective laser area
    ros::Publisher marker_warning_laser_pub_; //markers for warning laser area
    ros::Publisher markers_zone_pub_;
    ros::Publisher markers_deaccel_pub_;
    std::string global_frame_id, base_frame_id, plan_topic;
    ros::Subscriber joy_sub_;
    ros::Subscriber server_sub_;
    ros::Publisher robot_pose_pub_;
    int state_goto_;
    bool debug_, debug_vel_, always_show_protective_laser_, show_laser_front_, show_laser_back_;
   // double x_target, y_target, theta_target;
    //double target_x, target_y, target_theta, target_x3, target_y3, target_theta3,;
    bool flag_theta;
    bool flag_target_received_;
    bool flag_obst_front_left, flag_obst_front, flag_obst_front_right, flag_obst_back_left, flag_obst_back, flag_obst_back_right;
    bool enable_pub_; // prevents the program from continuously send 0 cmds when robot is not localized
    double CalcDist(double x1, double y1, double x2, double y2 );
    double max_angular_vel, max_angular_vel_hard, max_linear_vel, min_angular_vel, min_linear_vel;
    double tolerance_d, tolerance_erro_theta2, tolerance_x, tolerance_y, tolerance_yaw, DE_ACCEL_threshold;
    double current_x, current_y, current_theta; //current_yaw
    visualization_msgs::MarkerArray marker_array_arrows;
    visualization_msgs::MarkerArray marker_array_protective_laser, marker_array_warning_laser;
    visualization_msgs::MarkerArray marker_zones_array_;
    visualization_msgs::MarkerArray marker_deaccel_array_;
    int marker_id_zones, marker_id_laser;
   // std::vector<int> plan_poses;
    oea_msgs::Oea_path global_plan;
    int n_poses, pose_index;
    bool plan_received_, compute_next_target_, is_last_target_, is_first_target_;
    double Kp, Kd, Ki, previous_error, i_error, Kff;
    bool limit_velocity, keep_ratio, obstacle_detection, obstacle_timer_started_;
    ros::Time time_next_target_;
    int next_target_timeout, obstacle_timeout;
    double error_deviation, FRONT_CORNER_ANG, BACK_CORNER_ANG, LAT_DIST, FRONT_FWD_DIST, BACK_FWD_DIST;
    double FRONT_WARN_CORNER_ANG, BACK_WARN_CORNER_ANG, WARN_LAT_DIST, FRONT_WARN_FWD_DIST, BACK_WARN_FWD_DIST;
    int pub_state_, count;
    bool send_markers, pub_emergency_errors, stop_with_joystick;
    bool next_target_is_in_front;
    double scale_on_curves; //percentage of scale on curves
    double k_r; // curvature radius (v/w)
    pose target2, target3; // target2 is next target to follow, target3 is target after target2
    bool use_yaw_on_middle_points_;
    bool pub_zones_markers_;
    dynamic_reconfigure::Server<ctrl_paramsConfig> params_server;
    dynamic_reconfigure::Server<ctrl_paramsConfig>::CallbackType f;
    bool obstacle_in_warning_front_, obstacle_in_warning_back_, obstacle_in_warning_zone_;
    int timer_msecs_;
    double tolerance_d_last_point, tolerance_yaw_last_point, current_tolerance_d, current_tolerance_yaw;
    void obstacle_timerCB(const ros::TimerEvent& event);
    void serverCB(const mission_ctrl_msgs::hardware_stateConstPtr& server_msg);
    ros::ServiceServer service_server;
    bool force_stop_robot(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool cancel_action(std::string);
private Q_SLOTS:
    void MainTimerCallBack();
    void FrontLaserScanSubCallBack();
    void BackLaserScanSubCallBack();
    void PoseGoalSubCallBack();
    void planCallback(const oea_msgs::Oea_path path_msg);
    void GetNextTargetFromPlan(int next);
    void send_zones(float wx, float wy, float threshold, int color, visualization_msgs::MarkerArray& marker_array,  const std::string  frame);
    void paramsCB(oea_controller::ctrl_paramsConfig &config, uint32_t level);
    void delete_zones_array(int n_zones);
    void update_zone(int id, float wx, float wy, float threshold, int color, visualization_msgs::MarkerArray& marker_array,  const std::string  frame);
    void limit_v(double& v);
    void limit_w(double& w);
    void send_cube_array(float wx, float wy, visualization_msgs::MarkerArray& marker_array, const std::string frame, int color);
    void controller_command_call_back();
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);    
    bool deaccel_before_target(pose t2, pose t3);
    bool target_is_in_front(double erro_theta2);
    bool go_next_target(double erro_dist, double erro_theta2);
    double d_to_next_target(pose next_target);
    double compute_control_comands(float x_curr, float x_des);

};

}

#endif // OEA_CONTROLLER_H
