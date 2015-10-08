#include "oea_planner/oea_planner_ros.h"
#include <ros/assert.h>

sensor_msgs::PointCloud2 pcd;

// constructor
TOea_Planner::TOea_Planner(ros::NodeHandle &n, ros::NodeHandle &private_n, std::string logger_name) : Astar_(logger_name)//: plan_("/move_platform", n)
{
    logger_name_ = logger_name;
    ros::start();

    //load parameters:
    private_n.param<std::string>("global_frame_id", global_frame_id, "map");
    private_n.param<std::string>("base_frame_id", base_frame_id, "base_footprint");
    private_n.param("inflate_map_borders", Astar_.inflate_map_borders_, false);
    private_n.param("robot_x_size", Astar_.robot_x_size_, 1.2);//1.2
    private_n.param("robot_y_size", Astar_.robot_y_size_, 0.8);//0.8
    private_n.param("allow_unknown", Astar_.allow_unknown_, true);//0.8
    private_n.param("stop_at_exact_target", Astar_.stop_at_exact_target_, false);
    private_n.param("publish_entire_pcd", Astar_.publish_entire_pcd_, false);//0.8
    private_n.param("use_frontal_laser", Astar_.use_frontal_laser, true);
    private_n.param("use_back_laser", Astar_.use_back_laser, true);
    private_n.param("use_localization", use_localization, true);
    private_n.param("mark_end_cubes", mark_end_cubes, true);


    //TODO: DEBUG THIS
    //print parameters
   /* if (true)
    {
        std::cout << BOLDMAGENTA << " Parameters: " << RESET << std::endl;
        std::cout << BOLDWHITE   << "  * global_frame_id: " << RESET << global_frame_id << std::endl;
        std::cout << BOLDWHITE   << "  * base_frame_id: " << RESET << base_frame_id <<  std::endl;
        std::cout << BOLDWHITE   << "  * robot_x_size: " << RESET << Astar_.robot_x_size_ <<  std::endl;
        std::cout << BOLDWHITE   << "  * robot_y_size: " << RESET << Astar_.robot_y_size_ <<  std::endl;
        std::cout << BOLDWHITE   << "  * inflate_map_borders: " << RESET << Astar_.inflate_map_borders_ <<  std::endl;
        std::cout << BOLDWHITE   << "  * allow_unknown_: " << RESET << Astar_.allow_unknown_ <<  std::endl;
        std::cout << BOLDWHITE   << "  * stop_at_exact_target: " << RESET << Astar_.stop_at_exact_target_ << std::endl;
        std::cout << BOLDWHITE   << "  * publish_entire_pcd: " << RESET << Astar_.publish_entire_pcd_ << std::endl;
        std::cout << BOLDWHITE   << "  * use_frontal_laser: " << RESET << Astar_.use_frontal_laser << std::endl;
        std::cout << BOLDWHITE   << "  * use_back_laser: " << RESET << Astar_.use_back_laser <<  std::endl;
        std::cout << BOLDWHITE << " >>>>>>>>>>>>>>>>>< level_closest: " << Astar_.level_closest << RESET << std::endl;
        std::cout << BOLDWHITE << " >>>>>>>>>>>>>>>>>< level_middle: " << Astar_.level_middle << RESET << std::endl;
        std::cout << BOLDWHITE << " >>>>>>>>>>>>>>>>>< level_farthest: " << Astar_.level_farthest << RESET << std::endl;
        std::cout << BOLDWHITE <<" ****************************************" << RESET << std::endl;
    }*/


    // subscribers:
    goal_sub_ = n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &TOea_Planner::goalCB, this);
    start_pose_sub_ = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/start_pose", 1, &TOea_Planner::start_poseCB, this);
    map_sub_ = n.subscribe<nav_msgs::OccupancyGrid>("/map", 100, &TOea_Planner::mapCB, this);

    // publishers:
    pcd_pub_ =  private_n.advertise<sensor_msgs::PointCloud2>("inflation_cloud", 1, true); // inflation point cloud (latched)
    oea_path_pub_ = private_n.advertise<oea_msgs::Oea_path>("oea_plan",1); //publishing this topic causes the controller to start
    visual_path_pub_ = n.advertise<nav_msgs::Path>("/plan",1, true); //latched topic, same as for the actions -> this way we can have only one topic for visualization
    arrows_pub_ = private_n.advertise<visualization_msgs::MarkerArray>("arrows_marker_array", 1, true);
    state_pub_ = private_n.advertise<std_msgs::UInt8>("state", 1);
    cells_pub_ = private_n.advertise<visualization_msgs::MarkerArray>("cells_marker_array", 1, true);
    // services:
    ss_ = private_n.advertiseService("IsPoseValid", &TOea_Planner::IsPoseValid, this);

    map_received_ = false;

}

// destructor
TOea_Planner::~TOea_Planner()
{
    ros::shutdown();
}

int TOea_Planner::exec()
{
    while (ros::ok())
    {
        ros::spin();
    }
    return 0;
}



// service to check if some pose is valid
// is valid if in free or high cost zone
// not valid if obstacle, inflated or outside the map
bool TOea_Planner::IsPoseValid(oea_planner::isPoseValid::Request& req, oea_planner::isPoseValid::Response& res)
{
    ROS_DEBUG_NAMED(logger_name_, "IsPoseValid Service Called");

    if (!map_received_)
    {
        ROS_ERROR_NAMED(logger_name_, "No map received yet. Can't validate pose!");
        return false;
    }

    int x,y,yaw;
    Astar_.ConvertWorlCoordToMatrix(req.x, req.y, req.yaw, x, y, yaw);

    int state = Astar_.GetGridCellState(x,y,yaw);

    if ((state == AStarObstacle)||(state == AStarInflated)|| (state == AStarInvalid))
    {
        ROS_DEBUG_NAMED(logger_name_, "Invalid pose");
        res.isValid = false;
    }
    else
    {
        ROS_DEBUG_NAMED(logger_name_, "Pose is valid");
        res.isValid = true;
    }
    return true;
}

void TOea_Planner::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
{
    if (!map_received_)
    {
        ROS_ERROR_NAMED(logger_name_, "No map received yet. Unable to compute path.");
        return;
    }

    planner_state.data = hardware::BUSY; //PLANNING;
    state_pub_.publish(planner_state);

    ROS_DEBUG_NAMED(logger_name_, "New Goal received on topic");

    // get world pose from msg
    Astar_.goal_world_pose_.x = goal_msg->pose.position.x;
    Astar_.goal_world_pose_.y = goal_msg->pose.position.y;
    Astar_.goal_world_pose_.yaw = tf::getYaw(goal_msg->pose.orientation);

    oea_msgs::Oea_path oea_path; // plan variable
    // oea_path.path.poses.clear();

    //check validity:
    if (Astar_.goal_world_pose_.yaw!=Astar_.goal_world_pose_.yaw) //if nan
    {
        ROS_ERROR_STREAM_NAMED(logger_name_, "Invalig Goal: yaw is " << to_degrees(Astar_.goal_world_pose_.yaw));
        //else just publish the blank path

        oea_path_pub_.publish(oea_path); //publishing 0 poses will cause the controller to stop following the previous path
        visual_path_pub_.publish(oea_path.path); // publish nav_msgs/Path to view path on rviz

        planner_state.data = hardware::IDLE;
        state_pub_.publish(planner_state);

        return;
    }

    //convert it to grid coord
    Astar_.ConvertWorlCoordToMatrix(Astar_.goal_world_pose_.x, Astar_.goal_world_pose_.y, Astar_.goal_world_pose_.yaw , Astar_.goal_grid_pose_.x, Astar_.goal_grid_pose_.y, Astar_.goal_grid_pose_.z);

    std::string error_str;

    // check if goal is valid
    //error_str = "GoalCb: check grid pose val";
    if (Astar_.is_valid_point(Astar_.goal_grid_pose_,error_str))
    {
        //get path from the Astar...
        executeCycle(error_str, oea_path);
    }
    else
    {
        ROS_WARN_STREAM_NAMED(logger_name_, error_str);
        // no need to clear Grid, because Astar was not called
        // publish empty plan to stop the robot in case it's moving and received a new point...
        oea_path_pub_.publish(oea_path);
        visual_path_pub_.publish(oea_path.path);
    }

    planner_state.data = hardware::IDLE;
    state_pub_.publish(planner_state);

}


// start_poseCB Callback
void TOea_Planner::start_poseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start_msg)
{
   // Astar_.marker_id_ = 0;
    Astar_.marker_array_cells_.markers.clear();
    if (!map_received_)
    {
        ROS_ERROR_NAMED(logger_name_, "No map received yet. Unable to compute path.");
        return;
    }

    planner_state.data = hardware::BUSY; //PLANNING;
    state_pub_.publish(planner_state);

    ROS_DEBUG_NAMED(logger_name_, "New Goal received on topic");

    // get world pose from msg
    Astar_.robot_init_world_pose_.x = start_msg->pose.pose.position.x;
    Astar_.robot_init_world_pose_.y = start_msg->pose.pose.position.y;
    Astar_.robot_init_world_pose_.yaw = tf::getYaw(start_msg->pose.pose.orientation);

    int x, y, l;
    Astar_.ConvertWorlCoordToMatrix(Astar_.robot_init_world_pose_.x, Astar_.robot_init_world_pose_.y, Astar_.robot_init_world_pose_.yaw , x, y, l);
    if (mark_end_cubes) //mark start cell 
    {
        Astar_.add_cubes_array(x, y, 0x989898); // start
        cells_pub_.publish(Astar_.marker_array_cells_);
    }

	// print start pose info (coor and cost) to terminal
    int index = Astar_.Get_index(l,y,x, "SetGridCellCost(x,y,z)");
    std::cout << BOLDMAGENTA << "selected cell is: [l][h][w] = [" << l << "][" << y << "][" << x << "] , with cost: " << + Astar_.AStarMap_.Grid[index].Cost  << RESET <<  std::endl;


}

// Map Callback
void TOea_Planner::mapCB(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
    map_received_ = true;
    planner_state.data = hardware::BUSY;
    state_pub_.publish(planner_state);

    Astar_.world_map_.origin_yaw = tf::getYaw (map_msg->info.origin.orientation);

    ROS_ASSERT_MSG(Astar_.world_map_.origin_yaw == 0, "Can't handle rotated maps"); //if map is rotated, quit programm

    Astar_.world_map_.resolution = map_msg->info.resolution;
    Astar_.world_map_.height = map_msg->info.height;
    Astar_.world_map_.width = map_msg->info.width;
    Astar_.world_map_.origin_x = map_msg->info.origin.position.x;
    Astar_.world_map_.origin_y = map_msg->info.origin.position.y;
   // number_cells = world_map.width*world_map.height;

    Astar_.allocate();
    Astar_.get_robot_padded_footprint();
    Astar_.get_robot_high_cost_footprint(); // higher cost for cells close to obstacles

    sensor_msgs::PointCloud2 pcd;
    Astar_.SetGridFromMap(map_msg, pcd);
    pcd_pub_.publish(pcd);

    Astar_.AStarClear();
    planner_state.data = hardware::IDLE;
    state_pub_.publish(planner_state);
    ROS_INFO_NAMED(logger_name_, "Planner is ready to receive goals");
}

bool TOea_Planner::executeCycle(std::string &error_str, oea_msgs::Oea_path &planned_path)
{
    bool success = false;

    //get tf for robot location
    tf::TransformListener listener;
    tf::StampedTransform transform;

    if (use_localization)
    {
        try{
            listener.waitForTransform(global_frame_id, base_frame_id, ros::Time(0), ros::Duration(0.5) );
            listener.lookupTransform(global_frame_id, base_frame_id, ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR_NAMED(logger_name_, "%s",ex.what());
            error_str = "Robot is not localized!";
            ROS_ERROR_STREAM_NAMED(logger_name_, error_str);
            return success;
        }

        // set init pose from tf
        Astar_.robot_init_world_pose_.x = transform.getOrigin().x();
        Astar_.robot_init_world_pose_.y = transform.getOrigin().y();
        Astar_.robot_init_world_pose_.yaw = tf::getYaw(transform.getRotation());
    }
    /* else
    {
		use "2D pose estimate" or directly "/start_pose" topic to set a starting pose
        // check if already has a pose
    } */


    // and convert it to matrix coordinates
    Astar_.ConvertWorlCoordToMatrix(Astar_.robot_init_world_pose_, Astar_.robot_init_grid_pose_);

    Astar_.AStarMap_.InitialPoint.x = Astar_.robot_init_grid_pose_.x;
    Astar_.AStarMap_.InitialPoint.y = Astar_.robot_init_grid_pose_.y;
    Astar_.AStarMap_.InitialPoint.z = Astar_.robot_init_grid_pose_.z;

    Astar_.AStarMap_.TargetPoint.x = Astar_.goal_grid_pose_.x;
    Astar_.AStarMap_.TargetPoint.y = Astar_.goal_grid_pose_.y;
    Astar_.AStarMap_.TargetPoint.z = Astar_.goal_grid_pose_.z;

    Astar_.AStarMap_.ActualTargetPoint.x = Astar_.goal_grid_pose_.x;
    Astar_.AStarMap_.ActualTargetPoint.y = Astar_.goal_grid_pose_.y;
    Astar_.AStarMap_.ActualTargetPoint.z = Astar_.goal_grid_pose_.z;

    ros::Time time_init = ros::Time::now(); // to check how long it took to plan
    int number_cells = Astar_.world_map_.width*Astar_.world_map_.height*AStarDirCount; // # of cell in 3D map

    // delete arrows from previous path
    int n = Astar_.last_path_number_of_points_;
    Astar_.delete_arrows_array(n);

    if (mark_end_cubes) // mark goal point
    {
        Astar_.add_cubes_array(Astar_.goal_grid_pose_.x,Astar_.goal_grid_pose_.y, 0x909090);
    }
    cells_pub_.publish(Astar_.marker_array_cells_);

    success = Astar_.AStarGo(number_cells, error_str, planned_path);
    ros::Duration d = ros::Time::now()-time_init;
    ROS_DEBUG_STREAM_NAMED(logger_name_, "Planning took: " << d.toNSec()/1000000 << " miliseconds. ");

    arrows_pub_.publish(Astar_.marker_array_arrows_);
    //by publishing path here we don't need to publish it in move_server
    oea_path_pub_.publish(planned_path);
    visual_path_pub_.publish(planned_path.path);

    Astar_.AStarClear(); // clear open and closed cells
    return success;
}
