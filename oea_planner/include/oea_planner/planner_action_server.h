#ifndef PLANNER_ACTION_SERVER_H
#define PLANNER_ACTION_SERVER_H

# include <oea_planner/oea_planner_ros.h>
#include <actionlib/server/simple_action_server.h>
#include <oea_planner/planAction.h>
#include <ros/console.h>

#define IDLE 0;
#define BUSY 1;
//#define PLANNING 1;
#define ERROR 2;

class PlanAction
{
protected:
    TOea_Planner planner_ros_;
    oea_planner::planResult result_;
    std::string logger_name_;
public:
    actionlib::SimpleActionServer<oea_planner::planAction> as_;
    PlanAction(std::string name, ros::NodeHandle &n, ros::NodeHandle &private_n, std::string logger_name) : planner_ros_(n, private_n, logger_name),
        as_(n, name, boost::bind(&PlanAction::executeCB, this, _1), false) //, action_name_(name)
      {
        logger_name_ = logger_name;

        ROS_DEBUG_STREAM_NAMED(logger_name_, "Starting Planner Action Server: " << name);
        as_.start();
        ROS_DEBUG_STREAM_NAMED(logger_name_, "Planner Server started: " << name);
      }

      ~PlanAction(void)
      {
      }

    void executeCB(const oea_planner::planGoalConstPtr &goal_msg)
    {

        if (!planner_ros_.map_received_)
        {
            ROS_ERROR_NAMED(logger_name_, "No map received yet. Aborting!");
            result_.result_state = false;
            result_.error_string = "No navigation map received";
            as_.setAborted(result_);
            return;
        }

        planner_ros_.planner_state.data = BUSY;
        planner_ros_.state_pub_.publish(planner_ros_.planner_state);

        ROS_DEBUG_STREAM_NAMED(logger_name_, "Planner Received a New goal: #" << goal_msg->pose_goal.header.seq);
        std::string error_str;

        if (as_.isPreemptRequested())
        {
            error_str = "Preempt Request while planning!";
            ROS_WARN_STREAM_NAMED(logger_name_, error_str);

            result_.result_state = false;
            result_.error_string = error_str;

            as_.setPreempted(result_);

            planner_ros_.planner_state.data = IDLE; //ERROR; //MESMO QUE TENHA DADO ERRRO, ESTÁ PRONTO PARA RECEBER OUTRO
            planner_ros_.state_pub_.publish(planner_ros_.planner_state);
            return;
        }

        // get world pose from msg
        planner_ros_.Astar_.goal_world_pose_.x = goal_msg->pose_goal.pose.position.x;
        planner_ros_.Astar_.goal_world_pose_.y = goal_msg->pose_goal.pose.position.y;
        planner_ros_.Astar_.goal_world_pose_.yaw = tf::getYaw(goal_msg->pose_goal.pose.orientation);
	ROS_INFO_STREAM_NAMED("debug_valina", "YAW: "<< planner_ros_.Astar_.goal_world_pose_.yaw << " | deg: " << to_degrees(planner_ros_.Astar_.goal_world_pose_.yaw));

        if (planner_ros_.Astar_.goal_world_pose_.yaw!=planner_ros_.Astar_.goal_world_pose_.yaw) //if nan
        {
            error_str = "Invalig Goal: yaw is nan";
            ROS_ERROR_STREAM_NAMED(logger_name_, error_str << ": " << to_degrees(planner_ros_.Astar_.goal_world_pose_.yaw));
            //else just publish the blank path

            result_.result_state = false;
            result_.error_string = error_str;

            as_.setAborted(result_);

            planner_ros_.planner_state.data = IDLE; //ERROR; //MESMO QUE TENHA DADO ERRRO, ESTÁ PRONTO PARA RECEBER OUTRO
            planner_ros_.state_pub_.publish(planner_ros_.planner_state);
            return;
        }

        //convert it to grid coord
        planner_ros_.Astar_.ConvertWorlCoordToMatrix(planner_ros_.Astar_.goal_world_pose_, planner_ros_.Astar_.goal_grid_pose_);

        // check if goal is valid
        if (!planner_ros_.Astar_.is_valid_point(planner_ros_.Astar_.goal_grid_pose_, error_str))
        {
            ROS_ERROR_STREAM_NAMED(logger_name_, error_str);
            result_.result_state = false;
            result_.error_string = error_str;
            as_.setAborted(result_);
            planner_ros_.planner_state.data = IDLE; //ERROR; //MESMO QUE TENHA DADO ERRRO, ESTÁ PRONTO PARA RECEBER OUTRO
            planner_ros_.state_pub_.publish(planner_ros_.planner_state);
            //no need to clear Grid, because Astar was not called
            return;
        }
        else
        {
            nav_msgs::Path planned_path;

            result_.result_state = planner_ros_.executeCycle(error_str, planned_path);
            result_.error_string = error_str;
            result_.planned_path = planned_path;

            //set terminal status
            if (result_.result_state)
                as_.setSucceeded(result_);
            else
                as_.setAborted(result_);

            planner_ros_.planner_state.data = IDLE; //ERROR; //MESMO QUE TENHA DADO ERRRO, ESTÁ PRONTO PARA RECEBER OUTRO
            planner_ros_.state_pub_.publish(planner_ros_.planner_state);
            return;
        }

    }

    int spin()
    {
        while (ros::ok())
        {
            ros::spin(); //blocking
        }
        return 0;
    }

};

#endif // PLANNER_ACTION_SERVER_H

