/********************************************************************************************
*                                                                                           *
* move_platform_server is the link between mission controller and the controller + planner  *
*                                                                                           *
* Creates a client for each subsystem (planner + controller)                                *
* and converts move_goals into planner and controller goals                                 *
********************************************************************************************/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <mission_ctrl_msgs/movePlatformAction.h>
#include <oea_planner/planAction.h>
#include <oea_controller/controlPlatformAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

# include <std_msgs/String.h>
#include <mission_ctrl_msgs/hardware_state.h>
#include <mission_ctrl_msgs/mission_ctrl_defines.h>
#include <std_msgs/UInt8.h>
#include <ros/console.h>

class MovePlatformAction
{
protected:
    ros::NodeHandle n_;
    actionlib::SimpleActionServer<mission_ctrl_msgs::movePlatformAction> as_; // move platform action server
    actionlib::SimpleActionClient<oea_planner::planAction> ac_planner_; // planner action client
    actionlib::SimpleActionClient<oea_controller::controlPlatformAction> ac_control_; // controller action client

    //create msg for goals and results
    mission_ctrl_msgs::movePlatformGoal move_goal_;
    mission_ctrl_msgs::movePlatformResult move_result_; // result for the "move platform" action

    oea_planner::planGoal pose_goal_; // goal for the planner is a poseStamped
    oea_controller::controlPlatformGoal planned_path_goal_; // goal for the controller is output from the planner (nav_msgs/Path)
    ros::Publisher path_pub_, state_pub_; //path_pub_ is for visualization purposes only
    ros::Subscriber ctrl_state_sub, planner_state_sub; // know the state of each subsystem
    bool planning_, controlling_, set_terminal_state_, debug_;
    uint8_t control_state_, plan_state_;
    mission_ctrl_msgs::hardware_state hw_state_;
    std::string logger_name_;
public:
    MovePlatformAction() :
        as_(n_, CARLOS_MOVE_ACTION, false), //movePlatform action SERVER
        ac_planner_("/plan_action", true), // Planner action CLIENT
        ac_control_("/control_action", true) // Controller action CLIENT
    {

        n_.param("/move_platform_server/debug", debug_, false);

        std::string name = ROSCONSOLE_DEFAULT_NAME; //ros.carlos_motion_action_server
        name = name  + ".debug";
        logger_name_ = "debug";
        //logger is ros.carlos_motion_action_server.debug

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

        ROS_DEBUG_NAMED(logger_name_, "Starting Move Platform Server");

        as_.registerGoalCallback(boost::bind(&MovePlatformAction::moveGoalCB, this));
        as_.registerPreemptCallback(boost::bind(&MovePlatformAction::movePreemptCB, this));

        //start the move server
        as_.start();
        ROS_DEBUG_NAMED(logger_name_, "Move Platform Action Server Started");

        // now wait for the other servers (planner + controller) to start
        ROS_WARN_NAMED(logger_name_, "Waiting for planner server to start");
        ac_planner_.waitForServer();
        ROS_INFO_STREAM_NAMED(logger_name_, "Planner server started: " <<  ac_planner_.isServerConnected());

        ROS_WARN_NAMED(logger_name_, "Waiting for controller server to start");
        ac_control_.waitForServer();
        ROS_INFO_STREAM_NAMED(logger_name_, "Controller server started: " <<  ac_control_.isServerConnected());

        path_pub_ = n_.advertise<nav_msgs::Path>("/plan",1); //just to view the path

        n_.param("/carlos/fsm_frequency", frequency_, DEFAULT_STATE_FREQ);
        state_pub_timer_ = n_.createTimer(frequency_, &MovePlatformAction::state_pub_timerCB, this);
        state_pub_ = n_.advertise<mission_ctrl_msgs::hardware_state>(CARLOS_BASE_STATE_MSG,1);

        planning_ = false;
        controlling_ = false;
        //set_terminal_state_;
        ctrl_state_sub = n_.subscribe<std_msgs::String>("/oea_controller/controller_state", 5, &MovePlatformAction::control_stateCB, this);
        planner_state_sub = n_.subscribe<std_msgs::UInt8>("/oea_planner/state", 5, &MovePlatformAction::planner_stateCB, this);

    }

    ros::Timer state_pub_timer_;
    double frequency_;

    ~MovePlatformAction()
    {
        // Destructor: should cancell goals?
        // No need because controller stops when move_server dies
    }

    //publish state timer
    void state_pub_timerCB(const ros::TimerEvent& event)
    {
        if (ctrl_state_sub.getNumPublishers()==0)
        {
            hw_state_.state = hardware::ERROR;
            hw_state_.description = "Platform motion controller is down" ;
            state_pub_.publish(hw_state_);
            return;
        }

        if (planner_state_sub.getNumPublishers()==0)
        {
            hw_state_.state = hardware::ERROR;
            hw_state_.description = "Platform motion planner is down" ;
            state_pub_.publish(hw_state_);
            return;
        }

        if (plan_state_ == hardware::BUSY)
        {
            hw_state_.state = hardware::BUSY;
            hw_state_.description = "Robot is planning" ;
        }
        else
        {
            switch (control_state_)
            {
            case 0:
                hw_state_.state = hardware::IDLE;
                hw_state_.description = "Ready to receive a goal" ; // - What is zero?
                break;
            case 1:
                hw_state_.state = hardware::BUSY;
                hw_state_.description = "Robot is moving at normal speed" ;
                break;
            case 2:
                hw_state_.state = hardware::BUSY;
                hw_state_.description = "Robot is moving at slow speed" ;
                break;
            case 5:
                hw_state_.state = hardware::IDLE;
                hw_state_.description = "Ready to receive a goal" ;
                break;
            case 10:
                hw_state_.state = hardware::ERROR;
                hw_state_.description = "Robot is in EMERGENCY state" ;
                break;
            case 20:
                hw_state_.state = hardware::BUSY;
                hw_state_.description = "Some obstacle in the way of the robot" ;
                break;
            case 21:
                hw_state_.state = hardware::ERROR;
                hw_state_.description = "Immovable obstacle in the way of the robot" ;
                break;
            case 30:
                hw_state_.state = hardware::ERROR;
                hw_state_.description = "Localization NOT OK" ;
                break;
            }
        }

        state_pub_.publish(hw_state_);
        return;
    }


    // move platform server receives a new goal
    void moveGoalCB()
    {
        set_terminal_state_ = true;

        move_goal_.nav_goal = as_.acceptNewGoal()->nav_goal;
        ROS_INFO_STREAM_NAMED(logger_name_, "Received Goal #" <<move_goal_.nav_goal.header.seq);

        if (as_.isPreemptRequested() ||!ros::ok())
        {
            ROS_WARN_STREAM_NAMED(logger_name_, "Preempt Requested on goal #" << move_goal_.nav_goal.header.seq);
            if (planning_)
                ac_planner_.cancelGoalsAtAndBeforeTime(ros::Time::now());
            if (controlling_)
                ac_control_.cancelGoalsAtAndBeforeTime(ros::Time::now());
            move_result_.result_state = 0;
            move_result_.error_string = "Preempt Requested!!!";
            as_.setPreempted(move_result_);
            return;
        }

        // Convert move goal to plan goal
        pose_goal_.pose_goal = move_goal_.nav_goal;

        if (planner_state_sub.getNumPublishers()==0)
        {
            ROS_WARN_STREAM_NAMED(logger_name_, "Goal #" << move_goal_.nav_goal.header.seq << " not sent - planner is down");
            planning_ = false;
            move_result_.result_state = 0;
            move_result_.error_string = "Planner is down";
            as_.setAborted(move_result_);
        }
        else
        {
            ac_planner_.sendGoal(pose_goal_, boost::bind(&MovePlatformAction::planningDoneCB, this, _1, _2),
                                 actionlib::SimpleActionClient<oea_planner::planAction>::SimpleActiveCallback(),
                                 actionlib::SimpleActionClient<oea_planner::planAction>::SimpleFeedbackCallback());
            planning_ = true;
            ROS_DEBUG_STREAM_NAMED(logger_name_, "Goal #" << move_goal_.nav_goal.header.seq << " sent to planner");
        }
        return;
    }

    void planningDoneCB(const actionlib::SimpleClientGoalState& state, const oea_planner::planResultConstPtr &result)
    {
        planning_ = false;
        ROS_DEBUG_STREAM_NAMED(logger_name_, "Plan Action finished: " << state.toString());

        move_result_.result_state = result->result_state;

        if (move_result_.result_state) //if plan OK
        {
            planned_path_goal_.plan_goal = result->planned_path; // goal for the controller is result of the planner

            if (ctrl_state_sub.getNumPublishers()==0)
            {
                ROS_WARN_STREAM_NAMED(logger_name_, "Goal #" << move_goal_.nav_goal.header.seq << " not sent - Controller is down");
                controlling_ = false;
                move_result_.result_state = 0;
                move_result_.error_string = "Controller is down!!!";
                as_.setAborted(move_result_);
            }
            else
            {
                ac_control_.sendGoal(planned_path_goal_, boost::bind(&MovePlatformAction::ControlDoneCB, this, _1, _2),
                                     actionlib::SimpleActionClient<oea_controller::controlPlatformAction>::SimpleActiveCallback(),
                                     actionlib::SimpleActionClient<oea_controller::controlPlatformAction>::SimpleFeedbackCallback()); //boost::bind(&MovePlatformAction::ControlFeedbackCB, this,_1));
                controlling_ = true;
                ROS_DEBUG_STREAM_NAMED(logger_name_,"Goal #" << move_goal_.nav_goal.header.seq << " sent to Controller");
                path_pub_.publish(result->planned_path);
            }
        }
        else //if plan NOT OK
        {
            ac_control_.cancelGoalsAtAndBeforeTime(ros::Time::now());

            move_result_.error_string = "Planning Failed: " + result->error_string;
            ROS_WARN_STREAM_NAMED(logger_name_, "Aborting because " << move_result_.error_string);

            as_.setAborted(move_result_);
        }
        return;

    }

    void ControlDoneCB(const actionlib::SimpleClientGoalState& state, const oea_controller::controlPlatformResultConstPtr &result)
    {
        controlling_ = false;
        ROS_DEBUG_STREAM_NAMED(logger_name_, "Control Action finished: " << state.toString());

        move_result_.result_state = result->result_state;
        move_result_.error_string = result->error_string;

        if (move_result_.result_state)
        {
            as_.setSucceeded(move_result_);
            ROS_INFO_NAMED(logger_name_, "Goal was successful :)");
        }
        else
        {
             ROS_WARN_NAMED(logger_name_, "Goal was NOT successful :)");

            // if is preempted => as_ was already set, cannot set again
            if (state.toString() != "PREEMPTED")
            {
                as_.setAborted(move_result_);
                ROS_DEBUG_NAMED(logger_name_, "Goal was Aborted");
            }
            else
            {
                if (set_terminal_state_)
                {
                     as_.setPreempted(move_result_);
                     ROS_DEBUG_NAMED(logger_name_, "Goal was Preempted");
                }
            }
        }
    }

    void movePreemptCB()
    {
        ROS_WARN_STREAM_NAMED(logger_name_, "Preempt Requested");

        if (planning_)
        {
            ROS_DEBUG_NAMED(logger_name_, "Planning - cancelling all plan goals");
            ac_planner_.cancelGoalsAtAndBeforeTime(ros::Time::now());
        }
        if (controlling_)
        {
            ROS_DEBUG_NAMED(logger_name_, "Controlling - cancelling all ctrl goals");
            ac_control_.cancelGoalsAtAndBeforeTime(ros::Time::now());
        }

        move_result_.result_state = 0;
        move_result_.error_string = "Move Platform Preempt Request!";
        as_.setPreempted(move_result_);
        set_terminal_state_ = false;

        return;
    }

    //know which state the controller is
    void control_stateCB(const std_msgs::String::ConstPtr& msg)
    {
        //using topic instead of feedback allows to have info on the node before receiving any goal
        control_state_ = (uint8_t) atoi(msg->data.c_str());
    }

    //know which state the planner is
    void planner_stateCB(const std_msgs::UInt8::ConstPtr& msg)
    {
        plan_state_ = msg->data;
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "move_platform_server");

    MovePlatformAction move_server;

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        //ros::spin();
    }

    return 0;
}
