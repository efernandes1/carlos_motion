/********************************************************************************************
*                                                                                           *
*       move_platform_client replaces AAU's mission controller for testing                  *
*                                                                                           *
* Receives a goal on topic /nav_goal and sends a goal to move_platform_server               *                                                                                  *
********************************************************************************************/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <mission_ctrl_msgs/movePlatformAction.h> //include msgs from movePlatform
#include <geometry_msgs/PoseStamped.h>
#include <mission_ctrl_msgs/mission_ctrl_defines.h>


typedef actionlib::SimpleActionClient<mission_ctrl_msgs::movePlatformAction> Client;

bool send_goal = false, first_it = true;
mission_ctrl_msgs::movePlatformGoal move_goal;

// forward rviz goal to action msg
void goalCallback(const geometry_msgs::PoseStampedConstPtr& goal)
{
    move_goal.nav_goal.header = goal->header;
    move_goal.nav_goal.pose = goal->pose;
    send_goal = true;
    ROS_INFO_STREAM("**** New Client Move Goal (#" << move_goal.nav_goal.header.seq<< ")");
}

class ClientNode
{

public:
    ClientNode(): ac_(CARLOS_MOVE_ACTION, true)  // true -> don't need ros::spin()
    {
        ROS_WARN("*** Waiting for /move_platform server to start.");
        ac_.waitForServer();
        ROS_INFO_STREAM("*** /move_platform server STARTED: " << ac_.isServerConnected());

        goal_sub_ = n_.subscribe<geometry_msgs::PoseStamped>("nav_goal", 1, goalCallback); // get goal from topic (rviz)
    }

    ros::NodeHandle n_;

    void check_for_goals()
    {
        if (send_goal)
        {
            ROS_DEBUG_STREAM("*** Goal seq: " << move_goal.nav_goal.header.seq);

            //to cancel goals from the client
            if (!first_it) // this is needed to not check state before first goal (gives an error)
            {
                //if some goal is being tracked
                if (ac_.getState().toString() == "ACTIVE")
                {

                    ROS_WARN_STREAM("*** Goal #" << move_goal.nav_goal.header.seq-1 << " is still active - Cancelling goal ");
                    ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());
                    ac_.waitForResult(ros::Duration(5.0));

                    ROS_WARN_STREAM("*** Goal #" << move_goal.nav_goal.header.seq-1  << " state: " << ac_.getState().toString());
                }
            }
            else
                first_it = false;

            ac_.sendGoal(move_goal,
                         boost::bind(&ClientNode::doneCB, this, _1,_2),
                         boost::bind(&ClientNode::activeCB, this),//Client::SimpleActiveCallback()
                         boost::bind(&ClientNode::feedbackCB, this,_1)); //Client::SimpleFeedbackCallback());
            ROS_INFO_STREAM("*** Sent goal #" << move_goal.nav_goal.header.seq << " to move_platform_server");
            send_goal = false;
        }
    }


private:
    Client ac_;
    ros::Subscriber goal_sub_;

    void doneCB(const actionlib::SimpleClientGoalState& state, const mission_ctrl_msgs::movePlatformResultConstPtr result)
    {
        if (state.toString() == "SUCCEEDED")
            ROS_INFO_STREAM("*** Move Platform finished: " << state.toString());
        else
            ROS_WARN_STREAM("*** Move Platform finished: " << state.toString());

        bool result_state = result->result_state;
        std::string error = result->error_string;
         ROS_INFO_STREAM(" ** result_state: " << result_state);
         ROS_INFO_STREAM(" ** result_error: " << error);
    }

    // called whenever feedback is pubed
    void feedbackCB(const mission_ctrl_msgs::movePlatformFeedbackConstPtr feedback)
    {
      //  std::cout << "Feedback: " << +feedback->state << std::endl;

    }

   // called once on every goal, uppon acceptance
    void activeCB()
    {
        //std::cout << "************ active: " << std::endl;
    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_platform_client");

    ClientNode client;
    ros::NodeHandle nh;

    ROS_INFO("*** Move Platform Action Client Started");

    ros::Rate rate(10);
    while (ros::ok())
    {
        client.check_for_goals();

        ros::spinOnce();
        rate.sleep();
    }
}


