#include <oea_planner/planner_action_server.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "oea_planner");

    ros::NodeHandle n;
    ros::NodeHandle private_n("~");

    bool debug;
    private_n.param("debug", debug, true);

    if (debug) //output debug
    {
        if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
            ros::console::notifyLoggerLevelsChanged();
    }

    ROS_DEBUG("*** Starting oea_planner (main)");
    PlanAction plan_action("/plan_action", n, private_n);
    int result = plan_action.spin();

    return result;
}
