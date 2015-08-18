#include <oea_planner/planner_action_server.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "oea_planner");

    ros::NodeHandle n;
    ros::NodeHandle private_n("~");

    bool debug;
    private_n.param("debug", debug, true);

    std::string name = ROSCONSOLE_DEFAULT_NAME; //ros.oea_planner
    name = name  + ".planner";
    std::string logger_name = "planner";

    if (debug) //output debug
    {
        if(ros::console::set_logger_level(name, ros::console::levels::Debug))
            ros::console::notifyLoggerLevelsChanged();
    }
    else
    {
        if(ros::console::set_logger_level(name, ros::console::levels::Info)) //to avoid to much debug msg from actions
            ros::console::notifyLoggerLevelsChanged();
    }

    ROS_DEBUG_NAMED(logger_name, "Starting oea_planner");
    PlanAction plan_action("/plan_action", n, private_n, logger_name);
    int result = plan_action.spin();

    return result;
}
