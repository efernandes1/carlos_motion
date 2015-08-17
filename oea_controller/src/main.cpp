
#include <iostream>
#include <QCoreApplication>
#include <qt_ros_interface/qros_start.h>
#include <oea_controller/ctrl_action_server.h>

int main(int argc, char *argv[])
{
    std::cout<<"#################################################"<<std::endl;
    std::cout<<"Input args: num_strings: "<< argc <<std::endl;
    for(int i=0; i<argc; ++i){
        std::cout<<argv[i]<<" ";
    }
    std::cout<<std::endl<<"#################################################"<<std::endl;
    QCoreApplication* qtProg=new QCoreApplication(argc, argv);
    qt_ros_interface::TQRosStart* rosProg=new qt_ros_interface::TQRosStart(argc, argv, "oea_controller");

    ros::NodeHandle nodeHandle("~");

    ControlAction control("/control_action", nodeHandle);
    int proOut=qtProg->exec();
    if(rosProg) delete rosProg;
    if(qtProg) delete qtProg;
    return proOut;
}
