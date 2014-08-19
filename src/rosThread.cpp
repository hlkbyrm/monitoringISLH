#include "rosThread.h"
//#include <navigationISL/neighborInfo.h>
//#include "commclient.h"

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>


RosThread::RosThread()
{
    shutdown = false;

}
RosThread::RosThread(CommClient *currentcommclient)
{
    commclient = currentcommclient;
}

void RosThread::work(){

    if(!ros::ok()){
        emit rosStartFailed();
        return;
    }

    emit rosStarted();

    ros::Rate loop(30);

    while(ros::ok()){
        if(commclient!=NULL)
            commclient->sendWaitingMessages();


        ros::spinOnce();
        loop.sleep();
    }

    emit rosFinished();
}
void RosThread::shutdownROS()
{
    ros::shutdown();
}


