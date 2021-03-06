
// hlk byrm @ 2014

#include "commclient.h"
#include "rosThread.h"

#include <QThread>
#include <ros/ros.h>
#include <QApplication>

int main(int argc,char** argv)
{

    ros::init(argc,argv,"monitoringISLH");


    QApplication app(argc,argv);

    CommClient *commclient = new CommClient();

    RosThread* rosthread = new RosThread(commclient);

    commclient->setRosthread(rosthread);

    QThread thr;
    QThread thr2;

    rosthread->moveToThread(&thr);

    commclient->moveToThread(&thr2);

    QObject::connect(rosthread,SIGNAL(rosFinished()),&thr,SLOT(quit()));

    QObject::connect(&thr,SIGNAL(finished()),&app,SLOT(quit()));

    QObject::connect(&thr,SIGNAL(finished()),rosthread,SLOT(deleteLater()));

    QObject::connect(&thr,SIGNAL(started()),rosthread,SLOT(work()));

    thr.start();
    thr2.start();


    return app.exec();


}

