#ifndef COMMCLIENT_H
#define COMMCLIENT_H

#include <QtNetwork/QtNetwork>
#include <QtCore/QByteArray>
#include <QObject>
#include <QXmlStreamReader>
#include <ros/timer.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <kobuki_msgs/SensorState.h>
#include "ISLH_msgs/robotPose.h"
#include <create_node/TurtlebotSensorState.h>
#include <std_msgs/String.h>


#define CAPACITY 16.5
#define  DANGEROUS 13.2

class RosThread;

class CommClient : public QObject
{
    Q_OBJECT
public:
    explicit CommClient(QObject *parent = 0);

   ~CommClient(void);

    // Connection request to the given address and port
    void connectToHost(QString hostAddress, quint16 port);
    void setRosthread(RosThread* rosthread);

    RosThread* rosthread;

    void sendWaitingMessages();
private:

    QTcpSocket* socket;

    QString recData;
    QByteArray recDataBA;

    QString hostName;
    QStringList waitingMessages;

    QAbstractSocket::SocketError clientSocketError;

    int robotID;
    QString IP;
    bool isKobuki;
    bool connected;

    ros::Subscriber robotPoseSub;
    ros::Subscriber robotInfoSub;
    ros::Subscriber robotConnSub;
    ros::Publisher targetPosePublisher;
    ros::Timer timer;
    int battery;


    int numOfRobots;

    int readConfigFile(QString filename);

    void robotPoseCallback(const ISLH_msgs::robotPose::ConstPtr &msg);
    void robotInfoCallbackKobuki(const kobuki_msgs::SensorState::ConstPtr &msg);
    void robotInfoCallbackTurtlebot(const create_node::TurtlebotSensorState::ConstPtr &msg);
    void robotConnCallback(const std_msgs::String::ConstPtr &msg);
    void timerTick(const ros::TimerEvent&);

    int batteryLevel();

signals:
public slots:
    void receiveData();
private slots:
    void displaySocketError(QAbstractSocket::SocketError socketError);
};

#endif // COMMCLIENT_H
