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
#include "ISLH_msgs/targetPoseListMessage.h"
#include <std_msgs/UInt8.h>
#include <create_node/TurtlebotSensorState.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8MultiArray.h>
#include "ISLH_msgs/taskInfo2MonitorMessage.h"
#include <geometry_msgs/Point.h>


#define CAPACITY 16.5
#define  DANGEROUS 13.2

class RosThread;
struct taskProp{
    QString taskUUID;
    uint encounteringTime; // in timestamp - "the time when the task is encountered"
    int responsibleUnit;  // "who is responsible for the task"
    uint encounteringRobotID;  // "Id of the robot encountering the task"
    uint handlingDuration; // in seconds - "the required time to handle the task"
    uint timedOutDuration; // "the timed-out duration for the task"
    int status; // "status of the task"
    uint startHandlingTime; // in timestamp - "the time when the task starts being handled"
    geometry_msgs::Point pose; // the location of the task
    QString requiredResourcesString;
};

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
    ros::Subscriber taskInfoSub;
    ros::Subscriber leaderInfoSub;
    ros::Publisher targetPosePublisher;
    ros::Publisher startMissionPublisher;
    ros::Timer timer;
    int battery;

    QVector<taskProp> tasks;

    int numOfRobots;

    int readConfigFile(QString filename);

    void robotPoseCallback(const ISLH_msgs::robotPose::ConstPtr &msg);
    void robotInfoCallbackKobuki(const kobuki_msgs::SensorState::ConstPtr &msg);
    void robotInfoCallbackTurtlebot(const create_node::TurtlebotSensorState::ConstPtr &msg);
    void robotConnCallback(const std_msgs::String::ConstPtr &msg);
    void taskInfoCallback(const ISLH_msgs::taskInfo2MonitorMessage::ConstPtr &msg);
    void leaderInfoCallback(const std_msgs::Int8MultiArray::ConstPtr &msg);
    void timerTick(const ros::TimerEvent&);

    int batteryLevel();

signals:
public slots:
    void receiveData();
private slots:
    void displaySocketError(QAbstractSocket::SocketError socketError);
};

#endif // COMMCLIENT_H
