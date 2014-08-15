#include <tf/tf.h>
#include "commclient.h"
#include <QtNetwork/QHostInfo>
#include <QtGui/QMessageBox>
#include <QtCore/QString>
#include <string>
#include <sstream>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <qjson/parser.h>
#include <QDir>
#include <QFile>
#include <fstream>

#include "rosThread.h"

CommClient::CommClient(QObject* parent) :
    QObject(parent)
{
    connected = false;
    QString path = QDir::homePath();
    path.append("/ISL_workspace/src/configISL.json");

    robotID = 0;

    if( !readConfigFile(path) ){
        qDebug()<< "Read Config File Failed!!!";
    }
    else
    {
       qDebug() << "robotID is: "<< robotID;
    }

    socket = new QTcpSocket(this);
    socket->setReadBufferSize(0);
    connect(socket, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(displaySocketError(QAbstractSocket::SocketError)));
    qDebug() << "Host IP is: " << IP;
    socket->connectToHost(IP,11000);
    if(socket->waitForConnected(5000)){
        qDebug()<<"Connected";
        connected = true;
    }
    else
        qDebug()<<"Error timeout";
}

void CommClient::setRosthread(RosThread* rosthread){
    this->rosthread = rosthread;

    timer = this->rosthread->n.createTimer(ros::Duration(1),&CommClient::timerTick,this);
    timer.start();

    if(isKobuki)
        this->robotInfoSub = this->rosthread->n.subscribe("/mobile_base/sensors/core",1,&CommClient::robotInfoCallbackKobuki,this);
    else
        this->robotInfoSub = this->rosthread->n.subscribe("/mobile_base/sensors/core",1,&CommClient::robotInfoCallbackTurtlebot,this);
    this->robotPoseSub = this->rosthread->n.subscribe("robot_position_info",1,&CommClient::robotPoseCallback,this);
    this->robotConnSub = this->rosthread->n.subscribe("robot_connection_info",1,&CommClient::robotConnCallback,this);
}

void CommClient::timerTick(const ros::TimerEvent&){
    if(!connected) return;

    QString message = "info$";
    message.append(QString::number((robotID)));
    message.append("$");
    message.append(QString::number((battery)));
    message.append("$");
    message.append(QString::number((batteryLevel())));
    message.append("<EOF>");

    QByteArray byteArray = message.toUtf8();
    socket->write(byteArray); //write the data itself
    socket->waitForBytesWritten();
    qDebug() << "written";
    qDebug() << message;
}

int CommClient::batteryLevel(){
    long battLevFull = -1;
    long battLevNow = -1;
    std::ifstream _if1("/sys/class/power_supply/BAT0/energy_full");
    std::ifstream _if2("/sys/class/power_supply/BAT0/energy_now");
    _if1 >> battLevFull;
    _if2 >> battLevNow;
    _if1.close();
    _if2.close();
    return (int)(battLevNow*100/battLevFull);
}

void CommClient::robotInfoCallbackKobuki(const kobuki_msgs::SensorState::ConstPtr &msg){
    if(!connected) return;

    float voltage = msg->battery/10.0;
    float capacity = 16.5;
    float dangerous = 13.2;
    float percent = ((95*(voltage-dangerous)) / (capacity-dangerous)) + 5;

    battery = (int)std::max(std::min(percent,100.0f),0.0f);
}
void CommClient::robotInfoCallbackTurtlebot(const create_node::TurtlebotSensorState::ConstPtr &msg){
    if(!connected) return;
    battery = msg->charge * 100 / msg->capacity;
}

void CommClient::robotPoseCallback(const monitoringISLH::robotPose::ConstPtr &msg){
    if(!connected) return;

    QString message = "pos$";
    message.append(QString::number(msg->id));
    message.append("$");
    message.append(QString::number(msg->position.x));
    message.append("&");
    message.append(QString::number(msg->position.y));
    message.append("$");
    message.append(QString::number(msg->target.x));
    message.append("&");
    message.append(QString::number(msg->target.y));
    message.append("$");
    message.append(QString::number(msg->radYaw));
    message.append("&");
    message.append(QString::number(msg->calYaw));
    message.append("<EOF>");

    QByteArray byteArray = message.toUtf8();
    socket->write(byteArray); //write the data itself
    socket->waitForBytesWritten();
    qDebug() << "written";
    qDebug() << message;
}

void CommClient::robotConnCallback(const std_msgs::String::ConstPtr &msg){
    QString message = "conn$";
    message.append(QString::number((robotID)));
    message.append("$");
    message.append(QString::fromStdString(msg->data));
    message.append("<EOF>");

    QByteArray byteArray = message.toUtf8();
    socket->write(byteArray); //write the data itself
    socket->waitForBytesWritten();
    qDebug() << "written";
    qDebug() << message;
}

// Displays socket error in a MessageBox
void CommClient::displaySocketError(QAbstractSocket::SocketError socketError){
    qDebug()<<"Socket Error!!!";
    return;
}
int CommClient::readConfigFile(QString filename)
{
    QFile file(filename);

    if(!file.exists()) return false;

    if(!file.open(QFile::ReadOnly)) return false;

    QJson::Parser parser;

    bool ok;

    QVariantMap result = parser.parse(&file,&ok).toMap();

    if(!ok){

        file.close();
        qDebug()<<"Fatal reading error";

        return false;
    }
    else
    {
        robotID = result["robotID"].toInt();
        qDebug()<<result["robotID"].toString();

        IP = result["IP"].toString();
        qDebug()<<result["IP"].toString();


        numOfRobots = result["numrobots"].toInt();
        qDebug()<<result["numrobots"].toString();

        isKobuki = result["isKobuki"].toInt() == 1;
        qDebug()<<result["isKobuki"].toString();
    }
    file.close();
    return true;
}

CommClient::~CommClient()
{


}
