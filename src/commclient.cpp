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
    connect(socket,SIGNAL(readyRead()),this,SLOT(receiveData()));
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

    timer = this->rosthread->n.createTimer(ros::Duration(4),&CommClient::timerTick,this);
    timer.start();

    if(isKobuki)
        this->robotInfoSub = this->rosthread->n.subscribe("/mobile_base/sensors/core",1,&CommClient::robotInfoCallbackKobuki,this);
    else
        this->robotInfoSub = this->rosthread->n.subscribe("/mobile_base/sensors/core",1,&CommClient::robotInfoCallbackTurtlebot,this);
    this->robotPoseSub = this->rosthread->n.subscribe("navigationISLH/robotPositionInfo",1,&CommClient::robotPoseCallback,this);
    this->robotConnSub = this->rosthread->n.subscribe("communicationISLH/robotConnectionInfo",1,&CommClient::robotConnCallback,this);

    this->targetPosePublisher = this->rosthread->n.advertise<ISLH_msgs::targetPoseListMessage>("monitoringISLH/targetPoseList", 1000);
    this->startMissionPublisher = this->rosthread->n.advertise<std_msgs::UInt8>("monitoringISLH/targetPoseList", 1000);
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

    waitingMessages.push_back(message);
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
    float percent = ((95*(voltage-DANGEROUS)) / (CAPACITY-DANGEROUS)) + 5;

    battery = (int)std::max(std::min(percent,100.0f),0.0f);
}
void CommClient::robotInfoCallbackTurtlebot(const create_node::TurtlebotSensorState::ConstPtr &msg){
    if(!connected) return;
    battery = msg->charge * 100 / msg->capacity;
}

void CommClient::robotPoseCallback(const ISLH_msgs::robotPose::ConstPtr &msg){
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

    waitingMessages.push_back(message);
}

void CommClient::sendWaitingMessages(){
    if(waitingMessages.count() == 0) return;
    QStringList _waitingMessages = QStringList(waitingMessages);
    waitingMessages.clear();

    for(int i=0;i<_waitingMessages.count();i++){
        qDebug()<<"writing";
        QByteArray byteArray(_waitingMessages.at(i).toLatin1());
        socket->write(byteArray); //write the data itself
        qDebug() << _waitingMessages[i];
        //bool asd = socket->waitForBytesWritten(500);
        qDebug() << "written";
    }
}

void CommClient::robotConnCallback(const std_msgs::String::ConstPtr &msg){
    QString message = "conn$";
    message.append(QString::number((robotID)));
    message.append("$");
    message.append(QString::fromStdString(msg->data));
    message.append("<EOF>");

    waitingMessages.push_back(message);
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

void CommClient::receiveData(){
    recDataBA = socket->readAll();
    recData = QString::fromAscii(recDataBA);
    QStringList list = recData.split(";",QString::SkipEmptyParts);

    // Incoming data parts
    qDebug()<<"Number of incoming data parts"<<list.size();
    qDebug()<<list;

    if(list.size() > 1 && list.at(0) == "AA")
    {
        if(list.at(1) == "run"){
            std_msgs::UInt8 msg;
            msg.data = 1;
            startMissionPublisher.publish(msg);
        }
        else if(list.at(1) == "stop"){
            std_msgs::UInt8 msg;
            msg.data = 0;
            startMissionPublisher.publish(msg);

        }
        else if(list.size() == (1 + numOfRobots)){
            ISLH_msgs::targetPoseListMessage robotTargetMsg;
            for(int i = 1; i < list.size(); i++){
                qDebug()<<list[i]<<" "<<i;

                QStringList valsList = list[i].split(",",QString::SkipEmptyParts);
                qDebug()<< valsList;

                geometry_msgs::Pose2D robotPose;
                robotPose.x = valsList.at(0).toFloat();
                robotPose.y = valsList.at(1).toFloat();
                robotTargetMsg.robotIDs.push_back(i);
                robotTargetMsg.targetPoses.push_back(robotPose);
            }
            this->targetPosePublisher.publish(robotTargetMsg);
        }
    }

    recData.clear();
    recDataBA.clear();
}

CommClient::~CommClient()
{
}
