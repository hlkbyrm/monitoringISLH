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

    qRegisterMetaType<QAbstractSocket::SocketError>("SocketError");

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
        this->robotInfoSub = this->rosthread->n.subscribe("/mobile_base/sensors/core",queueSize,&CommClient::robotInfoCallbackKobuki,this);
    else
        this->robotInfoSub = this->rosthread->n.subscribe("/mobile_base/sensors/core",queueSize,&CommClient::robotInfoCallbackTurtlebot,this);
    this->robotPoseSub = this->rosthread->n.subscribe("navigationISLH/robotPositionInfo",queueSize,&CommClient::robotPoseCallback,this);
    this->robotConnSub = this->rosthread->n.subscribe("communicationISLH/robotConnectionInfo",queueSize,&CommClient::robotConnCallback,this);
    this->taskInfoSub = this->rosthread->n.subscribe("taskCoordinatorISLH/taskInfo2Monitor",queueSize,&CommClient::taskInfoCallback,this);
    this->leaderInfoSub = this->rosthread->n.subscribe("taskCoordinatorISLH/leaderIDInfo2Monitor",queueSize,&CommClient::leaderInfoCallback,this);
    this->coalInfoSub = this->rosthread->n.subscribe("coalitionLeaderISLH/coalStateInfo2Monitor",queueSize,&CommClient::coalInfoCallback,this);
    this->taskHandlerInfoSub = this->rosthread->n.subscribe("taskHandlerISLH/robotInfo2Monitor",queueSize,&CommClient::taskHandlerInfoCallback,this);

    this->targetPosePublisher = this->rosthread->n.advertise<ISLH_msgs::targetPoseListMessage>("monitoringISLH/targetPoseList", queueSize);
    this->startMissionPublisher = this->rosthread->n.advertise<std_msgs::UInt8>("monitoringISLH/startMission", queueSize);
}

void CommClient::coalInfoCallback(const std_msgs::UInt8::ConstPtr &msg){
    QString message = "coal$";
    message.append(QString::number((robotID)));
    message.append("$");
    message.append(QString::number((msg->data)));
    message.append("<EOF>");

    QMutexLocker ml(&mutex);
    waitingMessages.push_back(message);
}

void CommClient::taskHandlerInfoCallback(const std_msgs::UInt8::ConstPtr &msg){
    QString message = "rob$";
    message.append(QString::number((robotID)));
    message.append("$");
    message.append(QString::number((msg->data)));
    message.append("<EOF>");

    QMutexLocker ml(&mutex);
    waitingMessages.push_back(message);
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

    QMutexLocker ml(&mutex);
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

    QMutexLocker ml(&mutex);
    waitingMessages.push_back(message);
}

void CommClient::sendWaitingMessages(){
    QMutexLocker ml(&mutex);

    if(waitingMessages.count() == 0) return;
    QStringList _waitingMessages;

    foreach(QString str,waitingMessages)
        _waitingMessages.push_back(str);

    waitingMessages.clear();

    ml.unlock();

    for(int i=0;i<_waitingMessages.count();i++){
        qDebug()<<"writing";
        QByteArray byteArray = QByteArray(_waitingMessages.at(i).toLatin1());
        int writtenBytes = socket->write(byteArray);
        qDebug() << _waitingMessages[i];

        qDebug() << "written" <<writtenBytes<<" of "<<_waitingMessages.at(i).size();

        if (!socket->waitForBytesWritten(250))
            qDebug()<<"writing to the socket failed";


    }
}

void CommClient::robotConnCallback(const std_msgs::String::ConstPtr &msg){
    QString message = "conn$";
    message.append(QString::number((robotID)));
    message.append("$");
    message.append(QString::fromStdString(msg->data));
    message.append("<EOF>");

    QMutexLocker ml(&mutex);
    waitingMessages.push_back(message);
}

void CommClient::taskInfoCallback(const ISLH_msgs::taskInfo2MonitorMessage::ConstPtr &msg){
    QString message = "task$";
    bool flag = false;
    foreach(taskProp task,tasks){
        if(task.taskUUID == QString::fromStdString(msg->taskUUID)){
            flag = true;

            message.append(QString::fromStdString(msg->taskUUID));
            message.append("$");
            if(task.encounteringRobotID != msg->encounteringRobotID)
                message.append(QString::number(msg->encounteringRobotID));
            message.append("$");
            if(task.encounteringTime != msg->encounteringTime)
                message.append(QString::number(msg->encounteringTime));
            message.append("$");
            if(task.handlingDuration != msg->handlingDuration)
                message.append(QString::number(msg->handlingDuration));
            message.append("$");
            if(task.pose.x != msg->posXY.x)
                message.append(QString::number(msg->posXY.x));
            message.append("$");
            if(task.pose.y != msg->posXY.y)
                message.append(QString::number(msg->posXY.y));
            message.append("$");
            if(task.requiredResourcesString != QString::fromStdString(msg->taskResource))
                message.append(QString::fromStdString(msg->taskResource));
            message.append("$");
            if(task.responsibleUnit != msg->responsibleUnit)
                message.append(QString::number(msg->responsibleUnit));
            message.append("$");
            if(task.startHandlingTime != msg->startHandlingTime)
                message.append(QString::number(msg->startHandlingTime));
            message.append("$");
            if(task.status != msg->status)
                message.append(QString::number(msg->status));
            message.append("$");
            if(task.timedOutDuration != msg->timeOutDuration)
                message.append(QString::number(msg->timeOutDuration));
            message.append("$");
            if(task.taskSiteRadius != msg->taskSiteRadius)
                message.append(QString::number(msg->taskSiteRadius));
            message.append("<EOF>");

            task.encounteringRobotID = msg->encounteringRobotID;
            task.encounteringTime = msg->encounteringTime;
            task.handlingDuration = msg->handlingDuration;
            task.pose = msg->posXY;
            task.requiredResourcesString = QString::fromStdString(msg->taskResource);
            task.responsibleUnit = msg->responsibleUnit;
            task.startHandlingTime = msg->startHandlingTime;
            task.status = msg->status;
            task.taskUUID = QString::fromStdString(msg->taskUUID);
            task.timedOutDuration = msg->timeOutDuration;
            task.taskSiteRadius = msg->taskSiteRadius;

            break;
        }
    }
    if(!flag){
        taskProp task;
        task.encounteringRobotID = msg->encounteringRobotID;
        task.encounteringTime = msg->encounteringTime;
        task.handlingDuration = msg->handlingDuration;
        task.pose = msg->posXY;
        task.requiredResourcesString = QString::fromStdString(msg->taskResource);
        task.responsibleUnit = msg->responsibleUnit;
        task.startHandlingTime = msg->startHandlingTime;
        task.status = msg->status;
        task.taskUUID = QString::fromStdString(msg->taskUUID);
        task.timedOutDuration = msg->timeOutDuration;
        task.taskSiteRadius = msg->taskSiteRadius;
        tasks.push_back(task);

        message.append(task.taskUUID);
        message.append("$");
        message.append(QString::number(task.encounteringRobotID));
        message.append("$");
        message.append(QString::number(task.encounteringTime));
        message.append("$");
        message.append(QString::number(task.handlingDuration));
        message.append("$");
        message.append(QString::number(task.pose.x));
        message.append("$");
        message.append(QString::number(task.pose.y));
        message.append("$");
        message.append(task.requiredResourcesString);
        message.append("$");
        message.append(QString::number(task.responsibleUnit));
        message.append("$");
        message.append(QString::number(task.startHandlingTime));
        message.append("$");
        message.append(QString::number(task.status));
        message.append("$");
        message.append(QString::number(task.timedOutDuration));
        message.append("$");
        message.append(QString::number(task.taskSiteRadius));
        message.append("<EOF>");
    }

    QMutexLocker ml(&mutex);
    waitingMessages.push_back(message);
}

void CommClient::leaderInfoCallback(const std_msgs::Int8MultiArray::ConstPtr &msg){
    QString message = "lead";
    foreach(int8_t id,msg->data){
        message.append("$");
        message.append(QString::number(id));
    }
    message.append("<EOF>");

    QMutexLocker ml(&mutex);
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

        queueSize = result["queueSize"].toInt();
        qDebug()<<result["queueSize"].toString();
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
