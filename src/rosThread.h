//#include "navigationController.h"
#include <QThread>
#include <QObject>
#include <ros/ros.h>
//===================================================================================================================================#include "navigationISL/robotInfo.h"
//#include <navigationISL/networkInfo.h>
//#include <navigationISL/helpMessage.h>
//#include "commclient.h"
#include <QTimer>

//#define numOfRobots 5
class CommClient;

class RosThread:public QObject
{
    Q_OBJECT

public:

    RosThread();

    RosThread(CommClient* currentcommclient);

public:

     void shutdownROS();

     friend class CommClient;

private:

     CommClient* commclient;

     bool shutdown;

     ros::NodeHandle n;


public slots:
     void work();

signals:
   void rosFinished();
   void  rosStarted();
   void  rosStartFailed();
};
