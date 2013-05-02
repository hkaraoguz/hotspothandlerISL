
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <navigationISL/neighborInfo.h>
#include <navigationISL/robotInfo.h>
#include <navigationISL/hotspot.h>
#include <navigationISL/helpMessage.h>
#include <QTimer>
#include <QVector>
#include <QThread>
#include <QObject>
#include <QTime>

#define numOfRobots 5
class Robot
{
public:

    int robotID;
    bool isCoordinator;
    double radius;
    double targetX;
    double targetY;
    double initialX;
    double initialY;

};


enum HandlingState
{
    HS_IDLE = 0,
    HS_WAITING_FOR_HELP = 1,
    HS_HANDLING_HOTSPOT = 2,
    HS_HELPING = 3,

    HS_WAITING_FOR_RESPONSE = 4
};

enum HotspotMessageType
{
    HMT_NONE = -1,
    HMT_HELP_REQUEST = 0,
    HMT_HELPING = 1,
    HMT_NOT_HELPING = 2
};

class RosThread:public QObject
{
    Q_OBJECT

public:

    RosThread();

private:
     bool shutdown;

     Robot robot;

     ros::NodeHandle n;

     // Get neighbor info
     ros::Subscriber neighborInfoSubscriber;

     // Get current position
     ros::Subscriber positionSubscriber;

     // Get hotspot messages
     ros::Subscriber hotspotSubscriber;

     // Get incoming messages for help
     ros::Subscriber messageIn;

    // Send outgoing messages for help
     ros::Publisher messageOut;

     QVector<long> hotspotList;

     HandlingState currentState;

     void handleNeighborInfo(navigationISL::neighborInfo info);

     void handlePositionInfo(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

     void handleHotspotMessage(navigationISL::hotspot msg);

     void handleIncomingMessage(navigationISL::helpMessage msg);

     void manageHotspot();

     int getHotspot(uint timeout);

     int findHelper();

     int checkedNeighborList[numOfRobots+1];

     void clearCheckedList();

     bool readInitialPoses(QString filepath);

     double bin[numOfRobots+1][4];// positions including itself

     int adjM[numOfRobots+1][numOfRobots+1];

     bool dataReceived[numOfRobots+1];

     int timeoutHotspot;

     int handlingDuration;

     int helperID;

     int helpRequesterID;

     uint helpStartTime;

     uint waitingStartTime;

     uint waitingDuration;

public slots:
     void work();

     void shutdownROS();


signals:
   void rosFinished();
   void  rosStarted();
   void  rosStartFailed();

};
