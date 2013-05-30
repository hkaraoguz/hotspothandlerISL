#include <tf/tf.h>


#include "rosThread.h"
#include <navigationISL/networkInfo.h>
#include <QDebug>
#include <qjson/parser.h>
#include <QDir>
#include <QFile>


RosThread::RosThread()
{
    shutdown = false;
    this->currentState = HS_IDLE;

}



void RosThread::work(){



    QString path;
    path.append("../initialPoses.txt");

    // Read initial poses otherwise quit!!
    if(!readInitialPoses(path))
    {
        qDebug()<<"Initial poses file could not be read!! ";
        qDebug()<<"Quitting...";

        ros::shutdown();

        emit this->rosFinished();

        return;

    }


    srand(time(0));

    for(int i = 1 ; i<=numOfRobots; i++)
    {
        dataReceived[i] = true;
        for(int j = 1 ; j<=numOfRobots; j++)
        {
            adjM[i][j] = 0;
        }
       // bin[i][1] = rand()%300;
       // bin[i][2] = rand()%300;

      //  qDebug()<<bin[i][1]<<" "<<bin[i][2];

    }


    if(!ros::ok()){

        emit this->rosFinished();

        return;
    }

    emit rosStarted();

    ros::Rate loop(10);

    neighborInfoSubscriber = n.subscribe("communicationISL/neighborInfo",5,&RosThread::handleNeighborInfo,this);

    positionSubscriber = n.subscribe("amcl_pose",2,&RosThread::handlePositionInfo,this);

    hotspotSubscriber = n.subscribe("hotspotobserverISL/hotspot",5,&RosThread::handleHotspotMessage,this);

    messageIn = n.subscribe("communicationISL/hotspothandlerMessage",5,&RosThread::handleIncomingMessage,this);
    messageOut = n.advertise<navigationISL::helpMessage>("hotspothandlerISL/outMessage",5);

    while(ros::ok())
    {
        ros::spinOnce();

        this->manageHotspot();


        loop.sleep();



    }

    qDebug()<<"I am quitting";

    ros::shutdown();

    emit rosFinished();


}
void RosThread::shutdownROS()
{
    ros::shutdown();
    // shutdown = true;


}
void RosThread::clearCheckedList()
{
    for(int i = 0; i <=numOfRobots; i++)
    {
        checkedNeighborList[i] = 0;
    }
}
void RosThread::manageHotspot()
{
    int hotspotId = -1;
    if (this->currentState == HS_IDLE)
    {
        hotspotId = this->getHotspot(timeoutHotspot);
    }

    if(hotspotId >= 0 || this->currentState != HS_IDLE)
    {
        if(helpRequesterID > 0)
        {


            navigationISL::helpMessage helpMessage;

            helpMessage.robotid = helpRequesterID;
            helpMessage.messageid = HMT_NOT_HELPING;

            helpRequesterID = -1;

            this->messageOut.publish(helpMessage);

            return;


        }
    }
    else
    {
        if(helpRequesterID > 0 && this->currentState == HS_IDLE)
        {


            navigationISL::helpMessage helpMessage;

            helpMessage.robotid = helpRequesterID;
            helpMessage.messageid = HMT_HELPING;

            this->currentState = HS_HELPING;

            //helpRequesterID = -1;

            helpStartTime = QDateTime::currentDateTime().toTime_t();

            this->messageOut.publish(helpMessage);

            return;


        }

    }

    if(this->currentState != HS_WAITING_FOR_RESPONSE)
    {

        // Check if we have any hotspot waiting
        if(hotspotId >= 0)
        {

            // If I am available
            if(this->currentState == HS_IDLE)
            {

                int tempId = this->findHelper();

                if(tempId > 0)
                {
                    helperID = tempId;
                    navigationISL::helpMessage helpMessage;

                    helpMessage.robotid = helperID;
                    helpMessage.messageid = HMT_HELP_REQUEST;


                    this->messageOut.publish(helpMessage);

                    this->currentState = HS_WAITING_FOR_RESPONSE;

                    return;
                }
                else
                {
                    waitingStartTime = QDateTime::currentDateTime().toTime_t();

                    this->currentState == HS_WAITING_FOR_HELP;

                }


            }

        }

    }
    // We are waiting for a response
    else if(this->currentState == HS_WAITING_FOR_RESPONSE)
    {
        uint currentTime = QDateTime::currentDateTime().toTime_t();

        if(currentTime - hotspotList.at(0) > timeoutHotspot)
        {
            hotspotList.remove(0);
            this->currentState = HS_IDLE;
            /// HOTSPOT KAYIT OLACAK
        }
        else
        {   // If we have a response and a robot is helping
            if(helperID > 0)
            {
                this->currentState = HS_HANDLING_HOTSPOT;

                helpStartTime = QDateTime::currentDateTime().toTime_t();

                clearCheckedList();
            }

        }

    }
    else if(this->currentState == HS_HANDLING_HOTSPOT || this->currentState == HS_HELPING)
    {
        uint currentTime = QDateTime::currentDateTime().toTime_t();
        if(currentTime - helpStartTime > handlingDuration)
        {
            /// HOTSPOT KAYDEDILECEK
            hotspotList.remove(0);

            this->currentState = HS_IDLE;
        }


    }
    else if(this->currentState == HS_WAITING_FOR_HELP)
    {
        uint currentTime = QDateTime::currentDateTime().toTime_t();

        if(currentTime - waitingStartTime > waitingDuration)
        {
            int tempId = this->findHelper();

            if(tempId > 0)
            {
                helperID = tempId;
                navigationISL::helpMessage helpMessage;

                helpMessage.robotid = helperID;
                helpMessage.messageid = HMT_HELP_REQUEST;

                this->messageOut.publish(helpMessage);

                this->currentState = HS_WAITING_FOR_RESPONSE;

                return;
            }

            waitingStartTime = currentTime;

        }
    }
}
int RosThread::getHotspot(uint timeout)
{
    uint currentTime = QDateTime::currentDateTime().toTime_t();

    int firstSize = hotspotList.size();
    int i = 0;
    while(i < firstSize)
    {
        if(currentTime-hotspotList.at(i) > 0 && currentTime-hotspotList.at(i) > timeout)
        {
            firstSize--;
            hotspotList.remove(i);
            /// HOTSPOT SAVER YAZILACAK BURAYA SILINEN HOTSPOT KAYDEDILECEK TARIHI ILE
        }
        else
        {
            return i;
        }

    }
    return -1;
}
int RosThread::findHelper()
{
    int minID = -1;
    for(int i = 1; i <= numOfRobots;i++)
    {
        double minDist = 1000000;


        if(bin[i][3] > 0 && i != robot.robotID && checkedNeighborList[i] > 0)
        {
            double xdiff = bin[robot.robotID][1] - bin[i][1];

            double ydiff = bin[robot.robotID][2] - bin[i][2];

            double norm = sqrt(xdiff*xdiff + ydiff*ydiff);

            if(norm < minDist)
            {
                minID = i;
                minDist = norm;

            }

        }
    }


    return minID;
}
void RosThread::handleNeighborInfo(navigationISL::neighborInfo info)
{
    QString str = QString::fromStdString(info.name);
    navigationISL::neighborInfo inf = info;

  /*  QString str = QString::fromStdString(inf.name);

    str.remove("IRobot");

    int id = str.toInt();

    bin[id][1] = inf.posX;

    bin[id][2] = inf.posY;

    bin[id][3] = inf.radius;

    dataReceived[id] = true;*/

   if(str.contains("Neighbors"))
    {
        QStringList list = str.split(";");

        QVector<int> ids;

        for(int i = 0; i < list.size(); i++)
        {

            if(list.at(i).contains("IRobot"))
            {
                QString ss = list.at(i);

                ss.remove("IRobot");

                ids.push_back(ss.toInt());
            }
            else if(list.at(i) == "0")
            {
                for(int j= 1; j <= numOfRobots; j++)
                {
                    if(j != this->robot.robotID)
                    {
                        bin[j][1] = 0;
                        bin[j][2] = 0;
                        bin[j][3] = 0;
                    }

                }
                break;
            }
        }
        if(ids.size() > 0)
        {
            for(int i = 1; i <= numOfRobots; i++)
            {

                for(int j = 0; j < ids.size(); j++)
                {
                    if(i != this->robot.robotID && i != ids.at(j))
                    {
                        bin[i][1] = 0;
                        bin[i][2] = 0;
                        bin[i][3] = 0;
                    }
                    else if(i == ids.at(j))
                    {
                        bin[i][3] = 10;
                        /// BIN DOLDURALACAK
                    }

                }

            }
        }

        for(int k = 1; k <= numOfRobots; k++)
        {
            qDebug()<<"Bin value "<<k<<" "<<bin[k][3];
        }

        return;

    }

    str.remove("IRobot");

    int num = str.toInt();

    if(num > 0 && num < numOfRobots){

        bin[num][1] = inf.posX;
        bin[num][2] = inf.posY;
        bin[num][3] = inf.radius;

        //bt[num][1] = neighborInfo.targetX;
        //bt[num][2] = neighborInfo.targetY;
        qDebug()<<"robot number "<<num;
    }
    else qDebug()<<"Unknown robot id number";

}
void RosThread::handleHotspotMessage(navigationISL::hotspot msg)
{

    hotspotList.push_back(msg.hotspot);

}
void RosThread::handlePositionInfo(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    bin[robot.robotID][1] = msg->pose.pose.position.x*100;
    bin[robot.robotID][2] = msg->pose.pose.position.y*100;
    bin[robot.robotID][3] = robot.radius;


}
void RosThread::handleIncomingMessage(navigationISL::helpMessage msg)
{
    if(msg.messageid == HMT_HELP_REQUEST)
    {
        helpRequesterID = msg.robotid;
    }
    else if(msg.messageid == HMT_HELPING)
    {
        helperID = msg.robotid;
    }
    else if(msg.messageid == HMT_NOT_HELPING)
    {
        helperID = -1;
        checkedNeighborList[msg.robotid] = -1;
    }


}
bool RosThread::readInitialPoses(QString filepath)
{
    QFile file(filepath);

    if(!file.open(QFile::ReadOnly))
    {

        return false;
    }

    QTextStream stream(&file);
    int count  = 1;
    while(!stream.atEnd())
    {
        QString str = stream.readLine();

        QStringList poses = str.split(",");

        bin[count][1] = poses.at(0).toDouble();
        bin[count][2] = poses.at(1).toDouble();

        qDebug()<<"Pose "<<count<<" "<<bin[count][1]<<" "<<bin[count][2];
        count++;

    }

    file.close();

    return true;
}
