/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include "../include/LaserScannerApplication/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace LaserScannerApplication {

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
    {
        qRegisterMetaType<pcl::PointCloud<pcl::PointXYZ>::Ptr >("pcl::PointCloud<pcl::PointXYZ>::Ptr");
    }

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {

    pictureSet = false;
    ros::init(init_argc,init_argv,"qt_package");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    anglesub = n.subscribe<std_msgs::Int32,QNode>("current_angle", 1000, &QNode::angleCallback, this);
    anglepub = n.advertise<std_msgs::Int32>("set_turntable_angle", 1000);
    laserpub = n.advertise<std_msgs::Int32>("set_turntable_lasers", 1000);
    lasersub = n.subscribe<std_msgs::Int32>("active_lasers", 1000, &QNode::laserCallback, this);
    start();
    return true;
}

void QNode::run() {
    ros::Rate loop_rate(20);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::findTopics()
{
    QStringList list;
    QString tmp;
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        //std::cout << "Topic : " << it - master_topics.begin() << ": " << info.name << " -> " << info.datatype <<       std::endl;
        tmp = QString::fromUtf8(info.datatype.c_str());

        // XXX: Should filter out only the HD image
        // Add more types if needed
        if(QString::compare(tmp, "sensor_msgs/Image", Qt::CaseInsensitive) == 0   ) {
            list.append(QString::fromUtf8(info.name.c_str()));
        }
    }
    Q_EMIT sendTopics(list);
}

void QNode::subscribeToImage(QString topic)
{
    ros::NodeHandle n;
    const char *tmp = topic.toUtf8().constData();
    imageSub = n.subscribe<sensor_msgs::Image, QNode>(tmp, 1, &QNode::imageCallback, this);
}

void QNode::imageCallback(const sensor_msgs::ImageConstPtr &image_msg)
{
    pictureSet = true;
    //if (picLock.try_lock()) {
        imagePtr = image_msg;
        //picLock.unlock();
        Q_EMIT updateView(0);
    //}
    wait_for_pic = false;
}

bool QNode::pictureHasBeenSet()
{
    return pictureSet;
}

void QNode::setAngle(double angle)
{
    std_msgs::Int32 msg;
    //convert from 0-360 to 0-13760 here
    msg.data = angle*38.222222222;
    anglepub.publish(msg);
}

void QNode::setLasers(int i)
{
    std_msgs::Int32 msg;
    msg.data = i;
    laserpub.publish(msg);
}

void QNode::angleCallback(const std_msgs::Int32::ConstPtr &msg)
{
    double angle = msg->data*0.0261627907;
    Q_EMIT sendCurrentAngle(angle);
}

void QNode::laserCallback(const std_msgs::Int32::ConstPtr &msg)
{
    wait_for_laser = false;
}

cv::Mat QNode::getCurrentImage()
{
    // Lock the mutex so that we can avoid queing up images
    //picLock.lock();
    cv_bridge::CvImagePtr cvImage;

    if (pictureHasBeenSet()) {
        // Convert ROS image message to opencv
        try {
            cvImage = cv_bridge::toCvCopy(imagePtr, "8UC3");
        } catch (cv_bridge::Exception& e) {
            std::cout << "Failed conversion of the image" << std::endl;
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return cvImage.get()->image;
        }
    } else {
        std::cout << "Trying to convert an image that doesnt exist" << std::endl;
    }

    // Unlock the mutex so that we can get updated images
    //picLock.unlock();

    return cvImage.get()->image;
}



/* XXX: Not needed here, but instead in the method that uses this method

QString afterUrl;
QString beforeUrl;
QString afterReferenceUrl;
QString beforeReferenceUrl;

afterUrl = url.left(-1);

afterReferenceUrl = url.left(-1);

beforeUrl = url.left(-1);

beforeReferenceUrl = url.left(-1);
*/

}  // namespace LaserScannerApplication
