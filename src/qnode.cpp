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
    processing_pic = false;
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
    ros::Rate loop_rate(200);
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
    dump_pic_counter++;

    dump_pic_counter = dump_pic_counter % get_every_n_pics;

    std::cout << "The counter is at " << dump_pic_counter << std::endl;

    // We want to early return exept every n pics;
    if (dump_pic_counter != 0)
        return;

    Q_EMIT updateView(0);

    // XXX: This is not an ideal solution. We are signaling that a new image has arrived.
    // We are then dropping the image on the floor and the application has to use call into
    // this class to decide if it wants to acquire the image. If it does then it gets the next image.

    // If we are processing the current picture then we do not want to store a new one
    if (processing_pic)
        return;

    std::cout << "Setting the image pointer" << std::endl;
    imagePtr = image_msg;

    pictureSet = true;
}

void QNode::setAngle(double angle)
{
    std_msgs::Int32 msg;
    //convert from 0-360 to 0-14000 here
    msg.data = angle*38.8888888;
    wait_for_angle.store(true);
    anglepub.publish(msg);
    while (current_angle < angle - 0.1 || current_angle > angle + 0.1)
        std::cout << "Current angle is " << current_angle << "and wanted is " << angle << std::endl; // XXX: This is not good, there should be some infinite-loop-avoidance here
    wait_for_angle.store(false);
}

void QNode::setLasers(int i)
{
    std_msgs::Int32 msg;
    msg.data = i;
    wait_for_laser.store(true);
    laserpub.publish(msg);
    while (current_laser_val != i);
    wait_for_laser.store(false);
}

void QNode::angleCallback(const std_msgs::Int32::ConstPtr &msg)
{
    current_angle = msg->data*0.0257142857;
    Q_EMIT sendCurrentAngle(current_angle);
}

void QNode::laserCallback(const std_msgs::Int32::ConstPtr &msg)
{
    current_laser_val = msg->data;
}

cv::Mat QNode::getCurrentImage()
{
    // Lock the mutex so that we can avoid queing up images
    //picLock.lock();
    cv_bridge::CvImagePtr cvImage;

  //  wait_for_pic = true;

    if (pictureHasBeenSet()) {

        processing_pic = true;

//        while (wait_for_pic);

        // Convert ROS image message to opencv
        try {
            cvImage = cv_bridge::toCvCopy(imagePtr, "8UC3");
        } catch (cv_bridge::Exception& e) {
            std::cout << "Failed conversion of the image" << std::endl;
            ROS_ERROR("cv_bridge exception: %s", e.what());
            //return cvImage.get()->image;
        }

        processing_pic = false;
    } else {
        std::cout << "Trying to convert an image that doesnt exist" << std::endl;
    }

    return cvImage.get()->image;
}


// GETTER Methods
bool QNode::pictureHasBeenSet()
{
    return pictureSet;
}

bool QNode::waitingForAngle()
{
    return wait_for_angle;
}

bool QNode::waitingForLaser()
{
    return wait_for_laser;
}

}  // namespace LaserScannerApplication
