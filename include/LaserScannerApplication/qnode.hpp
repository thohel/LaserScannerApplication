/**
 * @file /include/LaserScannerApplication/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/

#ifndef LaserScannerApplication_QNODE_HPP_
#define LaserScannerApplication_QNODE_HPP_

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv/cv.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/atomic.hpp>
#include <qstringlist.h>

namespace LaserScannerApplication {

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void run();
    void angleCallback(const std_msgs::Int32::ConstPtr &msg);
    void imageCallback(const sensor_msgs::ImageConstPtr &image_msg);
    void laserCallback(const std_msgs::Int32::ConstPtr &msg);
    cv::Mat getCurrentImage();
    bool pictureHasBeenSet();
    bool waitingForLaser();
    bool waitingForAngle();
    bool waitingForPic();
    bool waitingForPicProcessing();

Q_SIGNALS:
    void rosShutdown();
    void sendCurrentAngle(double d);
    void sendTopics(QStringList list);
    void setPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr c);
    void setPointCloudRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr c);
    void displayImage(QString url);
    void setImageType(bool check);
    void updateView(int i);

public Q_SLOTS:
    void findTopics();
    void subscribeToImage(QString topic);
    void setAngle(double angle);
    void setLasers(int i);

private:
    int init_argc;
    char** init_argv;
    pcl::PointCloud<pcl::PointXYZRGB> cloudRGB;
    sensor_msgs::ImageConstPtr imagePtr;
    //const sensor_msgs::ImageConstPtr &imagePtr;
    ros::Publisher anglepub;
    ros::Publisher laserpub;
    ros::Subscriber anglesub;
    ros::Subscriber imageSub;
    ros::Subscriber lasersub;
    bool pictureSet;
    int current_laser_val;
    double current_angle;
    boost::atomic_bool wait_for_laser;
    boost::atomic_bool wait_for_angle;
    boost::atomic_bool wait_for_pic;
    boost::atomic_bool processing_pic;
    static const int get_every_n_pics = 6;
    int dump_pic_counter;
    bool flip_colours;

};

}  // namespace LaserScannerApplication

#endif /* LaserScannerApplication_QNODE_HPP_ */
