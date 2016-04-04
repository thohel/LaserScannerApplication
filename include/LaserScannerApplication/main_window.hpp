/**
 * @file /include/LaserScannerApplication/main_window.hpp
 *
 * @brief Qt based gui for LaserScannerApplication.
 *
 * @date November 2010
 **/
#ifndef LaserScannerApplication_MAIN_WINDOW_H
#define LaserScannerApplication_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>
#include <boost/atomic.hpp>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace LaserScannerApplication {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();
    void closeEvent(QCloseEvent *event); // Overloaded function
    void showNoMasterMessage();
    void performAutoScan();

public Q_SLOTS:
    //Auto connections
    void on_button_refresh_topic_clicked(bool check);
    void on_button_subscribe_topic_clicked(bool check);
    void on_button_folder_select_clicked(bool check);
    void on_button_take_pic_clicked(bool check);
    void on_button_load_picture_clicked(bool check);
    void on_button_auto_scan_clicked(bool check);
    void on_button_set_ref_images_clicked(bool check);
    //Manual connections
    void readCurrentAngle(double d);
    void updateTopics(QStringList list);
    void toggleFilter(bool filter);
    void updateView(int i);
    void toggleLasers(bool check);
    cv::Mat getImage(int lasers);
    void updatePointCloudViewer();

Q_SIGNALS:
    void getTopics();
    void subscribeToImage(QString topic);
    void setAngle(double angle);
    void setLasers(int i);

private:
    Ui::MainWindowDesign *ui;
    QNode qnode;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> pointCloudViewer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    QVTKWidget *w;
    int left;
    int right;
    cv::Mat referenceBefore;
    cv::Mat referenceAfter;
    cv::Mat matToShow;
    cv::Mat processImageSet(cv::Mat before, cv::Mat after, cv::Mat referenceBefore, cv::Mat referenceAfter);
    void performTriangulation(double amountRotated, cv::Mat img, cv::Mat color_img, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, bool leftSide);
    void save_picture(cv::Mat picture, int number, bool after, bool reference, bool final, bool leftSide);
    bool drawPointCloud;
    bool pointCloudWidgetAdded;
    boost::atomic_bool wait_for_processing_pic;
    boost::atomic_bool new_pic_to_present;
    boost::atomic_bool new_mat_to_convert;
    boost::atomic_bool wait_for_point_cloud_viewer;
    void spawnPointCloudWidget();
    void updateFilteredImage(bool left, bool right);
    void updateImageToShow(cv::Mat image);
    bool isImagePipelineReady();
    //boost::atomic<QImage> toShow;
    QImage toShow;
};

}  // namespace LaserScannerApplication

#endif // LaserScannerApplication_MAIN_WINDOW_H
