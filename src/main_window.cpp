/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <fstream>
#include "../include/LaserScannerApplication/main_window.hpp"
#include <boost/thread.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace LaserScannerApplication {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
    , ui(new Ui::MainWindowDesign)
    , cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
    , drawPointCloud(false)
    , pointCloudWidgetAdded(false)
{
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZ>::Ptr >("pcl::PointCloud<pcl::PointXYZ>::Ptr");
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >("pcl::PointCloud<pcl::PointXYZRGB>::Ptr");

    // Setup ui and ros
    ui->setupUi(this);

    ui->greenSliderMax->setValue(130);
    ui->blueSliderMax->setValue(130);
    ui->redSliderMax->setValue(255);
    ui->redSliderMin->setValue(30);
    ui->medianSlider->setValue(5);
    ui->erosionSlider->setValue(1);

    ui->diffReferenceCheckBox->setEnabled(false);
    ui->xposSlider->setEnabled(false);
    ui->yposSlider->setEnabled(false);
    ui->widthSlider->setEnabled(false);
    ui->heightSlider->setEnabled(false);

    ui->textbox_path->setText("/home/minions");
    ui->button_take_pic->setEnabled(false);

    setWindowIcon(QIcon(":/images/icon2.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    // Signal for angles
    QObject::connect(&qnode, SIGNAL(sendCurrentAngle(double)), this, SLOT(readCurrentAngle(double)));
    QObject::connect(this, SIGNAL(setAngle(double)), &qnode, SLOT(setAngle(double)));

    // Singal for lasers
    QObject::connect(this, SIGNAL(setLasers(int)), &qnode, SLOT(setLasers(int)));

    // Signals for topic list in drop-down box
    QObject::connect(this, SIGNAL(getTopics()), &qnode, SLOT(findTopics()));
    QObject::connect(&qnode, SIGNAL(sendTopics(QStringList)), this, SLOT(updateTopics(QStringList)));

    // Signals for subscribing to image topic
    QObject::connect(this, SIGNAL(subscribeToImage(QString)), &qnode, SLOT(subscribeToImage(QString)));

    // Signal that we should update our view
    QObject::connect(&qnode, SIGNAL(updateView(int)), this, SLOT(updateView(int)));

    // Toggle lasers upon request of override
    QObject::connect(ui->toggleLeftLaserCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleLasers(bool)));
    QObject::connect(ui->toggleRightLaserCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleLasers(bool)));

    // Ensure the viewer is updated when a control is manipulated
    QObject::connect(ui->blueSliderMin, SIGNAL(valueChanged(int)), this, SLOT(updateView(int)));
    QObject::connect(ui->blueSliderMax, SIGNAL(valueChanged(int)), this, SLOT(updateView(int)));
    QObject::connect(ui->greenSliderMin, SIGNAL(valueChanged(int)), this, SLOT(updateView(int)));
    QObject::connect(ui->greenSliderMax, SIGNAL(valueChanged(int)), this, SLOT(updateView(int)));
    QObject::connect(ui->redSliderMin, SIGNAL(valueChanged(int)), this, SLOT(updateView(int)));
    QObject::connect(ui->redSliderMax, SIGNAL(valueChanged(int)), this, SLOT(updateView(int)));
    QObject::connect(ui->medianSlider, SIGNAL(valueChanged(int)), this, SLOT(updateView(int)));
    QObject::connect(ui->erosionSlider, SIGNAL(valueChanged(int)), this, SLOT(updateView(int)));
    QObject::connect(ui->cannySlider, SIGNAL(valueChanged(int)), this, SLOT(updateView(int)));
    QObject::connect(ui->checkBox, SIGNAL(toggled(bool)), this, SLOT(toggleFilter(bool)));
    QObject::connect(ui->diffCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleFilter(bool)));
    QObject::connect(ui->diffReferenceCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleFilter(bool)));
    QObject::connect(ui->medianCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleFilter(bool)));
    QObject::connect(ui->erosionCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleFilter(bool)));
    QObject::connect(ui->mainCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleFilter(bool)));
    QObject::connect(ui->sharpeningCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleFilter(bool)));
    QObject::connect(ui->cannyCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleFilter(bool)));
    QObject::connect(ui->cropingCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleFilter(bool)));

    setlocale (LC_NUMERIC, "C");

    ui->textbox_path->setText("/home/minions");
    ui->saveCheckBox->setEnabled(false);

    // Init ROS
    qnode.init();
}

MainWindow::~MainWindow() {}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

void MainWindow::updateTopics(QStringList list)
{
    ui->comboBox->clear();
    ui->comboBox->addItems(list);
}

void MainWindow::on_button_refresh_topic_clicked(bool check)
{
    Q_EMIT getTopics();
}

void MainWindow::toggleLasers(bool check)
{
    if (ui->toggleLeftLaserCheckBox->isChecked() && ui->toggleRightLaserCheckBox->isChecked()) {
        Q_EMIT setLasers(3);
    } else if (ui->toggleLeftLaserCheckBox->isChecked()) {
        Q_EMIT setLasers(1);
    } else if (ui->toggleRightLaserCheckBox->isChecked()) {
        Q_EMIT setLasers(2);
    } else {
        Q_EMIT setLasers(0);
    }
}

void MainWindow::on_button_subscribe_topic_clicked(bool check)
{
    if (ui->comboBox->currentText().length() != 0) {
        if (ui->comboBox->currentText().contains("image_color"))
            Q_EMIT subscribeToImage(ui->comboBox->currentText());
    }
}

void MainWindow::on_button_take_pic_clicked(bool check)
{
    std::cout << "Not yet implemented" << std::endl;
}

void MainWindow::on_button_load_picture_clicked(bool check)
{
    std::cout << "Not yet implemented" << std::endl;
}

QImage MainWindow::mat2qimage(cv::Mat& mat) {
    switch (mat.type()) {
        // 8-bit, 4 channel
        case CV_8UC4: {
            QImage image(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB32);
            return image;
        }

        // 8-bit, 3 channel
        case CV_8UC3: {
            QImage image(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
            return image.rgbSwapped();
        }

        // 8-bit, 1 channel
        case CV_8UC1: {
            static QVector<QRgb>  sColorTable;

            // only create our color table once
            if (sColorTable.isEmpty()) {
                for (int i = 0; i < 256; ++i)
                    sColorTable.push_back(qRgb(i, i, i));
            }

            QImage image(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);
            image.setColorTable(sColorTable);

            return image;
        }

        default:
            std::cout << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << mat.type();
            break;
    }

    return QImage();
}

void MainWindow::processImageSet(cv::Mat before, cv::Mat after, cv::Mat referenceBefore, cv::Mat referenceAfter)
{
    cv::Mat final = after;
    cv::Mat temp;
    cv::Mat backup = after;

    if (ui->crossHairCheckBox->isChecked()) {
        cv::Point2i leftMid;
        leftMid.x = 0;
        leftMid.y = final.rows/2;
        cv::Point2i rightMid;
        rightMid.x = final.cols;
        rightMid.y = final.rows/2;
        cv::line(final, leftMid, rightMid, cv::Scalar(0, 255, 0), 2); // Crosshair horizontal
        cv::Point2i topMid;
        topMid.x = final.cols/2;
        topMid.y = 0;
        cv::Point2i botMid;
        botMid.x = final.cols/2;
        botMid.y = final.rows;
        cv::line(final, topMid, botMid, cv::Scalar(0, 255, 0), 2); // Crosshair vertical
    }

    // The picture has been set but the cropping sliders are not activated
    // We now know the size of the image, so we can adjust our sliders accordingly
    if (!ui->xposSlider->isEnabled()) {
        ui->xposSlider->setMinimum(0);
        ui->xposSlider->setMaximum(final.rows);
        ui->xposSlider->setEnabled(true);
        ui->heightSlider->setMinimum(0);
        ui->heightSlider->setMaximum(final.rows);
        ui->heightSlider->setEnabled(true);
        ui->yposSlider->setMinimum(0);
        ui->yposSlider->setMaximum(final.cols);
        ui->yposSlider->setEnabled(true);
        ui->widthSlider->setMinimum(0);
        ui->widthSlider->setMaximum(final.cols);
        ui->widthSlider->setEnabled(true);
    }

    if (ui->diffCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        cv::absdiff(before, after, temp);
        final = temp;
    }

    if (ui->diffReferenceCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        cv::Mat referenceDiff;
        cv::absdiff(referenceBefore, referenceAfter, referenceDiff);
        cv::absdiff(final, referenceDiff, temp);
        final = temp;
    }

    if (ui->cropingCheckBox->isChecked()) {
        // Define a box that is our Region of Interest
        cv::Rect roi;
        roi.x = ui->xposSlider->value();
        roi.y = ui->yposSlider->value();
        roi.width = ui->widthSlider->value();
        roi.height = ui->heightSlider->value();
        temp = final(roi);
        final = temp;
    } else if (ui->cropingVisualizeCheckBox->isChecked()) { // XXX: Should not be added to image when we start 3D capture
        // Define a box that is our Region of Interest
        cv::Rect roi;
        roi.x = ui->xposSlider->value();
        roi.y = ui->yposSlider->value();
        roi.width = ui->widthSlider->value();
        roi.height = ui->heightSlider->value();
        cv::rectangle(final, roi, cv::Scalar(0, 255, 0));
    }

    if (ui->sharpeningCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        cv::Mat temp2;
        cv::GaussianBlur(final, temp2, cv::Size(0, 0), 5);
        cv::addWeighted(final, 3, temp2, -1, 0, temp);
        final = temp;
    }

    if (ui->medianCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        // Post-processing to remove noise
        int kernelSize = ui->medianSlider->value();

        if (kernelSize % 2 != 1)
            kernelSize = kernelSize - 1;

        if (kernelSize < 1)
            kernelSize = 1;

        cv::medianBlur(final, temp, kernelSize);
        final = temp;
    }

    if (ui->erosionCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        int erosion_size = ui->erosionSlider->value();
        cv::Mat element = getStructuringElement(cv::MORPH_RECT,
                                                cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                                cv::Point( erosion_size, erosion_size ) );
        // Apply the erosion operation
        erode(final, temp, element );
        final = temp;
    }

    if (ui->cannyCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        // Detect edges using canny
        cv::Canny(final, temp, ui->cannySlider->value(), 255, 3 , true);

        cv::vector<cv::vector<cv::Point> > contours;
        cv::vector<cv::Vec4i> hierarchy;

        // Find contours
        findContours(temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

        // Draw contours
        final = cv::Mat::zeros(temp.size(), CV_8UC1 );
        for(uint i = 0; i < contours.size(); i++) {
            cv::Scalar color = cv::Scalar(255);
            drawContours(final, contours, i, color, 2, -2, hierarchy, 2, cv::Point() );
        }
    }

    if (ui->cropingCheckBox->isChecked()) {
        // Define a box that is our Region of Interest
        cv::Rect roi;
        roi.x = ui->xposSlider->value();
        roi.y = ui->yposSlider->value();
        roi.width = ui->widthSlider->value();
        roi.height = ui->heightSlider->value();
        final.copyTo(backup(roi));
        final = backup;
    }

    if (ui->checkBox->isChecked() && ui->mainCheckBox->isChecked()) {
        cv::inRange(final, cv::Scalar(ui->blueSliderMin->value(), ui->greenSliderMin->value(), ui->redSliderMin->value()),
                         cv::Scalar(ui->blueSliderMax->value(), ui->greenSliderMax->value(), ui->redSliderMax->value()), temp);
        final = temp;
    }

    this->filtered = final;
    this->color_img = before;
}

void MainWindow::performTriangulation(double amountRotated, cv::Mat img, cv::Mat color_img, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, bool leftSide)
{
    int cam_res_vert = img.rows;
    int cam_res_hor = img.cols;

    double radiansToDegrees = 180.0 / 3.14159265359;
    double degreesToRadians = 3.14159265359 / 180.0;

    double laser_angle = ui->degreeLaserAngle->value();

    // These values are not perfect, but the variance between cameras is good enough for now
    // Assumes hd resolution is used
    double cam_focal_distance_pixels = 1081.37;
//    double cam_focal_distance_mm = 3.291;
//    double cam_cx = 959.5;
//    double cam_cy = 539.5;

//    double cam_fov_hor = 84.1; // Degrees
//    double cam_fov_vert = 53.8;
//    double cam_tilt_angle = 0.0; // Degrees
    double cam_center_dist = 300.0; // mm
    double laser_cam_dist = cam_center_dist*tan(laser_angle*degreesToRadians);
    double laser_center_dist = cam_center_dist / cos(laser_angle*degreesToRadians);

    // If a area of intereset has been set then we should take that into account
    int startY = ui->yposSlider->value();
    int endY = ui->heightSlider->value() == 0 ? cam_res_vert : ui->yposSlider->value() + ui->heightSlider->value();
    int startX = ui->xposSlider->value();
    int endX = ui->widthSlider->value() == 0 ? cam_res_hor : ui->xposSlider->value() + ui->widthSlider->value();
    int radCounter = 0;
    double radSum = 0.0;

    for (int y = startY; y < endY; y++) {

        bool found_first_x = false;
        int first_x = -1;
        int last_x = -1;

        for (int x = startX; x < endX; x++) {
            cv::Scalar intensity = img.at<uchar>(y, x);
            if (intensity.val[0] > 200) {
                if (!found_first_x) {
                    found_first_x = true;
                    first_x = x;
                } else {
                    last_x = x;
                }
            } else if (found_first_x) {
                // End of continuous segment of the line. Just break out of the loop and calculate the point.
                break;
            }
        }

        if (found_first_x) {
            double average_x = double (first_x + last_x)/2;

            double pixel_angle_x;
            if (leftSide) {
                pixel_angle_x = atan2(average_x - (cam_res_hor/2), cam_focal_distance_pixels)*radiansToDegrees;
            } else {
                pixel_angle_x = atan2(average_x - (cam_res_hor/2), cam_focal_distance_pixels)*radiansToDegrees;
            }

            double pixel_angle_hor_x = (90 - pixel_angle_x);
            double laser_hor_angle = (180 - 90 - laser_angle);

            if (laser_hor_angle < 0.0)
                std::cout << "Hor_angle negative" << std::endl;

            double laser_cam_angle = 180 - pixel_angle_hor_x - laser_hor_angle;

            if (laser_cam_angle < 0.0)
                std::cout << "Laser_cam_angle negative" << std::endl;

            double laser_cam_plane_dist = laser_cam_dist*sin(pixel_angle_hor_x*degreesToRadians)/sin(laser_cam_angle*degreesToRadians);

            if (laser_cam_dist < 0.0)
                std::cout << "Laser cam dist negative" << std::endl;

            if (laser_cam_plane_dist < 0.0)
                std::cout << "Laser cam plane dist negative" << std::endl;

            double radius = laser_center_dist - laser_cam_plane_dist;

            // Don't bother adding the point if it is outside our defined radius
            if (radius > ui->radiusSlider->value()) {
                std::cout << "Radius to big" << std::endl;
                continue;
            }

            // Radius is negative, that's not good
            if (radius < 0.0) {
                std::cout << "Radius negative" << std::endl;
                continue;
            }

            // Filter out nan's
            if (radius != radius)
                continue;

            radCounter++;
            radSum += radius;

            double x_pos = sin((amountRotated - laser_angle)*degreesToRadians)*radius;
            double y_pos = cos((amountRotated - laser_angle)*degreesToRadians)*radius;

            double cam_plane_center_dist = cos(laser_angle*degreesToRadians)*radius;
            double cam_cam_plane_dist = cam_center_dist - cam_plane_center_dist;

            double z_pos = y / cam_focal_distance_pixels * cam_cam_plane_dist;

            cv::Vec3b bgrPixel = color_img.at<cv::Vec3b>(y, average_x);

            pcl::PointXYZRGB *point = new pcl::PointXYZRGB;
            point->r = bgrPixel[0];
            point->g = bgrPixel[1];
            point->b = bgrPixel[2];
            point->x = x_pos;
            point->y = y_pos;
            point->z = z_pos;

            cloud->push_back(*point);
        }
        double averageRad = radSum/radCounter;
        std::cout << "Radius is: " << averageRad << std::endl;
        radSum = 0.0;
        radCounter = 0;
    }
}

void MainWindow::readCurrentAngle(double d){
    ui->current_position->display(d);
}

void MainWindow::on_button_folder_select_clicked(bool check)
{
    QString fileName;
    fileName = QFileDialog::getExistingDirectory(this, tr("Choose Or Create Directory"),"/home/minions",QFileDialog::DontResolveSymlinks);
    ui->textbox_path->setText(fileName);
    ui->saveCheckBox->setEnabled(true);
}

void MainWindow::save_picture(cv::Mat picture, int number, bool after, bool reference, bool final, bool leftSide)
{
    if (!ui->saveCheckBox->isChecked())
        return;

    QString url = ui->textbox_path->toPlainText();
    url.append("/");
    url.append(ui->fileName->text());
    url.append("_");
    url.append(QString::number(number));

    if (reference)
        url.append("_reference");
    if (final) {
        url.append("_final");
    } else if (after) {
        url.append("_after");
    } else {
        url.append("_before");
    }

    if (leftSide && !reference) {
        url.append("_left");
    } else if (!reference){
        url.append("_right");
    }

    url.append(".png");
    cv::imwrite(url.toUtf8().constData(), picture);
}

void MainWindow::on_button_set_ref_images_clicked(bool check)
{
    // Get an image with the lasers on
    referenceAfter = getImage(3);
    save_picture(referenceAfter, 0, true, true, false, false);

    // Get an image with the lasers off
    referenceBefore = getImage(0);
    save_picture(referenceBefore, 0, false, true, false, false);

    ui->diffReferenceCheckBox->setEnabled(true);
}

void MainWindow::on_button_auto_scan_clicked(bool check)
{
    // We need to spawn a worker thread as to not block the GUI
    boost::thread* thr = new boost::thread(boost::bind(&MainWindow::performAutoScan, this));
    ui->button_auto_scan->setEnabled(false);
}

void MainWindow::performAutoScan()
{
    // XXX: Deactivate filter controls ++

    // Change from image to point cloud visualization
    drawPointCloud = true;

    double deg_per_pic = ui->degree_auto_scan->value();
    int iterations = (int) ((double) 360 / deg_per_pic);
    double shadow_position = 0.0;

    int i = 0;

    double old_val = ui->current_position->value();

    while (ui->current_position->value() < old_val + 360.0) {
        double new_pos = old_val + i*deg_per_pic;

        // Get an image with the lasers off
        cv::Mat before = getImage(0);
        save_picture(before, i, false, false, false, true);

        if (ui->toggleUseLeftLaserCheckBox) {
            // Get an image with the left laser on
            cv::Mat after = getImage(1);
            save_picture(after, i, true, false, false, true);

            // Process image and perform triangulation
            processImageSet(before, after, referenceBefore, referenceAfter);
            performTriangulation(deg_per_pic*((double) i), this->filtered, this->color_img, this->cloud, true);
            save_picture(this->filtered, i, false, false, true, true);
        }

        if (ui->toggleUseRightLaserCheckBox) {
            // Get an image with the right laser on
            cv::Mat after = getImage(2);
            save_picture(after, i, true, false, false, false);

            // Process image and perform triangulation
            processImageSet(before, after, referenceBefore, referenceAfter);
            performTriangulation(deg_per_pic*((double) i), this->filtered, this->color_img, this->cloud, false);
            save_picture(this->filtered, i, false, false, true, false);
        }

        // Increment the angle
        Q_EMIT setAngle(new_pos);

        // Wait until table has turned into position
        while (!(ui->current_position->value() < new_pos + 0.1 &&
                 ui->current_position->value() > new_pos - 0.1)  ) {
        }
        i++;
    }
    ui->button_auto_scan->setEnabled(false);

    // Save the point cloud to a pcd file
    QString url = ui->textbox_path->toPlainText();
    url.append("/");
    url.append(ui->fileName->text());
    url.append(".pcd");
    pcl::io::savePCDFileBinary(url.toUtf8().constData(), *cloud);
}

void MainWindow::toggleFilter(bool filter)
{
    updateView(0);
}

cv::Mat MainWindow::getImage(int lasers)
{
    if (!(ui->toggleLeftLaserCheckBox->isChecked() || ui->toggleRightLaserCheckBox)) {
        // Take picture with laser
        qnode.wait_for_laser = true;
        Q_EMIT setLasers(lasers);
        while (qnode.wait_for_laser.load());
    }

    // Wait until picture is updated
    qnode.wait_for_pic = true;
    while (qnode.wait_for_pic.load());
    return qnode.getCurrentImage();
}

void MainWindow::updateFilteredImage(bool left, bool right)
{
    wait_for_pic = true;
    cv::Mat after;

    if (left && right) {
        // Get an image with both lasers on
        after = getImage(3);
    } else if (left) {
        // Get an image with left laser on
        after = getImage(1);
    } else if (right) {
        // Get an image with right laser on
        after = getImage(2);
    } else {
        // Get an image with the lasers off
        after = getImage(0);
    }

    // Get an image with the lasers off
    cv::Mat before = getImage(0);

    processImageSet(before, after, referenceBefore, referenceAfter);
    wait_for_pic = false;
}

void MainWindow::updateView(int i)
{
    // If we have not yet reached the pointcloud stage we should show the image
    if (!drawPointCloud && qnode.pictureHasBeenSet()) {
        if (!wait_for_pic.load()) {
            QImage qImg = mat2qimage(this->filtered);
            QPixmap pixMap = QPixmap::fromImage(qImg);

            // Add the pixmap to the label that presents it
            ui->imageLabel->setPixmap(pixMap);
            boost::thread* thr = new boost::thread(boost::bind(&MainWindow::updateFilteredImage, this, true, true));
        }
    }

    // If we have reached the pointcloud stage then we should not show the image
    if (drawPointCloud) {
        // If we have not set the point cloud viewer visible then we need to do that
        if (!pointCloudWidgetAdded) {
            spawnPointCloudWidget();
            //boost::thread* thr = new boost::thread(boost::bind(&MainWindow::spawnPointCloudWidget, this));
        }

        // If we have not added the pointcloud to the viewer then we should do that
        if (cloud->empty())
            return;
        if (!viewer->updatePointCloud(this->cloud, "displayCloud")) {
            viewer->addPointCloud(this->cloud, "displayCloud");
            viewer->spinOnce();
        } else {
            viewer->spinOnce();
        }
        QImage qImg = mat2qimage(this->filtered);
        QPixmap pixMap = QPixmap::fromImage(qImg);

        // Add the pixmap to the label that presents it
        ui->imageLabel->setPixmap(pixMap);
    }
}

void MainWindow::spawnPointCloudWidget()
{
    // Set up the point cloud viewer
    w = new QVTKWidget();
    viewer.reset(new pcl::visualization::PCLVisualizer ("viewer", true));
    pointCloudWidgetAdded = true;
}

}  // namespace LaserScannerApplication

