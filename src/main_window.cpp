/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <fstream>
#include "../include/LaserScannerApplication/main_window.hpp"
#include "../include/LaserScannerApplication/QImageCvMat.hpp"
#include <boost/thread.hpp>

namespace LaserScannerApplication {

using namespace Qt;

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
    QObject::connect(&qnode, SIGNAL(updateView(int)), this, SLOT(updateView()));

    // Signal that we have chosen to use opencv camera grabber
    QObject::connect(ui->opencvCheckBox, SIGNAL(toggled(bool)), this, SLOT(opencvCheckBox_changed(bool)));

    // Toggle lasers upon request of override
    QObject::connect(ui->toggleLeftLaserCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleLasers(bool)));
    QObject::connect(ui->toggleRightLaserCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleLasers(bool)));

    setlocale(LC_NUMERIC, "C");

    ui->textbox_path->setText("/home/minions");
    ui->saveCheckBox->setEnabled(false);

    wait_for_processing_pic = false;
    new_pic_to_present = false;
    new_mat_to_convert = false;
    wait_for_scan = false;

    // We should load the camera calibration matrix.
    // XXX: This is a hack, as it is not portable between cameras. But it will do for now
    cv::FileStorage fs2("/home/minions/Workspaces/minion_ws/c930_camera_data.xml", cv::FileStorage::READ);
    fs2["Camera_Matrix"] >> cameraMatrix;
    fs2["Distortion_Coefficients"] >> distCoeffs;

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

void MainWindow::opencvCheckBox_changed(bool check)
{
    if (check) {
        // open the default camera, use something different from 0 otherwise;
        // Check VideoCapture documentation.
        if(!cap.open(0)) {
            std::cout << "Failed to open camera!" << std::endl;
            return;
        }

        timer = new QTimer(this);
        connect(timer, SIGNAL(timeout()), this, SLOT(updateView()));

        // XXX: Select one of the two options
        if (false) {
            std::cout << cap.set(CV_CAP_PROP_FRAME_WIDTH, 2304) << std::endl;
            std::cout << cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1536) << std::endl;
            std::cout << cap.set(CV_CAP_PROP_FPS, 2) << std::endl;
            timer->start(500);
        } else if (false) {
            std::cout << cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920) << std::endl;
            std::cout << cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080) << std::endl;
            std::cout << cap.set(CV_CAP_PROP_FPS, 5) << std::endl;
            timer->start(200);
        } else {
            std::cout << cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920) << std::endl;
            std::cout << cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080) << std::endl;
            std::cout << cap.set(CV_CAP_PROP_FPS, 30) << std::endl;
            timer->start(33);
        }
    } else {
        if (timer) {
            timer->stop();
            delete timer;
        }
    }
}

void MainWindow::on_button_refresh_topic_clicked(bool check)
{
    Q_EMIT getTopics();
}

void MainWindow::on_button_subscribe_topic_clicked(bool check)
{
    if (ui->comboBox->currentText().length() != 0) {
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

cv::Mat MainWindow::getThresholdImage(cv::Mat input)
{
    cv::Mat return_mat = input;

    if (input.type() != CV_8UC1) {
        // Convert the image to grayscale
        cvtColor(input, return_mat, CV_BGR2GRAY);
    }

    cv::Mat return_mat_binary;
    cv::threshold(return_mat, return_mat_binary, 120, 255, 0);
    return return_mat_binary;
}

cv::Mat MainWindow::getSecondOrderDerivativeOfImage(cv::Mat input)
{
    int kernel_size = 3;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;

    // Convert the image to grayscale
    cv::Mat input_gray;
    cv::cvtColor(input, input_gray, CV_BGR2GRAY);

    // Apply Laplacian transformation
    cv::Mat return_mat;
    cv::Laplacian(input_gray, return_mat, ddepth, kernel_size, scale, delta, cv::BORDER_DEFAULT);

    // Change format
    cv::Mat return_mat_formatted;
    return_mat.convertTo(return_mat_formatted, CV_8UC3);

    return return_mat_formatted;
}

cv::Mat MainWindow::getContours(cv::Mat input)
{
    cv::vector<cv::vector<cv::Point> > contours;
    cv::vector<cv::Vec4i> hierarchy;

    cv::findContours(input, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // Draw contours
    cv::Mat final = cv::Mat::zeros(input.size(), CV_8UC1 );

    for(uint i = 0; i < contours.size(); i++) {
        cv::Scalar color = cv::Scalar(255);
        cv::drawContours(final, contours, i, color, 1, 4, hierarchy, 2, cv::Point() );
    }

    return final;
}

cv::Mat MainWindow::getErodedImage(cv::Mat input, int erosion_size)
{
    cv::Mat element = getStructuringElement(cv::MORPH_RECT,
                                            cv::Size(2*erosion_size + 1, 2*erosion_size + 1),
                                            cv::Point(erosion_size, erosion_size));
    cv::Mat return_mat;

    // Apply the erosion operation
    cv::erode(input, return_mat, element);

    return return_mat;
}

cv::Mat MainWindow::getGaussianBlurSharpenedImage(cv::Mat input)
{
    cv::Mat temp;
    cv::Mat return_mat;
    cv::GaussianBlur(input, temp, cv::Size(0, 0), 3);
    cv::addWeighted(input, 3, temp, -1, 0, return_mat);
    return return_mat;
}

cv::Mat MainWindow::getMedianFilteredImage(cv::Mat input, int kernel_size)
{
    int kernelSize = kernel_size;

    if (kernelSize % 2 != 1)
        kernelSize = kernelSize - 1;

    if (kernelSize < 1)
        kernelSize = 1;

    cv::Mat return_mat;

    cv::medianBlur(input, return_mat, kernelSize);

    return return_mat;
}

cv::Mat MainWindow::processImageSet(cv::Mat before, cv::Mat after, cv::Mat referenceBefore, cv::Mat referenceAfter)
{
    cv::Mat final;
    cv::Mat backup;
    cv::Mat temp;
    cv::Mat before_loc;
    cv::Mat ref_before_loc;
    cv::Mat ref_after_loc;

    if (ui->upsampleCheckBox->isChecked()) {
        cv::pyrUp(after, final, cv::Size(final.cols*2, final.rows*2));
        cv::pyrUp(after, backup, cv::Size(final.cols*2, final.rows*2));
        cv::pyrUp(before, before_loc, cv::Size(final.cols*2, final.rows*2));
        cv::pyrUp(referenceBefore, ref_before_loc, cv::Size(final.cols*2, final.rows*2));
        cv::pyrUp(referenceAfter, ref_after_loc, cv::Size(final.cols*2, final.rows*2));
    } else {
        final = after;
        backup = after;
        before_loc = before;
        ref_before_loc = referenceBefore;
        ref_after_loc = referenceAfter;
    }

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
        ui->xposSlider->setMaximum(final.cols);
        ui->xposSlider->setEnabled(true);
        ui->yposSlider->setMinimum(0);
        ui->yposSlider->setMaximum(final.rows);
        ui->yposSlider->setEnabled(true);
        ui->widthSlider->setMinimum(0);
        ui->widthSlider->setMaximum(final.cols);
        ui->widthSlider->setEnabled(true);
        ui->heightSlider->setMinimum(0);
        ui->heightSlider->setMaximum(final.rows);
        ui->heightSlider->setEnabled(true);
    }

    if (ui->diffCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        cv::absdiff(before_loc, after, temp);
        final = temp;
    }

    if (ui->diffReferenceCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        cv::Mat referenceDiff;
        cv::absdiff(ref_before_loc, ref_after_loc, referenceDiff);
        cv::absdiff(final, referenceDiff, temp);
        final = temp;
    }

    cv::Rect roi;
    if (ui->cropingCheckBox->isChecked() || ui->cropingVisualizeCheckBox->isChecked()) {
        // Define a box that is our Region of Interest
        roi.x = ui->xposSlider->value();
        roi.y = ui->yposSlider->value();
        roi.width = (roi.x + ui->widthSlider->value() > final.cols) ? final.cols - roi.x : ui->widthSlider->value();
        roi.height = (roi.y + ui->heightSlider->value() > final.rows) ? final.rows - roi.y : ui->heightSlider->value();
    }

    if (ui->cropingCheckBox->isChecked()) {
        temp = final(roi);
        final = temp;
    } else if (ui->cropingVisualizeCheckBox->isChecked()) { // XXX: Should not be added to image when we start 3D capture
        cv::rectangle(final, roi, cv::Scalar(0, 255, 0));
    }

    if (ui->sharpeningCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        final = getGaussianBlurSharpenedImage(final);
    }

    if (ui->medianCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        // Post-processing to remove noise
        final = getMedianFilteredImage(final, ui->medianSlider->value());
    }

    if (ui->erosionCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        final = getErodedImage(final, ui->erosionSlider->value());
    }

    if (ui->cannyCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        // Detect edges using canny
        cv::Canny(final, temp, ui->cannySlider->value(), 255, 3 , true);

        final = getContours(temp);
    }

    if (ui->cropingCheckBox->isChecked()) {
        final.copyTo(backup(roi));
        final = backup;
    }

    if (ui->checkBox->isChecked() && ui->mainCheckBox->isChecked() && !ui->brightnessCheckBox->isChecked()) {
        cv::inRange(final, cv::Scalar(ui->blueSliderMin->value(), ui->greenSliderMin->value(), ui->redSliderMin->value()),
                         cv::Scalar(ui->blueSliderMax->value(), ui->greenSliderMax->value(), ui->redSliderMax->value()), temp);
        final = temp;
    }

    return final;
}

void MainWindow::performTriangulation(double amountRotated, cv::Mat img, cv::Mat color_img, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, bool leftSide)
{
    int cam_res_vert = img.rows;
    int cam_res_hor = img.cols;

    double radiansToDegrees = 180.0 / 3.14159265359;
    double degreesToRadians = 3.14159265359 / 180.0;

    double cam_focal_distance_pixels;

    // These values are not perfect, but the variance between cameras is good enough for now
    // Assumes hd resolution is used
    if (true) {
        // Camera parameters for Logitech C930e
        cam_focal_distance_pixels = cameraMatrix.at<double>(0, 0);
        //cam_focal_distance_pixels = 1163.93259;
        /*
        <FocalLength>
        <X>1163.93259</X>
        <Y>1166.94320</Y>
        </FocalLength>
        <PrincipalPoint>
        <X>931.83371</X>
        <Y>554.87796</Y>
        </PrincipalPoint>
        <Distortion>
        <k1>0.05243</k1>
        <k2>-0.12924</k2>
        <p1>0.00491</p1>
        <p2>0.00709</p2>
        </Distortion>
        </Camera>
        */
    } else {
        // Camera parameters for Kinect V2
        cam_focal_distance_pixels = 1081.37;
        //double cam_focal_distance_mm = 3.291;
        //double cam_cx = 959.5;
        //double cam_cy = 539.5;

        //double cam_fov_hor = 84.1; // Degrees
        //double cam_fov_vert = 53.8;
        //double cam_tilt_angle = 0.0; // Degrees
    }

    // If we have upsampled then we will need to double the focal distance
    if (ui->upsampleCheckBox->isChecked()) {
        cam_focal_distance_pixels *= 2;
    }

    double laser_angle = ui->degreeLaserAngle->value();

    double cam_center_dist = 300.0; // mm
    double laser_cam_dist = cam_center_dist*tan(laser_angle*degreesToRadians);
    double laser_center_dist = cam_center_dist / cos(laser_angle*degreesToRadians);

    int startY;
    int endY;
    int startX;
    int endX;

    // If a area of intereset has been set then we should take that into account
    if (ui->cropingCheckBox->isChecked()) {
        startY = ui->yposSlider->value();
        startX = ui->xposSlider->value();
        endY = ui->heightSlider->value() == 0 ? cam_res_vert : ui->yposSlider->value() + ui->heightSlider->value();
        endX = ui->widthSlider->value() == 0 ? cam_res_hor : ui->xposSlider->value() + ui->widthSlider->value();
    } else {
        startY = 0;
        startX = 0;
        endY = img.rows;
        endX = img.cols;
    }

    // Safeguard against trying to acces stuff outside the image
    endY = (endY > img.rows) ? img.rows : endY;
    endX = (endX > img.cols) ? img.cols : endX;

    std::cout << "Getting image between " << startY << ", " << startX << " and " << endY << ", " << endX << std::endl;

    int radCounter = 0;
    double radSum = 0.0;

    for (int y = startY; y < endY; y++) {

        bool found_first_x = false;
        int first_x = -1;
        int last_x = -1;
        int brightestX = 0;

        for (int x = startX; x < endX; x++) {
            //cv::Scalar intensity = img.at<uchar>(y, x);
            cv::Vec3b intensity = img.at<cv::Vec3b>(y, x);

            uchar blue = intensity.val[0];
            uchar green = intensity.val[1];
            uchar red = intensity.val[2];

            if (ui->brightnessCheckBox->isChecked()) {

                // XXX: This is a hacked up weighting algorithm, giving more weight to blue, and less to red
                int sum = blue*3 + green*0.5 + red*0.5;

                if (sum > 70 && sum > brightestX) {
//                    std::cout << "  Pixel at " << y << ", " << x << " is " << sum << std::endl;
                    found_first_x = true;
                    brightestX = x;
                }
            } else {
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
        }

        // If we are using brightness values, we want to continue with the next row if we did not find a decently large value
        if (ui->brightnessCheckBox->isChecked() && brightestX == 0) {
            continue;
        }

        if (found_first_x) {
            double average_x = ui->brightnessCheckBox->isChecked() ? (double) brightestX : (double) (first_x + last_x)/2;

            if (average_x < 200.0 || average_x > 1800.0) {
                std::cout << "averagex has weird value of " << average_x << std::endl;
                continue;
            }

            double pixel_angle_x;
            if (leftSide) {
                pixel_angle_x = atan2(average_x - (cam_res_hor/2.0), cam_focal_distance_pixels)*radiansToDegrees;
            } else {
                pixel_angle_x = atan2(average_x - (cam_res_hor/2.0), cam_focal_distance_pixels)*radiansToDegrees;
            }

            double pixel_angle_hor_x = (90.0 - pixel_angle_x);
            double laser_hor_angle = (180.0 - 90.0 - laser_angle);

            if (laser_hor_angle < 0.0)
                std::cout << "Hor_angle negative" << std::endl;

            double laser_cam_angle = 180.0 - pixel_angle_hor_x - laser_hor_angle;

            if (laser_cam_angle < 0.0) {
                // XXX: This means that the scanning is going through the center, and out on the other side.
                //std::cout << "Laser_cam_angle negative" << std::endl;
                //std::cout << "Pixel_angle_hor_x is " << pixel_angle_hor_x << " and laser_hor_angle is " << laser_hor_angle << std::endl;
                continue;
            }

            double laser_cam_plane_dist = laser_cam_dist*sin(pixel_angle_hor_x*degreesToRadians)/sin(laser_cam_angle*degreesToRadians);

            if (laser_cam_dist < 0.0)
                std::cout << "Laser cam dist negative" << std::endl;

            if (laser_cam_plane_dist < 0.0)
                std::cout << "Laser cam plane dist negative" << std::endl;

            double radius = laser_center_dist - laser_cam_plane_dist;

            // Don't bother adding the point if it is outside our defined radius
            if (radius > ui->radiusSlider->value() && ui->radiusSlider->value() != 0) {
                std::cout << "Radius to big" << std::endl;
                continue;
            }

            if (pixel_angle_hor_x < 0.0 || pixel_angle_hor_x > 180.0 - laser_hor_angle) {
                std::cout << "Pixel angle hor x has weird value of " << pixel_angle_hor_x << std::endl;
                continue;
            }

            // Radius is negative, that's not good
            if (radius < 0.0) {
                //std::cout << "Radius negative" << std::endl;
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

            pcl::PointXYZRGB *point = new pcl::PointXYZRGB;
            if (ui->diffCheckBox->isChecked()) {
                cv::Vec3b bgrPixel = color_img.at<cv::Vec3b>(y, average_x);
                point->r = bgrPixel[0];
                point->g = bgrPixel[1];
                point->b = bgrPixel[2];
            } else {
                point->r = 100;
                point->g = 100;
                point->b = 100;
            }
            point->x = x_pos;
            point->y = y_pos;
            point->z = z_pos;

            cloud->push_back(*point);
        }

        double averageRad = radSum/radCounter;
        //std::cout << "Radius is: " << averageRad << std::endl;
        radSum = 0.0;
        radCounter = 0;
    }
    std::cout << "We are done calculating the stuffs" << std::endl;
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
    url.append(QString("%1").arg(number, 3, 10, QChar('0')));

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

cv::Mat MainWindow::restore_picture(int number, bool after, bool reference, bool final, bool leftSide)
{
    QString url = ui->textbox_path->toPlainText();
    url.append("/");
    url.append(ui->fileName->text());
    url.append("_");
    url.append(QString("%1").arg(number, 3, 10, QChar('0')));

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
    return cv::imread(url.toUtf8().constData());
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

void MainWindow::on_button_cont_scan_clicked(bool check)
{
//    performContScan2();
    // We need to spawn a worker thread as to not block the GUI
    boost::thread* thr = new boost::thread(boost::bind(&MainWindow::performContScan2, this));
}

void MainWindow::on_button_auto_scan_clicked(bool check)
{
    // We need to spawn a worker thread as to not block the GUI
    boost::thread* thr = new boost::thread(boost::bind(&MainWindow::performAutoScan, this));
}

void MainWindow::grabAndSave(int number, bool after, bool reference, bool final, bool leftSide)
{
    std::cout << "Ran Grab&Save" << std::endl;
    cv::Mat pic;
    if (left && right) {
        // Get an image with both lasers on
        pic = getImage(3);
    } else if (left) {
        // Get an image with left laser on
        pic = getImage(1);
    } else if (right) {
        // Get an image with right laser on
        pic = getImage(2);
    } else {
        // Get an image with the lasers off
        pic = getImage(0);
    }
    save_picture(pic, number, false, false, false, true);

    if (isImagePipelineReady()) {
        matToShow = pic.clone();
        new_mat_to_convert = true;
    }
}

void MainWindow::grabAndSaveBefore()
{
    grabAndSave(counter, false, false, false, false);
    counter++;
}

void MainWindow::grabAndSaveAfterLeft()
{
    grabAndSave(counter, true, false, false, true);
    counter++;
}

void MainWindow::grabAndSaveAfterRight()
{
    grabAndSave(counter, true, false, false, false);
    counter++;
}

void MainWindow::performContScan2()
{
    wait_for_scan = true;

    double old_val = ui->current_position->value();

    counter = 0;

    if (ui->diffCheckBox->isChecked()) {
        std::cout << "Let's get the before images" << std::endl;

        timer2 = new QTimer(this);
        connect(timer2, SIGNAL(timeout()), this, SLOT(grabAndSaveBefore()));

        Q_EMIT setAngle(old_val + 360.0);
        timer->start(30);

        while (ui->current_position->value() < old_val + 359.7);
        timer->stop();

        // Move into position for next round
        Q_EMIT setAngle(old_val + 360.0);

        old_val = old_val + 360;
    }

    int beforeCounter = counter;

    if (ui->toggleUseLeftLaserCheckBox->isChecked()) {
        std::cout << "Let's get the left after images" << std::endl;
        counter = 0;

        timer2 = new QTimer(this);
        connect(timer2, SIGNAL(timeout()), this, SLOT(grabAndSaveAfterLeft()));

        Q_EMIT setAngle(old_val + 360.0);
        timer->start(30);

        while (ui->current_position->value() < old_val + 359.7);
        timer->stop();

        // Move into position for next round
        Q_EMIT setAngle(old_val + 360.0);

        old_val = old_val + 360;
    }

    int afterLeftCounter = counter;

    if (ui->toggleUseRightLaserCheckBox->isChecked()) {
        std::cout << "Let's get the right after images" << std::endl;
        counter = 0;

        timer2 = new QTimer();
        connect(timer2, SIGNAL(timeout()), this, SLOT(grabAndSaveAfterRight()));

        Q_EMIT setAngle(old_val + 360.0);
        timer->start(30);

        while (ui->current_position->value() < old_val + 359.7);
        timer->stop();
    }

    int afterRightCounter = counter;

    if (beforeCounter < counter && beforeCounter != 0)
        counter = beforeCounter;
    if (afterRightCounter < counter && afterRightCounter != 0)
        counter = afterRightCounter;
    if (afterLeftCounter < counter && afterLeftCounter != 0)
        counter = afterLeftCounter;

    double deg_per_pic = 360 / counter;

    int iterations = counter;

    // We have not set the point cloud viewer visible, we need to do that
    spawnPointCloudWidget();

    // Change from image to point cloud visualization
    drawPointCloud = true;

    for (int i = 0; i < iterations; i++) {
        cv::Mat local_filtered;
        cv::Mat before;

        if (ui->diffCheckBox->isChecked()) {
            // Get the image with the lasers off
            before = restore_picture(i, false, false, false, true);
        }

        if (ui->toggleUseLeftLaserCheckBox->isChecked()) {
            // Get the image with the left laser on
            cv::Mat after = restore_picture(i, true, false, false, true);

            // Process image and perform triangulation
            local_filtered = processImageSet(before, after, referenceBefore, referenceAfter);
            performTriangulation(deg_per_pic*((double) i), local_filtered, before, this->cloud, true);
            save_picture(local_filtered, i, false, false, true, true);
        }

        if (ui->toggleUseRightLaserCheckBox->isChecked()) {
            // Get an image with the right laser on
            cv::Mat after = restore_picture(i, true, false, false, false);

            // Process image and perform triangulation
            local_filtered = processImageSet(before, after, referenceBefore, referenceAfter);
            performTriangulation(deg_per_pic*((double) i), local_filtered, before, this->cloud, false);
            save_picture(local_filtered, i, false, false, true, false);
        }

        if (isImagePipelineReady()) {
            matToShow = local_filtered.clone();
            new_mat_to_convert = true;
        }

        // Update the point-cloud-viewer with the newly scanned strips
        if (!wait_for_point_cloud_viewer)
            Q_EMIT updatePointCloudViewer();
    }

    // We need to do a simple spin() of the point cloud viewer to not block it when it is done
    pointCloudViewer->spin();

    // Save the point cloud to a pcd file
    QString url = ui->textbox_path->toPlainText();
    url.append("/");
    url.append(ui->fileName->text());
    url.append(".pcd");
    pcl::io::savePCDFileBinary(url.toUtf8().constData(), *cloud);

    wait_for_scan = false;
}

void MainWindow::performContScan()
{
    wait_for_scan = true;

    double deg_per_pic = ui->degree_auto_scan->value();
    int iterations = (int) ((double) 360 / deg_per_pic);
    double shadow_position = 0.0;

    int i = 0;

    double old_val = ui->current_position->value();

    std::cout << "Let's get the before images" << std::endl;

    for (i = 0; i < iterations; i++) {
        std::cout << "  Before image number " << i << std::endl;
    //while (ui->current_position->value() < old_val + 360.0) {
        double new_pos = old_val + i*deg_per_pic;

        // Get an image with the lasers off
        cv::Mat before = getImage(0);
        save_picture(before, i, false, false, false, true);

        if (isImagePipelineReady()) {
            matToShow = before.clone();
            new_mat_to_convert = true;
        }

        // Increment the angle
        Q_EMIT setAngle(new_pos);

        std::cout << "      Done with number " << i << std::endl;
        //i++;
    }

    i = 0;
    old_val = ui->current_position->value();

    if (ui->toggleUseLeftLaserCheckBox->isChecked()) {
        std::cout << "Let's get the left after images" << std::endl;
        for (i = 0; i < iterations; i++) {
            std::cout << "  Left after image number " << i << std::endl;
        //while (ui->current_position->value() < old_val + 360.0) {
            double new_pos = old_val + i*deg_per_pic;

            // Get an image with the left laser on
            cv::Mat after = getImage(1);
            save_picture(after, i, true, false, false, true);

            if (isImagePipelineReady()) {
                matToShow = after.clone();
                new_mat_to_convert = true;
            }

            // Increment the angle
            Q_EMIT setAngle(new_pos);

            std::cout << "      Done with number " << i << std::endl;
            //i++;
        }
    }

    i = 0;
    old_val = ui->current_position->value();

    if (ui->toggleUseRightLaserCheckBox->isChecked()) {
        std::cout << "Let's get the right after images" << std::endl;
        for (i = 0; i < iterations; i++) {
            std::cout << "  Right after image number " << i << std::endl;
        //while (ui->current_position->value() < old_val + 360.0) {
            double new_pos = old_val + i*deg_per_pic;

            // Get an image with the right laser on
            cv::Mat after = getImage(2);
            save_picture(after, i, true, false, false, false);

            if (isImagePipelineReady()) {
                matToShow = after.clone();
                new_mat_to_convert = true;
            }

            // Increment the angle
            Q_EMIT setAngle(new_pos);

            std::cout << "      Done with number " << i << std::endl;
            //i++;
        }
    }

    i = 0;

    // We have not set the point cloud viewer visible, we need to do that
    spawnPointCloudWidget();

    // Change from image to point cloud visualization
    drawPointCloud = true;

    for (i = 0; i < iterations; i++) {
        cv::Mat local_filtered;

        // Get the image with the lasers off
        cv::Mat before = restore_picture(i, false, false, false, true);

        if (ui->toggleUseLeftLaserCheckBox->isChecked()) {
            // Get the image with the left laser on
            cv::Mat after = restore_picture(i, true, false, false, true);

            // Process image and perform triangulation
            local_filtered = processImageSet(before, after, referenceBefore, referenceAfter);
            performTriangulation(deg_per_pic*((double) i), local_filtered, before, this->cloud, true);
            save_picture(local_filtered, i, false, false, true, true);
        }

        if (ui->toggleUseRightLaserCheckBox->isChecked()) {
            // Get an image with the right laser on
            cv::Mat after = restore_picture(i, true, false, false, false);

            // Process image and perform triangulation
            local_filtered = processImageSet(before, after, referenceBefore, referenceAfter);
            performTriangulation(deg_per_pic*((double) i), local_filtered, before, this->cloud, false);
            save_picture(local_filtered, i, false, false, true, false);
        }

        if (isImagePipelineReady()) {
            matToShow = local_filtered.clone();
            new_mat_to_convert = true;
        }

        // Update the point-cloud-viewer with the newly scanned strips
        if (!wait_for_point_cloud_viewer)
            Q_EMIT updatePointCloudViewer();

    }

    // We need to do a simple spin() of the point cloud viewer to not block it when it is done
    pointCloudViewer->spin();

    // Save the point cloud to a pcd file
    QString url = ui->textbox_path->toPlainText();
    url.append("/");
    url.append(ui->fileName->text());
    url.append(".pcd");
    pcl::io::savePCDFileBinary(url.toUtf8().constData(), *cloud);

    wait_for_scan = false;
}

void MainWindow::performAutoScan()
{
    wait_for_scan = true;
    // XXX: Deactivate filter controls


    // We have not set the point cloud viewer visible, we need to do that
    spawnPointCloudWidget();

    // Change from image to point cloud visualization
    drawPointCloud = true;

    double deg_per_pic = ui->degree_auto_scan->value();
    int iterations = (int) ((double) 360 / deg_per_pic);
    double shadow_position = 0.0;

    int i = 0;

    double old_val = ui->current_position->value();

    while (ui->current_position->value() < old_val + 360.0) {
        double new_pos = old_val + i*deg_per_pic;

        cv::Mat local_filtered;
        cv::Mat before;
        cv::Mat after;

        if (ui->diffCheckBox->isChecked()) {
            // Get an image with the lasers off
            before = getImage(0);
            save_picture(before, i, false, false, false, true);
        }

        if (ui->toggleUseLeftLaserCheckBox->isChecked()) {
            // Get an image with the left laser on
            cv::Mat after = getImage(1);
            save_picture(after, i, true, false, false, true);

            // Process image and perform triangulation
            local_filtered = processImageSet(before, after, referenceBefore, referenceAfter);
            performTriangulation(deg_per_pic*((double) i), local_filtered, before, this->cloud, true);
            save_picture(local_filtered, i, false, false, true, true);
        }

        if (ui->toggleUseRightLaserCheckBox->isChecked()) {
            // Get an image with the right laser on
            after = getImage(2);
            save_picture(after, i, true, false, false, false);

            // Process image and perform triangulation
            local_filtered = processImageSet(before, after, referenceBefore, referenceAfter);
            performTriangulation(deg_per_pic*((double) i), local_filtered, before, this->cloud, false);
            save_picture(local_filtered, i, false, false, true, false);
        }

        if (isImagePipelineReady()) {
            matToShow = local_filtered.clone();
            new_mat_to_convert = true;
        }

        // Update the point-cloud-viewer with the newly scanned strips
        if (!wait_for_point_cloud_viewer)
            Q_EMIT updatePointCloudViewer();

        // Increment the angle
        //Q_EMIT setAngle(new_pos);

        qnode.setAngle(new_pos);

        // Wait until table has turned into position
        // while (qnode.waitingForAngle());

        i++;
    }

    std::cout << "Done autoscanning" << std::endl;

    // We need to do a simple spin() of the point cloud viewer to not block it when it is done
    pointCloudViewer->spin();

    ui->button_auto_scan->setEnabled(false);

    // Save the point cloud to a pcd file
    QString url = ui->textbox_path->toPlainText();
    url.append("/");
    url.append(ui->fileName->text());
    url.append(".pcd");
    pcl::io::savePCDFileBinary(url.toUtf8().constData(), *cloud);

    wait_for_scan = false;
}

static const QString type2str(int type) {
  QString r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

cv::Mat MainWindow::getImage(int lasers)
{
    if (ui->opencvCheckBox->isChecked()) {
        cv::Mat image;

//        while (qnode.waitingForAngle() || qnode.waitingForLaser())
//            std::cout << "STUCK IN GET IMAGE " << std::endl;;

        if (!(ui->toggleLeftLaserCheckBox->isChecked() || ui->toggleRightLaserCheckBox->isChecked())) {
            // Take picture with laser
            qnode.setLasers(lasers);
        }

        cap >> image;

        assert(!image.empty());

        assert(image.type() == CV_8UC3);

        cv::Mat image2;
        cv::undistort(image, image2, cameraMatrix, distCoeffs);

        return image2;

    } else {
        while (qnode.waitingForAngle() || qnode.waitingForLaser() || qnode.waitingForPic() || qnode.waitingForPicProcessing());

        if (!(ui->toggleLeftLaserCheckBox->isChecked() || ui->toggleRightLaserCheckBox->isChecked())) {
            // Take picture with laser
            qnode.setLasers(lasers);
        }
        cv::Mat image = qnode.getCurrentImage();
        cv::Mat image2;
        cv::undistort(image, image2, cameraMatrix, distCoeffs);

        return image2;
    }
}

void MainWindow::updateFilteredImage(bool left, bool right)
{
    wait_for_processing_pic = true;

    cv::Mat after;
    cv::Mat before;

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

    if (ui->diffCheckBox->isChecked()) {
        // Get an image with the lasers off
        before = getImage(0);
    }

    if (ui->mainCheckBox->isChecked()) {

        std::cout << " Printing before size " << before.size << " And the after size" << after.size << std::endl;

        matToShow = processImageSet(before, after, referenceBefore, referenceAfter).clone();
    } else {
        matToShow = after.clone();
    }

    new_mat_to_convert = true;

    wait_for_processing_pic = false;
}

bool MainWindow::isImagePipelineReady()
{
    return !wait_for_processing_pic && !new_pic_to_present && !new_mat_to_convert;
}

void MainWindow::updateView()
{
    // If there is not yet any image to process then we shouldn't try to
    if (qnode.pictureHasBeenSet() || ui->opencvCheckBox->isChecked()) {
        // If we have not yet reached the pointcloud stage we should show the image
        if (!drawPointCloud && isImagePipelineReady()  && !wait_for_scan) {

            // XXX: Debug, get time to check processing time
            gettimeofday(&start, NULL);

            wait_for_processing_pic = true; // Need to set this before launching the thread, or else we will fall through on the next if
            boost::thread* thr = new boost::thread(boost::bind(&MainWindow::updateFilteredImage, this,
                                                               ui->toggleUseLeftLaserCheckBox->isChecked(),
                                                               ui->toggleUseRightLaserCheckBox->isChecked()));
        }

        if (new_mat_to_convert) {
            boost::thread* thr = new boost::thread(boost::bind(&MainWindow::updateImageToShow, this,
                                                               this->matToShow));
        }

        if (new_pic_to_present) {
            QPixmap pixMap = QPixmap::fromImage(toShow);

            // Add the pixmap to the label that presents it
            ui->imageLabel->setPixmap(pixMap);
            new_pic_to_present = false;

            timeval endTime;

            long seconds, useconds;
            double duration;

            gettimeofday(&endTime, NULL);

            seconds  = endTime.tv_sec  - start.tv_sec;
            useconds = endTime.tv_usec - start.tv_usec;

            duration = seconds + useconds/1000000.0;

            std::cout << "The processing of the images took " << duration << " seconds" << std::endl;
        }
    }
}

void MainWindow::updateImageToShow(cv::Mat image)
{
    toShow = mat2qimage(image);
    new_mat_to_convert = false;
    new_pic_to_present = true;
}

void MainWindow::updatePointCloudViewer()
{
    wait_for_point_cloud_viewer = true;

    // If we have not added the pointcloud to the viewer then we should do that
    if (this->cloud->empty()) {
        wait_for_point_cloud_viewer = false;
        return;
    }

    if (!pointCloudViewer->updatePointCloud(this->cloud, "displayCloud")) {
        pointCloudViewer->addPointCloud(this->cloud, "displayCloud");
        pointCloudViewer->spinOnce();
    } else {
        pointCloudViewer->spinOnce();
    }

    wait_for_point_cloud_viewer = false;
}

void MainWindow::spawnPointCloudWidget()
{
    // Set up the point cloud viewer
    w = new QVTKWidget();
    pointCloudViewer.reset(new pcl::visualization::PCLVisualizer ("viewer", true));
    pointCloudWidgetAdded = true;
}

}  // namespace LaserScannerApplication

