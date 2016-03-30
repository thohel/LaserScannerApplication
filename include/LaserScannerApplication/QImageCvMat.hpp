/**
 * @file /include/LaserScannerApplication/main_window.hpp
 *
 * @brief Qt based gui for LaserScannerApplication.
 *
 * @date November 2010
 **/
#ifndef QImageCvMat
#define QImageCvMat

static QImage mat2qimage(cv::Mat& mat) {
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


#endif // QImageCvMat
