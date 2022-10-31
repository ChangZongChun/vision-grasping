#include <libfreenect/libfreenect.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <cmath>

#include <pthread.h>
#include <libusb.h>

#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui_c.h>

#include <libfreenect/libfreenect.h>
 

using namespace cv;
using namespace std;


class myMutex {
    public:
    myMutex() {
        pthread_mutex_init( &m_mutex, NULL );
    }
    void lock() {
        pthread_mutex_lock( &m_mutex );
    }
    void unlock() {
        pthread_mutex_unlock( &m_mutex );
    }
    private:
        pthread_mutex_t m_mutex;
};


class MyFreenectDevice : public Freenect::FreenectDevice {
    public:
    // freenect_context *_ctx;
    MyFreenectDevice(freenect_context *_ctx, int _index) : Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_DEPTH_11BIT),
    m_buffer_rgb(FREENECT_VIDEO_RGB), m_gamma(2048), m_new_rgb_frame(false),
    m_new_depth_frame(false), depthMat(Size(640,480),CV_16UC1),
    rgbMat(Size(640,480), CV_8UC3, Scalar(0)),
    ownMat(Size(640,480),CV_8UC3,Scalar(0)) {

        for( unsigned int i = 0 ; i < 2048 ; i++) {
            float v = i/2048.0;
            v = std::pow(v, 3)* 6;
            m_gamma[i] = v*6*256;
        }
}

    // Do not call directly even in child
    void VideoCallback(void* _rgb, uint32_t timestamp) {
        std::cout << "RGB callback" << std::endl;
        m_rgb_mutex.lock();
        uint8_t* rgb = static_cast<uint8_t*>(_rgb);
        rgbMat.data = rgb;
        m_new_rgb_frame = true;
        m_rgb_mutex.unlock();
    };

    // Do not call directly even in child
    void DepthCallback(void* _depth, uint32_t timestamp) {
        std::cout << "Depth callback" << std::endl;
        m_depth_mutex.lock();
        uint16_t* depth = (uint16_t*)(_depth);
        depthMat.data = (uchar*) depth;
        m_new_depth_frame = true;
        m_depth_mutex.unlock();
    }


    bool getVideo(Mat& output) {
        m_rgb_mutex.lock();
        if(m_new_rgb_frame) {
            cv::cvtColor(rgbMat, output, CV_RGB2BGR);
            m_new_rgb_frame = false;
            m_rgb_mutex.unlock();
            return true;
        } 
        else {
            m_rgb_mutex.unlock();
            return false;
        }
    }

    bool getDepth(Mat& output) {
        m_depth_mutex.lock();
        if(m_new_depth_frame) {
            depthMat.copyTo(output);
            m_new_depth_frame = false;
            m_depth_mutex.unlock();
            return true;
        } 
        else {
            m_depth_mutex.unlock();
            return false;
        }
    }


    // bool smoothDepth (Mat& output) {
    //     m_depth_mutex.lock();
    //     if(m_new_depth_frame) {
    //         int width = 640;
    //         int height = 480;
    //         int widthBound = width -1;
    //         int heightBound = height -1;

    //         for(int rowidx = 0 ; rowidx < width; rowidx++) {
    //             for(int colidx = 0 ; colidx < height ; colidx++) {
    //                 // int depthIndex = rowidx + (colidx * height);
    //                 unsigned char currentPixel = depthMat.at<int>(rowidx, colidx);
    //                 if( currentPixel == 0) {

    //                     int innerBandCount = 0;
    //                     int outerBandCount = 0;

    //                     int filterCollection[24][2];

    //                     // loop through a 5 X 5 matrix of pixels surrounding the candidate pixel.
    //                     for (int yi = -2; yi < 3; yi++) {
    //                         for (int xi = -2; xi < 3; xi++) {
    //                             // the candidate pixel (xi = 0, yi = 0) in our process at this point. We already know that it's 0
    //                             if (xi != 0 || yi != 0) {

    //                                 int xSearch = rowidx + xi;
    //                                 int ySearch = colidx + yi;

    //                                 // out of actual matrix
    //                                 if (xSearch >= 0 && xSearch <= widthBound && ySearch >= 0 && ySearch <= heightBound) {
    //                                     // We only want to look for non-0 values
    //                                     if (depthMat.at<int>(xSearch, ySearch) != 0) {
    //                                         for (int i = 0; i < 24; i++) {
    //                                             if (filterCollection[i][0] == depthMat.at<int>(xSearch, ySearch)) {
    //                                                 filterCollection[i][1]++;
    //                                                 break;
    //                                             }
    //                                             else if (filterCollection[i, 0] == 0) {
    //                                                 filterCollection[i][0] = depthMat.at<int>(xSearch, ySearch);
    //                                                 filterCollection[i][1]++;
    //                                                 break;
    //                                             }
    //                                         }
    //                                         if (yi != 2 && yi != -2 && xi != 2 && xi != -2) {
    //                                             innerBandCount++;
    //                                         }
    //                                         else {
    //                                             outerBandCount++;
    //                                         }
    //                                     }
    //                                 }
    //                             }

    //                         }
    //                     }
                        
    //                     int innerBandThreshold = 5;
    //                     int outerBandThreshold = 9;
    //                     if (innerBandCount >= innerBandThreshold || outerBandCount >= outerBandThreshold) {
    //                         short frequency = 0;
    //                         short depth = 0; 
    //                         for (int i = 0; i < 24; i++) {
    //                             if (filterCollection[i][0] == 0) {
    //                                 break;
    //                             }
    //                             if (filterCollection[i][1] > frequency) {
    //                                 depth = filterCollection[i][0];
    //                                 frequency = filterCollection[i][1];
    //                             }
    //                         }

    //                         currentPixel = depth;;
    //                     }
    //                 }
    //                 else {
    //                     depthMat.at<int>(rowidx, colidx) = currentPixel;
    //                 }
    //             }
    //         }

    //         depthMat.copyTo(output);
    //         m_new_depth_frame = false;
    //         m_depth_mutex.unlock();
    //         return true;
    //     } 
    //     else {
    //         m_depth_mutex.unlock();
    //         return false;
    //     }
    // }

    private:
    std::vector<uint8_t> m_buffer_depth;
    std::vector<uint8_t> m_buffer_rgb;
    std::vector<uint16_t> m_gamma;
    Mat depthMat;
    Mat rgbMat;
    Mat ownMat;
    myMutex m_rgb_mutex;
    myMutex m_depth_mutex;
    bool m_new_rgb_frame;
    bool m_new_depth_frame;
};


int main(int argc, char **argv) {
    bool die(false);
    string filename_rgb("color_");
    string filename_depth("depth_");
    string suffix(".png");
    int i_snap(0),iter(0);

    Mat depthMat(Size(640,480),CV_16UC1);
    Mat depthf (Size(640,480),CV_8UC1);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
    Mat ownMat(Size(640,480),CV_8UC3,Scalar(0));

    // The next two lines must be changed as Freenect::Freenect
    // isn't a template but the method createDevice:
    // Freenect::Freenect<MyFreenectDevice> freenect;
    // MyFreenectDevice& device = freenect.createDevice(0);
    // by these two lines:

    Freenect::Freenect freenect;
    MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);
    device.setDepthFormat(FREENECT_DEPTH_REGISTERED);
    // device.setDepthFormat(FREENECT_DEPTH_MM);

    namedWindow("rgb",CV_WINDOW_AUTOSIZE);
    namedWindow("depth",CV_WINDOW_AUTOSIZE);
    // device.set_image_registration_mode(1);
    device.startVideo();
    device.startDepth();
    while (!die) {
        device.getVideo(rgbMat);
        device.getDepth(depthMat);
        // device.smoothDepth(depthMat);
        cv::imshow("rgb", rgbMat);
        depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
        cv::imshow("depth",depthf);
        char k = cvWaitKey(100);
        if( k == 27 ) {
            std::cout << "exit" << std::endl;
            cvDestroyWindow("rgb");
            cvDestroyWindow("depth");
            break;
        }
        if( k == 's' ) {
            std::ostringstream file_rgb;
            std::ostringstream file_depth;
            file_rgb << filename_rgb << i_snap << suffix;
            cv::imwrite(file_rgb.str(),rgbMat);
            file_depth << filename_depth << i_snap << suffix;
            cv::imwrite(file_depth.str(),depthMat);
            std::cout << "save" << std::endl;
            i_snap++;
        }
        // if(iter >= 1000) break;
        // iter++;
    }

    device.stopVideo();
    device.stopDepth();
    return 0;
}