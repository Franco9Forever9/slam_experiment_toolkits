#ifndef _DENOISE_FILTERING_H_
#define _DENOISE_FILTERING_H_

#include <queue>
#include <vector>
#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>

std::chrono::time_point<std::chrono::system_clock> start, end;
std::string median = "median";
std::string equalize = "equalize";
std::string laplacestr = "laplace";
std::mutex imgLock;
ros::Subscriber subRawImg;
ros::Publisher pubMedianImg;
ros::Publisher pubEqualizedImg;
ros::Publisher pubSharpenedImg;
std::queue<sensor_msgs::ImageConstPtr> imgBuf;

void tic()
{
    start = std::chrono::system_clock::now();
}

void toc(const char* section)
{
    end = std::chrono::system_clock::now();
    std::chrono::duration<double, std::milli> time = end - start;
    std::cout << "--- >> " << section << " : " << time.count() << "<< ---" << std::endl;
}

// 只对灰度图像使用
cv::Mat median_filtering(const cv::Mat &src, int size)
{
    cv::Mat dst = src.clone();
    int edge = size / 2;
    
    for(int row = edge; row < dst.rows - edge; row++){
        for(int col = edge; col < dst.cols - edge; col++){
            std::vector<uchar> core;
            for(int m = row - edge; m <= row + edge; m++){
                for(int n = col - edge; n <= col + edge; n++){
                    core.push_back(src.at<uchar>(m,n));
                }
            }
            std::sort(core.begin(), core.end());
            dst.at<uchar>(row,col) = core[(int)(size * size / 2)];
        }
    }
    return dst;
}
// 只对灰度图像使用
cv::Mat mean_filtering(const cv::Mat &src, int size)
{
    cv::Mat dst = src.clone();
    int edge = size / 2;
    
    for(int row = edge; row < dst.rows - edge; row++){
        for(int col = edge; col < dst.cols - edge; col++){
            int sum = 0;
            for(int m = row - edge; m <= row + edge; m++){
                for(int n = col - edge; n <= col + edge; n++){
                    sum += src.at<uchar>(m,n);
                }
            }

            dst.at<uchar>(row,col) = (int)(sum / size / size);
        }
    }
    return dst;
}

// cv::Mat guassian_filtering(const cv::Mat &src, int size, double sigma)
// {
//     std::vector<std::vector<double>> guassianTemplete = guassian_template(size, sigma);
//     cv::Mat dst = src.clone();
//     int edge = size / 2;

//     for (int row = edge; row < dst.rows - edge; row++){
//         for (int col = edge; col < dst.cols - edge; col++){
//             int sum = 0;
//             for (int m = row - edge; m <= row + edge; m++){
//                 for (int n = col - edge; n <= col + edge; n++){
//                     sum += src.at<uchar>(m, n) * guassianTemplete[m - row + edge][n - col + edge];
//                 }
//             }

//             dst.at<uchar>(row, col) = (uchar)sum;
//         }
//     }

//     return dst;
// }

// std::vector<std::vector<double>> guassian_template(int size, double sigma)
// {

// }

// 只对灰度图像使用
cv::Mat laplace_filtering(const cv::Mat &src, cv::Mat &_operator)
{
    cv::Mat dst = src.clone();
    int size = _operator.rows;
    int edge = size / 2;
    
    for(int row = edge; row < dst.rows - edge; row++){
        for(int col = edge; col < dst.cols - edge; col++){
            int i = 0, j = 0, sum = 0;
            for(int m = row - edge; m <= row + edge; m++){
                for(int n = col - edge; n <= col + edge; n++){
                    sum += _operator.at<int>(i,j) * src.at<uchar>(m,n);
                    j += 1;
                }
                i += 1;
            }
            dst.at<uchar>(row,col) = sum;
        }
    }
    return dst;
}

cv::Mat convert_to_grayscale(cv::Mat &src)
{
    cv::Mat dst = src.clone();
    cv::cvtColor(src, dst, cv::COLOR_RGB2GRAY);
    return dst;
}

void histo_equalize(cv::Mat &src, cv::Mat &dst)
{
    cv::equalizeHist(src,dst);
}

void edge_enhance(cv::Mat &src, cv::Mat &dst)
{
    std::vector<cv::Mat> grayImgs;

    if(!dst.empty()) dst.release();
    if(src.channels() == 3) cv::split(src, grayImgs);
    else if(src.channels() == 1) grayImgs.push_back(src);

    for(size_t i = 0; i < grayImgs.size(); i++)
    {
        cv::Mat blurImg;
        cv::Mat sharpImg;
        cv::Mat sharpImg8U;
        
        cv::GaussianBlur(grayImgs[i], blurImg, cv::Size(3,3), 0, 0);
        cv::Laplacian(blurImg, sharpImg, 3, 3);
        sharpImg.convertTo(sharpImg8U, 0);
        cv::add(grayImgs[i], sharpImg8U, grayImgs[i]);
    }

    cv::merge(grayImgs, dst);
}

void raw_image_process()
{
    ROS_INFO("--- image processing thread ---");
    
    while(true){
        if(imgBuf.empty() == false){
            std::lock_guard<std::mutex> _lock(imgLock);
            cv_bridge::CvImagePtr cv_ptr = nullptr; sensor_msgs::ImageConstPtr msg_ptr = nullptr;
            cv::Mat img, medianImg, equalizedImg, sharpenedImg;

            try{
                cv_ptr = cv_bridge::toCvCopy(imgBuf.front(), sensor_msgs::image_encodings::TYPE_8UC3);          
                img = convert_to_grayscale(cv_ptr->image);

                tic();
                cv::Mat laplace = (cv::Mat_<int>(3,3) << 0,1,0,1,-8,1,0,1,0);
                // cv::Laplacian(equalizedImg, sharpenedImg, 0, 7);
                edge_enhance(img, sharpenedImg);
                toc("laplace");

                tic();
                cv::medianBlur(img,medianImg,5);
                toc("median");

                tic();
                histo_equalize(medianImg, equalizedImg);
                toc("equalize");


                cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
                cv_ptr->image = medianImg;
                msg_ptr = cv_ptr->toImageMsg();
                pubMedianImg.publish(*msg_ptr);

                cv_ptr->image = equalizedImg;
                msg_ptr = cv_ptr->toImageMsg();
                pubEqualizedImg.publish(*msg_ptr);

                cv_ptr->image = sharpenedImg;
                msg_ptr = cv_ptr->toImageMsg();
                pubSharpenedImg.publish(*msg_ptr);
                // ROS_INFO("--- image processing ---");
            }
            catch (std::exception &e){
                ROS_INFO("raw_image_process ERROR: %s", e.what());
            }

            imgBuf.pop();
        }
        // sleep(0.01);
    }
}

void raw_image_callback(const sensor_msgs::ImageConstPtr &img)
{
    std::lock_guard<std::mutex> _lock(imgLock);
    imgBuf.push(img);
}

#endif