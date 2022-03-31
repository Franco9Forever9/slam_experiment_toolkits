#ifndef _DENOISE_FILTERING_H_
#define _DENOISE_FILTERING_H_

#include <queue>
#include <vector>
#include <mutex>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

std::mutex imgLock;
ros::Subscriber subRawImg;
ros::Publisher pubMedianImg;
std::queue<sensor_msgs::ImageConstPtr> imgBuf;

// 只对灰度图像使用
cv::Mat median_filtering(const cv::Mat &src, int size)
{
    cv::Mat dst = src.clone();
    int edge = size / 2;
    
    for(int row = edge; row < dst.rows - edge; row++){
        for(int col = edge; col < dst.cols - edge; col++){
            std::vector<uchar> core;
            for(int m = row - edge; m <= m + edge; m++){
                for(int n = col - edge; n <= n + edge; n++){
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
            for(int m = row - edge; m <= m + edge; m++){
                for(int n = col - edge; n <= n + edge; n++){
                    sum += src.at<uchar>(m,n);
                }
            }

            dst.at<uchar>(row,col) = (int)(sum / size / size);
        }
    }
    return dst;
}

cv::Mat guassian_filtering(const cv::Mat &src, int size, double sigma)
{
    std::vector<std::vector<double>> guassianTemplete = guassian_template(size, sigma);
    cv::Mat dst = src.clone();
    int edge = size / 2;

    for (int row = edge; row < dst.rows - edge; row++){
        for (int col = edge; col < dst.cols - edge; col++){
            int sum = 0;
            for (int m = row - edge; m <= m + edge; m++){
                for (int n = col - edge; n <= n + edge; n++){
                    sum += src.at<uchar>(m, n) * guassianTemplete[m - row + edge][n - col + edge];
                }
            }

            dst.at<uchar>(row, col) = (uchar)sum;
        }
    }
}

std::vector<std::vector<double>> guassian_template(int size, double sigma)
{

}

cv::Mat convert_to_grayscale(cv::Mat &src)
{
    cv::Mat dst = src.clone();
    cv::cvtColor(src, dst, cv::COLOR_RGB2GRAY);
    return dst;
}

void histo_equalize(cv::Mat &src)
{
    cv::equalizeHist(src,src);
}



void raw_image_callback(const sensor_msgs::ImageConstPtr &img)
{
    std::lock_guard<std::mutex> _lock(imgLock);
    imgBuf.push(img);
}

#endif