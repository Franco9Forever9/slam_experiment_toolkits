#include "../include/image_filtering/denoise_filtering.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "denoise_filtering");
    ros::NodeHandle nh;

    subRawImg = nh.subscribe<sensor_msgs::Image>("/thermal_image_raw", 100, raw_image_callback);
    pubMedianImg = nh.advertise<sensor_msgs::Image>("/thermal/median",100);
    pubEqualizedImg = nh.advertise<sensor_msgs::Image>("/thermal/equalized",100);
    pubSharpenedImg = nh.advertise<sensor_msgs::Image>("/thermal/laplace",100);

    std::thread image_process(raw_image_process);
    ros::spin();
    return 0;
}