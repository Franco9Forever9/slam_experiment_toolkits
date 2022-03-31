#include "../include/image_filtering/denoise_filtering.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "denoise_filtering");
    ros::NodeHandle nh;
    subRawImg = nh.subscribe<sensor_msgs::Image>("thermal_image_raw", 1000, raw_image_callback);
    
    ros::spin();
    return 0;
}