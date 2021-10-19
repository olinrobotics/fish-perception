#ifndef CAMERA_DRIVERS_PUBLISH_CAMERA_DATA
#define CAMERA_DRIVERS_PUBLISH_CAMERA_DATA

#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <memory>
#include <string.h>

namespace sensor_drivers
{
namespace camera_driver
{

// using a class so that multiple camera drivers can be created, each with their own information
class CameraDriver
{
    public:
        // using a CameraInfo struct to keep all camera information tied closely and accessible
        struct CameraInfo 
        {
            std::string name;
            std::string frame_id;
            int capture_width;
            int capture_height;
            int display_width;
            int display_height;
            int framerate;
            int flip_method;
            cv::Mat img;
        };
        // constructor
        CameraDriver(std::string& name, int& capture_width, int& capture_height, 
                    int& display_width, int& display_height, int& framerate, int& flip_method);

    private:
        // create streamer
        std::string gstreamer_pipeline(CameraInfo& cam_info);
        // converts image to a ros msg format
        void convert_to_ros(cv::Mat& img, sensor_msgs::Image& msg, std_msgs::Header& header, cv_bridge::CvImage& img_bridge)
        // image publisher on loop
        void execute_publish();

        CameraInfo cam_info_;
        std::string streamer_;
        std::unique_ptr<cv::VideoCapture> cap_;

        cv_bridge::CvImage& img_bridge_;
        sensor_msgs::Image img_msg_;
        std_msgs::Header msg_header_;

        ros::NodeHandle nh_;
        ros::Publisher img_pub_;
}

} // end namespace camera_driver
} // end namespace sensor_drivers
#endif // end CAMERA_DRIVERS_PUBLISH_CAMERA_DATA