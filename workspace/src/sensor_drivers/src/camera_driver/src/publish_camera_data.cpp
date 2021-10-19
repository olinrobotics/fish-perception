
#include <ros/ros.h>
#include "publish_camera_data.h"

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <memory>
#include <string.h>


namespace sensor_drivers
{
namespace camera_driver
{

class CameraDriver
{
    public:
        CameraDriver(std::string& name, int& capture_width, int& capture_height, int& display_width, 
                    int& display_height, int& framerate, int& flip_method)
        {
            // initialize camera info
            cam_info_.name = name;
            cam_info_.capture_width = capture_width;
            cam_info_.capture_height = capture_height;
            cam_info_.display_width = display_width;
            cam_info_.display_height = display_height;
            cam_info_.framerate = framerate;
            cam_info_.flip_method = flip_method;

            streamer_ = gstreamer_pipeline(cam_info_);
            cap_ = std::make_unique<cv::VideoCapture>(streamer_, cv::CAP_GSTREAMER);

            if(!cap.isOpened()) 
            {
                ROS_ERROR("Failed to open image stream");
            }
            ROS_INFO_STREAM("Opened camera, reading frames");

            // setup header frame id
            header_.frame_id = name;

            // TODO if needed: add publisher for Camera Info
            auto IMAGE_TOPIC = "/" + name + "/image";
            image_pub_ = nh_.advertise(IMAGE_TOPIC, 5);
        }

    private:
        // create streamer with camera parameters to load images from Jetson
        std::string gstreamer_pipeline(CameraInfo& cam_info) 
        {
            return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(cam_info.capture_width) + ", height=(int)" +
            std::to_string(cam_info.capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(cam_info.framerate) +
            "/1 ! nvvidconv flip-method=" + std::to_string(cam_info.flip_method) + " ! video/x-raw, width=(int)" + std::to_string(cam_info.display_width) + ", height=(int)" +
            std::to_string(cam_info.display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
        }

        // convert the image to a ros msg
        void convert_to_ros(cv::Mat& img, sensor_msgs::Image& msg, std_msgs::Header& header, cv_bridge::CvImage& img_bridge)
        {
            // header.seq = counter; // user defined counter
            header.stamp = ros::Time::now(); // time
            img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
            try
            {
                img_bridge.toImageMsg(msg); // from cv_bridge to sensor_msgs::Image
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
        }

        void execute_publish()
        {
            while(nh_.ok())
            {
                // read the image
                if (!cap.read(cam_info_.img)) 
                {
                    ROS_ERROR("couldn't read image");
	            }
                // convert to image msg
                convert_to_ros(cam_info_.img, img_msg_, msg_header_, img_bridge_));

                // publish image
                img_pub_.publish(img_msg_);

                ros::spinOnce();
            }
            cap_.release();
        }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_image_node");
  auto driver = CameraDriver("PICAM1", 1280, 720, 1280, 720, 60, 0);
  driver.execute_publish();
  return 0;
}
} // end namespace camera_driver
} // end namespace sensor_drivers