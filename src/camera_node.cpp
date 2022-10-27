#include <UnitreeCameraSDK.hpp>
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>
#include <vector>

int main(int argc, char *argv[])
{
    //TODO init camera_name from argv[1]
    std::string camera_name;
    std::string camera_node_name{"camera_"+camera_name+"_node"};

    // node init
    ros::init(argc, argv, camera_node_name);
    ros::NodeHandle n("~");

    // init UnitreeCamera object by camera index
    int camera_idx;
    n.getParam("camera_idx", camera_idx);
    ROS_INFO("Connecting to camera");
    UnitreeCamera cam(camera_idx);
    ROS_INFO("Camera connected");

    // check if the camera has been started correctly
    if(!cam.isOpened())
    {
        ROS_ERROR("Camera has not been opened properly. Look if some connections to the camera are running.");
        exit(EXIT_FAILURE);
    }

    bool publish_rect_rgb, publish_raw_rgb, publish_depth;
    bool publish_pointcloud, publish_camera_info;

    n.getParam("publish_rect_rgb", publish_rect_rgb);
    n.getParam("publish_raw_rgb", publish_raw_rgb);
    n.getParam("publish_depth", publish_depth);
    n.getParam("publish_pointcloud", publish_pointcloud);
    n.getParam("publish_camera_info", publish_camera_info);

    int image_width, image_height, fps;

    n.getParam("image_width", image_width);
    n.getParam("image_height", image_height);
    n.getParam("fps", fps);

    // set frame size (default: 1856, 800)
    cv::Size frameSize(image_width, image_height); 
   
    //-------------------PUBLISHERS--------------------
    std_msgs::Header image_header;
    image_header.frame_id = "camera_optical_" + camera_name;

    // Rectified images
    cv::Mat left_rect_image, right_rect_image, feim_rect_image;
    std::chrono::microseconds t_rect;
    sensor_msgs::ImagePtr left_rect_image_msg, right_rect_image_msg;
    ros::Publisher left_rect_image_pub, right_rect_image_pub;
    if (publish_rect_rgb)
    {
        cam.setRectFrameSize(cv::Size(frameSize.width >> 2, frameSize.height >> 1));
        left_rect_image_pub = n.advertise<sensor_msgs::Image>("left_rect_image_" + camera_name, 10);
        right_rect_image_pub = n.advertise<sensor_msgs::Image>("right_rect_image_" + camera_name, 10);
    }

    // Raw images
    cv::Mat left_raw_image, right_raw_image, feim_raw_image;
    sensor_msgs::ImagePtr left_raw_image_msg, right_raw_image_msg;
    std::chrono::microseconds t_raw;
    ros::Publisher left_raw_image_pub, right_raw_image_pub;
    if (publish_raw_rgb)
    {
        // set raw image sizes and fps
        cam.setRawFrameSize(frameSize); ///< set camera frame size
        cam.setRawFrameRate(fps);       ///< set camera camera fps
        left_raw_image_pub = n.advertise<sensor_msgs::Image>("left_raw_image_" + camera_name, 10);
        right_raw_image_pub = n.advertise<sensor_msgs::Image>("right_raw_image_" + camera_name, 10);
    }

    // Depth Image
    cv::Mat depth_image;
    sensor_msgs::ImagePtr depth_image_msg;
    ros::Publisher depth_image_pub;
    std::chrono::microseconds t_depth;
    if (publish_depth)
    {
        depth_image_pub = n.advertise<sensor_msgs::Image>("depth_image_" + camera_name, 10);  
    }

    // camera info
    ros::Publisher camera_info_pub;
    std::vector<cv::Mat> paramsArray;
    sensor_msgs::CameraInfo cam_info;
    if (publish_camera_info)
    {
        camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("camera_info_" + camera_name, 10);
    }

    // Pointcloud
    sensor_msgs::PointCloud2 pcd_msg;
    std::vector<PCLType> pcl_vec;
    ros::Publisher pcd_pub;
    std::chrono::microseconds t_pcd;
    if (publish_pointcloud)
    {
        pcd_pub = n.advertise<sensor_msgs::PointCloud2>("pcd_" + camera_name, 10);
    }

    // start camera capture
    cam.startCapture(); ///< disable image h264 encoding and share memory sharing
    cam.startStereoCompute(); ///< start disparity computing

    int seq;
    // TODO complete the publish logic. remember timestamp
    while(ros::ok() && cam.isOpened()){
        image_header.stamp = ros::Time::now();
        image_header.seq = seq;
        
        // Publish rectified image
        if (publish_rect_rgb)
        {
            if(cam.getRectStereoFrame(left_rect_image, right_rect_image, feim_rect_image))
            {

                left_rect_image_msg = cv_bridge::CvImage(image_header, "bgr8", left_rect_image).toImageMsg();
                left_rect_image_pub.publish(left_rect_image_msg);

                right_rect_image_msg = cv_bridge::CvImage(image_header, "bgr8", right_rect_image).toImageMsg();
                right_rect_image_pub.publish(right_rect_image_msg);
            }
            else
            {
                ROS_ERROR("Rect camera image not retrieved at seq: %d", seq);
            }
        }

        // Publish raw image
        if (publish_raw_rgb)
        {
            if(cam.getStereoFrame(left_raw_image, right_raw_image, t_rect))
            {
                left_raw_image_msg = cv_bridge::CvImage(image_header, "bgr8", left_raw_image).toImageMsg();
                left_raw_image_pub.publish(left_raw_image_msg);

                right_raw_image_msg = cv_bridge::CvImage(image_header, "bgr8", right_raw_image).toImageMsg();
                right_raw_image_pub.publish(right_raw_image_msg);
            }
            else
            {
                ROS_ERROR("Raw camera image not retrieved at seq: %d", seq);
            }
        }

        // Publish depth image
        if (publish_depth)
        {
            if(cam.getDepthFrame(depth_image, false, t_depth))
            {
                depth_image_msg = cv_bridge::CvImage(image_header, "mono16", depth_image).toImageMsg();
                depth_image_pub.publish(depth_image_msg);
            }
            else
            {
                ROS_ERROR("Depth image not retrieved at seq: %d", seq);
            }
        }

        // Publish camera prameters
        if (publish_camera_info)
        {
            if(cam.getCalibParams(paramsArray)){
                auto intr = paramsArray[0];
                auto dist = paramsArray[1];
                cam_info.K[0] = intr.at<double>(0, 0);
                cam_info.K[1] = intr.at<double>(0, 1);
                cam_info.K[2] = intr.at<double>(0, 2);
                cam_info.K[3] = intr.at<double>(1, 0);
                cam_info.K[4] = intr.at<double>(1, 1);
                cam_info.K[5] = intr.at<double>(1, 2);
                cam_info.K[6] = intr.at<double>(2, 0);
                cam_info.K[7] = intr.at<double>(2, 1);
                cam_info.K[8] = intr.at<double>(2, 2);

                cam_info.header = image_header;
            }
            camera_info_pub.publish(cam_info);
        }

        // Publish camera pointcloud
        if (publish_pointcloud)
        {
            // get rgb colored poincloud
            if(cam.getPointCloud(pcl_vec, t_pcd)){
                
            }
            pcd_pub.publish(pcd_msg);
        }

        seq++;
        ros::spinOnce();
    }

    cam.stopStereoCompute();  ///< stop disparity computing 
    cam.stopCapture(); ///< stop camera capturing

    ROS_INFO("FINE");
    return 0;
}