#include <UnitreeCameraSDK.hpp>
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>

int main(int argc, char *argv[])
{

    // node init
    ros::init(argc, argv, "camera_node", ros::init_options::AnonymousName);
    ros::NodeHandle n("camera_node");

    // init UnitreeCamera object by camera index
    int camera_idx;
    n.getParam("camera_idx", camera_idx);
    UnitreeCamera cam(camera_idx);

    // TODO add enum to choose between cameras names
    std::string camera_name;

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

   
    //-------------------PUBLISHERS--------------------s
    // TODO insert camera name in topics

    // Rectified images
    sensor_msgs::Image rect_image_msg;
    cv::Mat left_rect_image, right_rect_image, feim_rect_image;
    std::chrono::microseconds t_rect;
    ros::Publisher left_rect_image_pub, right_rect_image_pub;
    if (publish_rect_rgb)
    {
        cam.setRectFrameSize(cv::Size(frameSize.width >> 2, frameSize.height >> 1));
        left_rect_image_pub = n.advertise<sensor_msgs::Image>("left_rect_image_", 10);
        right_rect_image_pub = n.advertise<sensor_msgs::Image>("right_rect_image_", 10);
    }

    // Raw images
    sensor_msgs::Image raw_image_msg;
    cv::Mat left_raw_image, right_raw_image, feim_raw_image;
    std::chrono::microseconds t_raw;
    ros::Publisher left_raw_image_pub, right_raw_image_pub;
    if (publish_raw_rgb)
    {
        // set raw image sizes and fps
        cam.setRawFrameSize(frameSize); ///< set camera frame size
        cam.setRawFrameRate(fps);       ///< set camera camera fps
        left_raw_image_pub = n.advertise<sensor_msgs::Image>("raw_image_", 10);
        right_raw_image_pub = n.advertise<sensor_msgs::Image>("raw_image_", 10);
    }

    // Depth Image
    sensor_msgs::Image depth_image_msg;
    cv::Mat depth;
    ros::Publisher depth_image_pub;
    std::chrono::microseconds t_depth;
    if (publish_depth)
    {
        depth_image_pub = n.advertise<sensor_msgs::Image>("depth_image_", 10);  
    }

    // Pointcloud
    sensor_msgs::PointCloud2 pcd_msg;
    ros::Publisher pcd_pub;
    if (publish_pointcloud)
    {
        pcd_pub = n.advertise<sensor_msgs::Image>("pcd_", 10);
    }

    // camera info
    ros::Publisher camera_info_pub;
    if (publish_camera_info)
    {
        camera_info_pub = n.advertise<sensor_msgs::Image>("camera_info_", 10);
    }

    // cv_bridge
    cv_bridge::CvImagePtr cv_ptr;

    // start camera capture
    cam.startCapture(); ///< disable image h264 encoding and share memory sharing
    cam.startStereoCompute(); ///< start disparity computing

    int seq;
    // TODO complete the publish logic. remember timestamp
    while(ros::ok() && cam.isOpened()){
        
        // Publish rectified image
        if (publish_rect_rgb)
        {
            if(cam.getRectStereoFrame(left_rect_image, right_rect_image, feim_rect_image))
            {
                cv_ptr->encoding = "bgr8";

                cv_ptr->image = left_rect_image;
                left_rect_image_pub.publish(cv_ptr->toImageMsg());

                cv_ptr->image = right_rect_image;
                right_rect_image_pub.publish(cv_ptr->toImageMsg());
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
                cv_ptr->encoding = "bgr8";

                cv_ptr->image = left_raw_image;
                left_raw_image_pub.publish(cv_ptr->toImageMsg());

                cv_ptr->image = right_raw_image;
                right_raw_image_pub.publish(cv_ptr->toImageMsg());
            }
            else
            {
                ROS_ERROR("Raw camera image not retrieved at seq: %d", seq);
            }
        }

        // Publish depth image
        if (publish_depth)
        {
            if(cam.getDepthFrame(depth, false, t_depth))
            {
                cv_ptr->encoding = "mono16";

                cv_ptr->image = depth;
                depth_image_pub.publish(depth_image_msg);
            }
            else
            {
                ROS_ERROR("Depth image not retrieved at seq: %d", seq);
            }
        }

        // Publish camera pointcloud
        if (publish_pointcloud)
        {
            // // get rgb colored poincloud
            // std::vector<PCLType> pcl_vec;
            // if(!cam.getPointCloud(pcl_vec, t)){
            //     usleep(1000);
            //     continue;
            // }
        }

        // Publish camera prameters
        if (publish_camera_info)
        {

        }






        seq++;
        ros::spinOnce();
    }

    cam.stopStereoCompute();  ///< stop disparity computing 
    cam.stopCapture(); ///< stop camera capturing

    return 0;
}