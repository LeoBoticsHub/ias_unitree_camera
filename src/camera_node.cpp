#include <UnitreeCameraSDK.hpp>
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <string>

// dictionary containing camera names and corresponding indexes
std::map<std::string, int> _camera_dict{{"face", 1}, {"chin", 0}, {"right", 0}, {"left", 1}, {"rearDown", 0}};

// @param left_camera: true - right camera info params, false - left camera info params
sensor_msgs::CameraInfo get_camera_info(bool right_camera)
{
    std::vector<cv::Mat> paramsArray;
    bool params_received;
    sensor_msgs::CameraInfo cam_info;

    do
    {
        params_received = cam.getCalibParams(paramsArray, left_camera);
        ROS_WARN_THROTTLE(3, "Waiting for left_camera_info messages.");
    } while (!params_received);
    ROS_INFO("camera_info messages received.");

    auto intr = paramsArray[0];
    // auto dist = paramsArray[1];
    cam_info.K[0] = intr.at<double>(0, 0);
    cam_info.K[1] = intr.at<double>(0, 1);
    cam_info.K[2] = intr.at<double>(0, 2);
    cam_info.K[3] = intr.at<double>(1, 0);
    cam_info.K[4] = intr.at<double>(1, 1);
    cam_info.K[5] = intr.at<double>(1, 2);
    cam_info.K[6] = intr.at<double>(2, 0);
    cam_info.K[7] = intr.at<double>(2, 1);
    cam_info.K[8] = intr.at<double>(2, 2);

    return cam_info;
}

int main(int argc, char *argv[])
{
    // initializing camera_name trough argument
    std::string camera_name;
    if (argc > 2)
    {
        camera_name = std::string(argv[1]);
    }
    else
    {
        ROS_ERROR("You should launch the cameras passing the camera name. i.e., roslaunch ias_unitree_camera unitree_camera.launch camera_name:=current_camera_name, where current_camera_name must be [head/chin/left/right/rearDown].");
        exit(1);
    }

    // node init
    std::string camera_node_name{"camera_" + camera_name + "_node"};
    ros::init(argc, argv, camera_node_name);
    ros::NodeHandle n("~");

    // init UnitreeCamera object by camera index
    int camera_idx;
    if (_camera_dict.find(camera_name) != _camera_dict.end()) 
    {
        camera_idx = _camera_dict[camera_name];
    } 
    else 
    {
        ROS_ERROR("camera_name [%s] does not exists. Choose between [head/chin/left/right/rearDown].", camera_name.c_str());
        exit(1);
    }

    ROS_WARN("Connecting to camera");
    UnitreeCamera cam(camera_idx);
    ROS_WARN("Camera connected");

    // check if the camera has been started correctly
    if(!cam.isOpened())
    {
        ROS_ERROR("Camera has not been opened properly. Look if some connections to the camera are running.");
        exit(EXIT_FAILURE);
    }

    bool publish_rect_rgb, publish_raw_rgb, publish_depth;
    bool publish_pointcloud, publish_camera_info;
    double ros_rate;

    n.getParam("publish_rect_rgb", publish_rect_rgb);
    n.getParam("publish_raw_rgb", publish_raw_rgb);
    n.getParam("publish_depth", publish_depth);
    n.getParam("publish_camera_info", publish_camera_info);
    n.getParam("publish_pointcloud", publish_pointcloud);
    n.getParam("ros_rate", ros_rate);

    int image_width, image_height, fps;

    n.getParam("image_width", image_width);
    n.getParam("image_height", image_height);
    n.getParam("fps", fps);

    // set frame size (default: 1856, 800)
    cv::Size frameSize(image_width, image_height); 
    
    // -----------------------------------------------------
    // -   Camera variables and publisher initialization   -
    // -----------------------------------------------------
    std_msgs::Header image_header;
    image_header.frame_id = "camera_optical_" + camera_name;

    // --------------Rectified images--------------
    cv::Mat temp_left_rect_image, temp_right_rect_image, left_rect_image, right_rect_image, feim_rect_image;
    std::chrono::microseconds t_rect;
    sensor_msgs::ImagePtr left_rect_image_msg, right_rect_image_msg;
    ros::Publisher left_rect_image_pub, right_rect_image_pub;
    cam.setRectFrameSize(cv::Size(frameSize.width >> 2, frameSize.height >> 1));
    if (publish_rect_rgb)
    {
        left_rect_image_pub = n.advertise<sensor_msgs::Image>("left_rect_image" , 10);
        right_rect_image_pub = n.advertise<sensor_msgs::Image>("right_rect_image" , 10);
    }

    // --------------Raw images--------------
    cv::Mat temp_left_raw_image, temp_right_raw_image, left_raw_image, right_raw_image, feim_raw_image;
    sensor_msgs::ImagePtr left_raw_image_msg, right_raw_image_msg;
    std::chrono::microseconds t_raw;
    ros::Publisher left_raw_image_pub, right_raw_image_pub;
    // set raw image sizes and fps
    // cam.setRawFrameSize(frameSize); ///< set camera frame size
    cam.setRawFrameRate(fps);       ///< set camera camera fps
    if (publish_raw_rgb)
    {
        left_raw_image_pub = n.advertise<sensor_msgs::Image>("left_raw_image", 10);
        right_raw_image_pub = n.advertise<sensor_msgs::Image>("right_raw_image" , 10);
    }

    // --------------Depth Image--------------
    cv::Mat temp_depth_image, depth_image;
    sensor_msgs::ImagePtr depth_image_msg;
    ros::Publisher depth_image_pub;
    std::chrono::microseconds t_depth;
    if (publish_depth)
    {
        depth_image_pub = n.advertise<sensor_msgs::Image>("depth_image" , 10);  
    }

    // --------------camera info--------------
    ros::Publisher left_camera_info_pub, right_camera_info_pub;
    sensor_msgs::CameraInfo left_cam_info, right_cam_info;
    if (publish_camera_info)
    {

        left_cam_info = get_camera_info(false);
        right_cam_info = get_camera_info(true);

        right_camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("right_cam_info", 10);
        left_camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("left_camera_info", 10);
    }

    // --------------Pointcloud--------------
    sensor_msgs::PointCloud2 pcd_msg;
    std::vector<PCLType> pcl_vec;
    ros::Publisher pcd_pub;
    std::chrono::microseconds t_pcd;
    std::vector<PCLType>::iterator pointcloud_iter;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlier_removal;
    outlier_removal.setMeanK(30);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointXYZRGB point;
    if (publish_pointcloud)
    {
        pcd_pub = n.advertise<sensor_msgs::PointCloud2>("pointcloud", 10);
    }

    // ------------------------
    // - start camera capture -
    // ------------------------

    cam.startCapture(); // disable image h264 encoding and share memory sharing
    cam.startStereoCompute(); // start disparity computing

    int seq;
    ros::Rate loop_rate(ros_rate);

    // -------------------------
    // -  while loop starting  -
    // -------------------------
    while(ros::ok() && cam.isOpened()){
        image_header.stamp = ros::Time::now();
        image_header.seq = seq;
        
        // -------------Rectified image-------------
        if (publish_rect_rgb)
        {
            if(cam.getRectStereoFrame(temp_left_rect_image, temp_right_rect_image, feim_rect_image))
            {
                
                cv::flip(temp_left_rect_image, left_rect_image, 0);
                left_rect_image_msg = cv_bridge::CvImage(image_header, "bgr8", left_rect_image).toImageMsg();
                left_rect_image_pub.publish(left_rect_image_msg);
                 
                
                cv::flip(temp_right_rect_image, right_rect_image, 0);
                right_rect_image_msg = cv_bridge::CvImage(image_header, "bgr8", right_rect_image).toImageMsg();
                right_rect_image_pub.publish(right_rect_image_msg);
            }
            else
            {
                ROS_ERROR("Rect camera image not retrieved at seq: %d", seq);
            }
        }

        // -------------Raw image-------------
        if (publish_raw_rgb)
        {
            if(cam.getStereoFrame(temp_left_raw_image, temp_right_raw_image, t_rect))
            {
                cv::flip(temp_left_raw_image, left_raw_image, 0);
                left_raw_image_msg = cv_bridge::CvImage(image_header, "bgr8", left_raw_image).toImageMsg();
                left_raw_image_pub.publish(left_raw_image_msg);

                cv::flip(temp_right_raw_image, right_raw_image, 0);
                right_raw_image_msg = cv_bridge::CvImage(image_header, "bgr8", right_raw_image).toImageMsg();
                right_raw_image_pub.publish(right_raw_image_msg);
            }
            else
            {
                ROS_ERROR("Raw camera image not retrieved at seq: %d", seq);
            }
        }

        // -------------Depth image-------------
        if (publish_depth)
        {
            if(cam.getDepthFrame(temp_depth_image, false, t_depth))
            {
                cv::flip(temp_depth_image, depth_image, 0);
                depth_image_msg = cv_bridge::CvImage(image_header, "mono16", depth_image).toImageMsg();
                depth_image_pub.publish(depth_image_msg);
            }
            else
            {
                ROS_ERROR("Depth image not retrieved at seq: %d", seq);
            }
        }

        // -------------Camera Info parameters-------------
        if (publish_camera_info)
        {
            left_camera_info_pub.publish(left_cam_info);
            right_camera_info_pub.publish(right_cam_info);
        }

        // -------------Camera pointcloud-------------
        if (publish_pointcloud)
        {
            // empty pcl_vector
            pcl_vec.clear();
            // get rgb colored poincloud
            if(cam.getPointCloud(pcl_vec, t_pcd)){
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
                for(pointcloud_iter = pcl_vec.begin(); pointcloud_iter != pcl_vec.end(); pointcloud_iter++)
                {
                    // set point
                    point.x = (*pointcloud_iter).pts(0);
                    point.y = (*pointcloud_iter).pts(1);
                    point.z = (*pointcloud_iter).pts(2);
                    // set color
                    point.r = (*pointcloud_iter).clr(2);
                    point.g = (*pointcloud_iter).clr(1);
                    point.b = (*pointcloud_iter).clr(0);

                    cloud_ptr->points.push_back(point);
                }

                // statistical outlier removal filtering
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

                outlier_removal.setInputCloud(cloud_ptr);
                outlier_removal.filter(*cloud_filtered);

                pcd_msg.header = image_header;
                pcl::toROSMsg(*cloud_ptr, pcd_msg);
            }
            pcd_pub.publish(pcd_msg);
        }
        // ------------------------------------------

        seq++;

        loop_rate.sleep();
        ros::spinOnce();
    }

    // -----------------------
    // - stop camera capture -
    // -----------------------
    cam.stopStereoCompute();  // stop disparity computing 
    cam.stopCapture(); // stop camera capturing

    return 0;
}
