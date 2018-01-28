#include <UnitreeCameraSDK.hpp>
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/Image>
// TODO include cvbridge

int main(int argc, char *argv[])
{

    // node init
    ros::init(argc, argv, "camera_node", ros::init_options::AnonymousName);
    ros::NodeHandle n("camera_node");

    // init UnitreeCamera object by camera index
    int camera_idx;
    n.getParam("camera_idx", camera_idx);
    UnitreeCamera cam(camera_idx);

    // check if the camera has been started correctly
    if(!cam.isOpened())
    {
        ROS_ERROR("Camera has not been opened properly. Look if some connections to the camera are running.")
        exit(EXIT_FAILURE);
    }

    bool publish_rect_rgb, publish_raw_rgb, publish_depth;
    bool publish_pointcloud, publish_camera_info;

    n.getParam("publish_rect_rgb", publish_rect_rgb);
    n.getParam("publish_raw_rgb", publish_raw_rgb);
    n.getParam("publish_depth", publish_depth);
    n.getParam("publish_pointcloud", publish_pointcloud);
    n.getParam("publish_camera_info", publish_camera_info);

    int image_width, image_height, fps,

    n.getParam("image_width", image_width);
    n.getParam("image_height", image_height);
    n.getParam("fps", fps);

    // set frame size (default: 1856, 800)
    cv::Size frameSize(image_width, image_height); 

    // TODO use publish bools to publish only needed values

    // set image sizes and fps
    cam.setRawFrameSize(frameSize); ///< set camera frame size
    cam.setRawFrameRate(fps);       ///< set camera camera fps
    cam.setRectFrameSize(cv::Size(frameSize.width >> 2, frameSize.height >> 1)); ///< set camera rectify frame size
    
    // start camera capture
    cam.startCapture(); ///< disable image h264 encoding and share memory sharing
    cam.startStereoCompute(); ///< start disparity computing

    // TODO creates publisher for all the values

    // TODO complete the publish logic
    while(ros::ok() && cam.isOpened()){

        cv::Mat left,right,feim;
        cv::Mat depth;
        std::chrono::microseconds t;
        if(!cam.getRectStereoFrame(left,right,feim)){ ///< get longlat rectify left,right and fisheye rectify feim  
            usleep(1000);
            continue;
        }
        if(!cam.getDepthFrame(depth, true, t)){  ///< get stereo camera depth image 
            usleep(1000);
            continue;
        }
        if(!depth.empty()){
            cv::imshow("UnitreeCamera-Depth", depth);
        }

        // get rgb colored poincloud
        std::vector<PCLType> pcl_vec;
        if(!cam.getPointCloud(pcl_vec, t)){
            usleep(1000);
            continue;
        }
        // get pointcloud without color
        // std::vector<cv::Vec3f> pcl_vec;
        // if(!cam.getPointCloud(pcl_vec, t)){
        //     usleep(1000);
        //     continue;
        // }
        
        cv::Mat stereo;
        cv::hconcat(left,right, stereo); 
        cv::imshow("FishEye_Rect", feim);
        cv::imshow("Longlat_Rect", stereo);

        ros::spinOnce();
    }

    cam.stopStereoCompute();  ///< stop disparity computing 
    cam.stopCapture(); ///< stop camera capturing

    return 0;
}