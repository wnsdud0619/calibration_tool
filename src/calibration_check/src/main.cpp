#include <iostream>
#include <fstream>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

int img_wdith, img_height;
double tunning_scale, display_scale, display_resize;
bool display_mode;

cv::Mat cameraMat;
cv::Mat distCoeff;

double sensor2vel_top_x;
double sensor2vel_top_y;
double sensor2vel_top_z;
double sensor2vel_top_roll;
double sensor2vel_top_pitch;
double sensor2vel_top_yaw;

double sensor2cam_x;
double sensor2cam_y;
double sensor2cam_z;
double sensor2cam_roll;
double sensor2cam_pitch;
double sensor2cam_yaw;

cv::Mat image;
cv::Mat pts_img;
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> point_cloud(new pcl::PointCloud<pcl::PointXYZI>);

cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
cv::Mat rvec = cv::Mat::eye(3, 3, CV_64FC1);
cv::Mat r_optical2sensor = cv::Mat::eye(3, 1, CV_64FC1);
cv::Mat t_sensor2optical = cv::Mat::eye(3, 3, CV_64FC1);

// Transform Camera to Optical [-pi/2, 0, -pi/2]
cv::Mat r_cam2optical = (cv::Mat_<double>(3, 3) << 0., 0., 1.,
                -1., 0., 0.,
                0., -1., 0.);

bool parse_params(ros::NodeHandle _nh)
{
    std::vector<double> vector_camMat, vector_distCoeff;
    if(!_nh.getParam("/tunning_scale", tunning_scale)) 
    {
        ROS_INFO("No parameter : tunning_scale");
        return true;
    }
    if(!_nh.getParam("/display_mode", display_mode))
    {
        ROS_INFO("No parameter : display_mode");
        return true;
    }
    if(!_nh.getParam("/display_scale", display_scale))
    {   
        ROS_INFO("No parameter : display_scale");
        return true;
    }
    if(!_nh.getParam("/display_resize", display_resize)) 
    {
        ROS_INFO("No parameter : display_resize");
        return true;
    }

    if(!_nh.getParam("/image_width", img_wdith)) 
    {
        ROS_INFO("No parameter : img_wdith");
        return true;
    }

    if(!_nh.getParam("/image_height", img_height)) 
    {
        ROS_INFO("No parameter : img_height");
        return true;
    }
    image = cv::Mat(img_height, img_wdith, CV_8UC3);
    pts_img = cv::Mat(img_height, img_wdith, CV_8UC3);

    if(!_nh.getParam("/camera_matrix/data", vector_camMat)) 
    {
        ROS_INFO("No parameter : cameraMat");
        return true;
    }
    cameraMat = (cv::Mat_<double>(3, 3) << vector_camMat[0], vector_camMat[1], vector_camMat[2], 
        vector_camMat[3], vector_camMat[4], vector_camMat[5], 
        vector_camMat[6], vector_camMat[7], vector_camMat[8]);

    if(!_nh.getParam("/distortion_coefficients/data", vector_distCoeff)) 
    {
        ROS_INFO("No parameter : distCoeff");
        return true;
    }
    distCoeff = (cv::Mat_<double>(5, 1) << vector_distCoeff[0], vector_distCoeff[1], vector_distCoeff[2], vector_distCoeff[3], vector_distCoeff[4]);

    if(!_nh.getParam("/sensor_kit_base_link2velodyne_top_base_link/x", sensor2vel_top_x))
    {
        ROS_INFO("No parameter : sensor_kit_base_link2velodyne_top_base_link");
        return true;
    }
    if(!_nh.getParam("/sensor_kit_base_link2velodyne_top_base_link/y", sensor2vel_top_y))
    {
        ROS_INFO("No parameter : sensor_kit_base_link2velodyne_top_base_link");
        return true;
    }
    if(!_nh.getParam("/sensor_kit_base_link2velodyne_top_base_link/z", sensor2vel_top_z))
    {
        ROS_INFO("No parameter : sensor_kit_base_link2velodyne_top_base_link");
        return true;
    }
    if(!_nh.getParam("/sensor_kit_base_link2velodyne_top_base_link/roll", sensor2vel_top_roll))
    {
        ROS_INFO("No parameter : sensor_kit_base_link2velodyne_top_base_link");
        return true;
    }
    if(!_nh.getParam("/sensor_kit_base_link2velodyne_top_base_link/pitch", sensor2vel_top_pitch))
    {
        ROS_INFO("No parameter : sensor_kit_base_link2velodyne_top_base_link");
        return true;
    }
    if(!_nh.getParam("/sensor_kit_base_link2velodyne_top_base_link/yaw", sensor2vel_top_yaw))
    {
        ROS_INFO("No parameter : sensor_kit_base_link2velodyne_top_base_link");
        return true;
    }

    if(!_nh.getParam("/sensor_kit_base_link2traffic_light_camera/x", sensor2cam_x))
    {
        ROS_INFO("No parameter : sensor_kit_base_link2traffic_light_camera");
        return true;
    }
    if(!_nh.getParam("/sensor_kit_base_link2traffic_light_camera/y", sensor2cam_y))
    {
        ROS_INFO("No parameter : sensor_kit_base_link2traffic_light_camera");
        return true;
    }
    if(!_nh.getParam("/sensor_kit_base_link2traffic_light_camera/z", sensor2cam_z))
    {
        ROS_INFO("No parameter : sensor_kit_base_link2traffic_light_camera");
        return true;
    }
    if(!_nh.getParam("/sensor_kit_base_link2traffic_light_camera/roll", sensor2cam_roll))
    {
        ROS_INFO("No parameter : sensor_kit_base_link2traffic_light_camera");
        return true;
    }
    if(!_nh.getParam("/sensor_kit_base_link2traffic_light_camera/pitch", sensor2cam_pitch))
    {
        ROS_INFO("No parameter : sensor_kit_base_link2traffic_light_camera");
        return true;
    }
    if(!_nh.getParam("/sensor_kit_base_link2traffic_light_camera/yaw", sensor2cam_yaw))
    {
        ROS_INFO("No parameter : sensor_kit_base_link2traffic_light_camera");
        return true;
    }
    return false;
}

void updateMatrix()
{
    Eigen::AngleAxisf yawAngle(sensor2cam_yaw, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(sensor2cam_pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rollAngle(sensor2cam_roll, Eigen::Vector3f::UnitX());

    // Validation :: Matlab eul2rotm
    Eigen::Quaternion<float> quatenion = yawAngle * pitchAngle * rollAngle;

    cv::Mat r_sensor2cam;
    cv::eigen2cv(quatenion.matrix(), r_sensor2cam);
    r_sensor2cam.convertTo(r_sensor2cam, CV_64F);

    // Translation for Lidar to Camera
    cv::Mat_<double> t_sensor2cam(3, 1); t_sensor2cam << sensor2cam_x, sensor2cam_y, sensor2cam_z;

    cv::Mat r_sensor2optical = r_sensor2cam * r_cam2optical;
    t_sensor2optical = r_cam2optical * t_sensor2cam;
    //std::cout << "r_sensor2optical = " << std::endl << r_sensor2optical << std::endl;
    //std::cout << "t_sensor2optical = " << std::endl  <<  t_sensor2optical << std::endl;

    r_optical2sensor = r_sensor2optical.t();
    tvec = -r_sensor2optical * t_sensor2optical;
    //std::cout << "r_optical2sensor = " << std::endl  <<  r_optical2sensor << std::endl;
    //std::cout << "t_optical2sensor = " << std::endl  <<  t_optical2sensor << std::endl;

    cv::Rodrigues(r_optical2sensor, rvec);
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        // ROS_INFO("sub_img");
        image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        pts_img = image.clone();
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert to image!");
    }
}

void Callback_point_cloud(const sensor_msgs::PointCloud2::ConstPtr &point_cloud_msg)
{
    // ROS_INFO("Sub_points");
    pcl::fromROSMsg(*point_cloud_msg, *point_cloud);
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "calibration_check_node");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    // parsing parameters
    if(parse_params(nh)) return -1;

    cv::namedWindow("view", CV_WINDOW_NORMAL);
    cv::resizeWindow("view", img_height * display_resize, img_wdith * display_resize);

//    ros::Subscriber sub_img = nh.subscribe("/sensing/camera/traffic_light/image_raw", 1, imageCallback);
    ros::Subscriber sub_img = nh.subscribe("/sensing/camera/traffic_light/image_raw", 1, imageCallback);
    ros::Subscriber sub_points = nh.subscribe("/points_raw", 1, Callback_point_cloud);

    Eigen::AngleAxisf yawAngle(sensor2vel_top_yaw, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(sensor2vel_top_pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rollAngle(sensor2vel_top_roll, Eigen::Vector3f::UnitX());
    Eigen::Quaternion<float> quatenion = yawAngle * pitchAngle * rollAngle;

    cv::Mat r_lidar2sensor;
    cv::eigen2cv(quatenion.matrix(), r_lidar2sensor);
    r_lidar2sensor.convertTo(r_lidar2sensor, CV_64F);

    while (ros::ok())
    {
        ros::spinOnce();
        updateMatrix();
        std::vector<cv::Point3f> sensor_pts;
        std::vector<double> distance;
        int point_cloud_size = point_cloud->size();
        for (int idx = 0; idx < point_cloud_size; idx++)
        {
            cv::Mat lidar_mat = (cv::Mat_<double>(3, 1) << point_cloud->points[idx].x,
                                        point_cloud->points[idx].y,
                                        point_cloud->points[idx].z);

            cv::Mat sensor_mat = r_lidar2sensor * lidar_mat;
            cv::Mat optical_mat = r_optical2sensor * lidar_mat + t_sensor2optical;
            if (optical_mat.at<double>(2) <= 2.) continue;
            distance.push_back(optical_mat.at<double>(2));
            sensor_pts.push_back(cv::Point3f(sensor_mat.at<double>(0), sensor_mat.at<double>(1), sensor_mat.at<double>(2)));
        }

        std::vector<cv::Point2f> projected_img_pts;
        if (!sensor_pts.empty()) cv::projectPoints(sensor_pts, rvec, tvec, cameraMat, distCoeff, projected_img_pts);

        for (int idx = 0; idx < projected_img_pts.size(); idx++)
        {
            if (projected_img_pts[idx].x < 0. || projected_img_pts[idx].y < 0.) continue;
            if (projected_img_pts[idx].x >= img_wdith || projected_img_pts[idx].y >= img_wdith) continue;
            
            cv::circle(pts_img, projected_img_pts[idx], 2, CV_RGB(255 - (int)(display_scale * distance[idx]), (int)(display_scale * distance[idx]), 0), -1);
        }
    
        if(display_mode)   cv::imshow("view", pts_img);
        else                cv::imshow("view", image);

        
        int chkey = cv::waitKey(1);
        if (chkey == 27) return -1;
        switch(chkey)
        {
            case 'q' : sensor2cam_yaw += tunning_scale;         break;
            case 'a' : sensor2cam_yaw -= tunning_scale;         break;
            case 'w' : sensor2cam_pitch += tunning_scale;       break;
            case 's' : sensor2cam_pitch -= tunning_scale;       break;
            case 'e' : sensor2cam_roll += tunning_scale;        break;
            case 'd' : sensor2cam_roll -= tunning_scale;        break;
            case 'o' : display_scale += 0.25;                   break;
            case 'l' : display_scale -= 0.25;                   break;
            case 'p' : display_mode = !display_mode;            break;
            case 32 :
            std::cout << "roll: " << sensor2cam_roll << std::endl;
            std::cout << "pitch: " << sensor2cam_pitch << std::endl;
            std::cout << "yaw: " << sensor2cam_yaw << std::endl;
            break;
        }
    }
    return 0;
}