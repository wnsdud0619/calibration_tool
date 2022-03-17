#include <iostream>
#include <fstream>
#include <pwd.h>
#include <string>

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

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>

int img_wdith, img_height;
double tunning_scale, display_scale, display_resize;
bool display_mode;

static cv::Mat cameraMat;
static cv::Mat distCoeff;
cv::Size ImageSize;
static cv::Mat T_Sensor2Cam;
cv::Point3f R_Sensor2Cam;

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
              
void SaveCalibrationFile(double x, double y, double z, double roll, double pitch, double yaw)
{
    std::string path_filename_str;
    const char *homedir;

    cv::Point3f sensor2cam_T;
    cv::Point3f sensor2cam_R;

    sensor2cam_T = cv::Point3f(x, y, z);
    sensor2cam_R = cv::Point3f(roll, pitch, yaw);

    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }
    path_filename_str = std::string(homedir) + "/calibration_tool/" + "RT_revise.yaml";

    cv::FileStorage fs(path_filename_str.c_str(), cv::FileStorage::WRITE);
    if(!fs.isOpened()){
        fprintf(stderr, "%s : cannot open file\n", path_filename_str.c_str());
        exit(EXIT_FAILURE);
    }

    fs << "sensor2cam_T" << sensor2cam_T;
    fs << "sensor2cam_R" << sensor2cam_R;

    ROS_INFO("Wrote Autoware Calibration file in: %s", path_filename_str.c_str());
}

bool parse_params(ros::NodeHandle _nh)
{

    char __APP_NAME__[] = "calibration_check";

    std::string calibration_file;
    _nh.param<std::string>("calibration_file", calibration_file, "");
    ROS_INFO("calibration_file: '%s'", calibration_file.c_str());

    if (calibration_file.empty())
    {
      ROS_ERROR("[%s] Missing calibration file path '%s'.", __APP_NAME__, calibration_file.c_str());
      ros::shutdown();
      return -1;
    }

    cv::FileStorage fs(calibration_file, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
      ROS_ERROR("[%s] Cannot open file calibration file '%s'", __APP_NAME__, calibration_file.c_str());
      ros::shutdown();
      return -1;
    }

    fs["CameraMat"] >> cameraMat;
    fs["DistCoeff"] >> distCoeff;
    fs["ImageSize"] >> ImageSize;
    fs["T_Sensor2Cam"] >> T_Sensor2Cam;
    fs["R_Sensor2Cam"] >> R_Sensor2Cam;

    img_height = ImageSize.height;
    img_wdith = ImageSize.width;
    image = cv::Mat(img_height, img_wdith, CV_8UC3);
    pts_img = cv::Mat(img_height, img_wdith, CV_8UC3);

    sensor2cam_x = T_Sensor2Cam.at<double>(0,0);
    sensor2cam_y = T_Sensor2Cam.at<double>(1,0);
    sensor2cam_z = T_Sensor2Cam.at<double>(2,0);
    sensor2cam_roll = R_Sensor2Cam.x;
    sensor2cam_pitch = R_Sensor2Cam.y;
    sensor2cam_yaw = R_Sensor2Cam.z;

    std::cout<<"sensor2cam_T: "<<sensor2cam_x<<", "<<sensor2cam_y<<", "<<sensor2cam_z<<std::endl;
    std::cout<<"sensor2cam_R: "<<sensor2cam_roll<<", "<<sensor2cam_pitch<<", "<<sensor2cam_yaw<<std::endl;

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

    ros::Subscriber sub_img = nh.subscribe("/sensing/camera/traffic_light/image_raw", 1, imageCallback);
    ros::Subscriber sub_points = nh.subscribe("/points_raw", 1, Callback_point_cloud);

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tfListener(tf_buffer_);
    geometry_msgs::TransformStamped tfs;
    try 
    {  
      tfs = tf_buffer_.lookupTransform("sensor_kit_base_link", "base_link", ros::Time(0), ros::Duration(1.0));
    }
    catch (tf2::TransformException & ex) 
    {
      ROS_WARN_THROTTLE(5, "cannot get transform");
    }

    tf2::Vector3 lidar2sensor_vec(tfs.transform.translation.x, tfs.transform.translation.y, tfs.transform.translation.z);
    tf2::Matrix3x3 lidar2sensor_rot(tf2::Quaternion(tfs.transform.rotation.x, tfs.transform.rotation.y, tfs.transform.rotation.z,tfs.transform.rotation.w));
    lidar2sensor_rot.getRPY(sensor2vel_top_yaw, sensor2vel_top_pitch, sensor2vel_top_roll);
    std::cout<<"sensor2vel_top_R: "<<sensor2vel_top_roll<<", "<<sensor2vel_top_pitch<<", "<<sensor2vel_top_yaw<<std::endl;

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
            case 32 : //space
            std::cout << "roll: " << sensor2cam_roll << std::endl;
            std::cout << "pitch: " << sensor2cam_pitch << std::endl;
            std::cout << "yaw: " << sensor2cam_yaw << std::endl;
            SaveCalibrationFile(sensor2cam_x, sensor2cam_y, sensor2cam_z, sensor2cam_roll, sensor2cam_pitch, sensor2cam_yaw);
            break;
        }
    }
    return 0;
}