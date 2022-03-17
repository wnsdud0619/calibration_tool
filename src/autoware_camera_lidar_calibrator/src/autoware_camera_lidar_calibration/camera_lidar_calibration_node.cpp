#include <string>
#include <vector>
#include <ctime>
#include <sstream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <boost/filesystem.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#if (CV_MAJOR_VERSION == 3)
  #include <opencv2/imgcodecs.hpp>
#else
  #include <opencv2/contrib/contrib.hpp>
#endif

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>

#define __APP_NAME__ "autoware_camera_lidar_calibration_node"

class ROSCameraLidarApp

{
  bool isClickedImgPt;
  ros::NodeHandle   node_handle_;

  ros::Subscriber   subscriber_image_raw_;
  ros::Subscriber   subscriber_points_raw_;
  ros::Subscriber   subscriber_intrinsics_;
  ros::Subscriber   subscriber_clicked_point_;
  ros::Subscriber   subscriber_image_point_;

  cv::Size            image_size_;
  cv::Mat             camera_instrinsics_;
  cv::Mat             distortion_coefficients_;

  cv::Mat             lidar_camera_rotation_matrix_;
  cv::Mat             lidar_camera_translation_vector_;

  std::vector<cv::Point2f> clicked_image_points_;
  std::vector<cv::Point3f> clicked_velodyne_points_;
  std::vector<cv::Point3f> clicked_sensor_points_;
  std::vector<double> re_error;
  double re_error_sum;

  bool is_rotation_matrix(const cv::Mat& in_matrix)
  {
    cv::Mat rotation_matrix;
    transpose(in_matrix, rotation_matrix);

    cv::Mat test_matrix = rotation_matrix * in_matrix;
    cv::Mat test_result = cv::Mat::eye(3,3, test_matrix.type());

    return cv::norm(test_result, test_matrix) < 1e-6;

  }

  cv::Point3f get_ypr_from_matrix(const cv::Mat& in_rotation)
  {

    assert(is_rotation_matrix(in_rotation));

    float sy = sqrt(in_rotation.at<double>(0,0) * in_rotation.at<double>(0,0) +
                        in_rotation.at<double>(1,0) * in_rotation.at<double>(1,0) );

    bool is_singular = sy < 1e-6;

    float x, y, z;
    if (!is_singular)
    {
      x = atan2(in_rotation.at<double>(2,1), in_rotation.at<double>(2,2));
      y = atan2(-in_rotation.at<double>(2,0), sy);
      z = atan2(in_rotation.at<double>(1,0), in_rotation.at<double>(0,0));
    }
    else
    {
      x = atan2(-in_rotation.at<double>(1,2), in_rotation.at<double>(1,1));
      y = atan2(-in_rotation.at<double>(2,0), sy);
      z = 0;
    }
    return cv::Point3f(z, y, x);
  }

  void SaveCalibrationFile(cv::Mat in_extrinsic, cv::Mat in_intrinsic, cv::Mat in_dist_coeff, cv::Size in_size, cv::Mat tran_sensor2optical, cv::Mat R_sensor2optical, std::vector<cv::Point3f> lidar_pts, std::vector<cv::Point2f> img_pts, double error)
  {
    std::string path_filename_str;
    const char *homedir;

    if ((homedir = getenv("HOME")) == NULL) {
      homedir = getpwuid(getuid())->pw_dir;
    }
    path_filename_str = std::string(homedir) + "/calibration_tool/src/extract_RT/param/" + "lidar_camera_calibration.yaml";

    cv::FileStorage fs(path_filename_str.c_str(), cv::FileStorage::WRITE);
    if(!fs.isOpened()){
      fprintf(stderr, "%s : cannot open file\n", path_filename_str.c_str());
      exit(EXIT_FAILURE);
    }

    cv::Mat R_optical2cam = (cv::Mat_<double>(3, 3) << 0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0);  // zyx rotation : [-pi/2 0 -pi/2]'
  
    fs << "CameraExtrinsicMat" << in_extrinsic;
    fs << "CameraMat" << in_intrinsic;
    fs << "DistCoeff" << in_dist_coeff;
    fs << "DistModel" << "plumb_bob";
    fs << "ImageSize" << in_size;
    fs << "T_Sensor2Cam" << tran_sensor2optical;
    fs << "R_Sensor2Cam" << get_ypr_from_matrix(R_sensor2optical * R_optical2cam);
    fs << "T_Cam2Optical" << cv::Mat::zeros(3, 1, CV_64FC1);
    fs << "R_Cam2Optical" << get_ypr_from_matrix(R_optical2cam.t());
    fs << "img_pts" << cv::Mat(img_pts);
    fs << "lidar_pts" << cv::Mat(lidar_pts);
    fs << "ReprojectionError" << error;
	
    cv::Point3f ypr = get_ypr_from_matrix(R_sensor2optical * R_optical2cam);
    ROS_INFO("Wrote Autoware Calibration file in: %s", path_filename_str.c_str());
    
    std::cout << std::endl << "============================== Result ==============================" << std::endl;
    std::cout << "Sensor to Optical translation(x, y, z) : " << tran_sensor2optical << std::endl;
    std::cout << "Sensor to Optical Rotation(z, y, x) : " << "[" << RAD2DEG(ypr.z) << "; " << RAD2DEG(ypr.y) << "; " << RAD2DEG(ypr.x)  << "]" << std::endl;
    std::cout << "Reprojection Error(pixel) : " << error << std::endl;
    std::cout << "====================================================================" << std::endl;
  }

  void CalibrateSensors()
  {
    std::cout << "Number of points(Lidar pt. : Image pt.): " << clicked_velodyne_points_.size() << ":"  << clicked_image_points_.size() << std::endl;
    std::cout << "===============================================" << std::endl << std::endl;

    if (clicked_velodyne_points_.size() == clicked_image_points_.size()
        && clicked_image_points_.size() > 8)
    {
      /////////////////////////////////////////////////////////////////////
      ////////*                   Sensor_kit base                 *////////
      /////////////////////////////////////////////////////////////////////

      cv::Mat sensor_rotation_vector, sensor_translation_vector;

      cv::solvePnP(clicked_sensor_points_,
                   clicked_image_points_,
                   camera_instrinsics_,
                   distortion_coefficients_,
                   sensor_rotation_vector,
                   sensor_translation_vector,
                   false,
                   cv::SOLVEPNP_ITERATIVE
      );

      cv::Mat sensor_rotation_matrix;
      cv::Rodrigues(sensor_rotation_vector, sensor_rotation_matrix);

      cv::Mat sensor_camera_velodyne_rotation = sensor_rotation_matrix.t();
      cv::Point3f sensor_camera_velodyne_point = cv::Point3f(sensor_translation_vector);
      cv::Point3f sensor_camera_velodyne_translation;

      sensor_camera_velodyne_translation.x = -sensor_camera_velodyne_point.z;
      sensor_camera_velodyne_translation.y = sensor_camera_velodyne_point.x;
      sensor_camera_velodyne_translation.z = sensor_camera_velodyne_point.y;

      cv::Mat sensor_extrinsics = cv::Mat::eye(4,4, CV_64F);
      sensor_camera_velodyne_rotation.copyTo(sensor_extrinsics(cv::Rect_<float>(0,0,3,3)));
      sensor_extrinsics.at<double>(0,3) = sensor_camera_velodyne_translation.x;
      sensor_extrinsics.at<double>(1,3) = sensor_camera_velodyne_translation.y;
      sensor_extrinsics.at<double>(2,3) = sensor_camera_velodyne_translation.z;

      std::vector<cv::Point2f> sensor_reproject_img_pts;
      cv::projectPoints(clicked_sensor_points_, sensor_rotation_vector, sensor_translation_vector, camera_instrinsics_, distortion_coefficients_, sensor_reproject_img_pts);

      for(int i = 0; i<sensor_reproject_img_pts.size(); i++)
      {
        //l2norm
        double result = 0;
        double xDiff = clicked_image_points_[i].x - sensor_reproject_img_pts[i].x;
        double yDiff = clicked_image_points_[i].y - sensor_reproject_img_pts[i].y;
        result = std::pow(xDiff,2) + std::pow(yDiff,2);
        result = std::sqrt(result);
        re_error_sum += result;
        re_error.push_back(result);
      }
      double re_error_avg = re_error_sum/sensor_reproject_img_pts.size();	
      std::cout << "sensor err : " << re_error_avg << std::endl;
      std::cout << "if you want to save param and select next point, (y/n) : "<<std::endl;
     
      char key;
      std::cin >> key;

      if(key == 'y' || key == 'Y') // Y or y
      {
        //sensor_kit cali save
        std::cout << "\t\t\t\t\t\t[Sensor kit based]" << std::endl;
        SaveCalibrationFile(sensor_extrinsics, camera_instrinsics_, distortion_coefficients_, image_size_, sensor_translation_vector, sensor_camera_velodyne_rotation, clicked_sensor_points_, clicked_image_points_, re_error_avg);
    
      }
      else if(key == 'n' || key == 'N') // N or n
      {
        std::cout<<"pop points"<<std::endl;
        clicked_velodyne_points_.pop_back();
        clicked_sensor_points_.pop_back();
        clicked_image_points_.pop_back();
      }
      else
      {
        std::cout << "your key is : " << key << " error, " << std::endl;
        CalibrateSensors(); 
      }
    }
  }

  void RvizClickedPointCallback(const geometry_msgs::PointStamped& in_clicked_point)
  {
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tfListener(tf_buffer_);
    geometry_msgs::TransformStamped tfs;
    try 
    {  
      tfs = tf_buffer_.lookupTransform("base_link", "sensor_kit_base_link", ros::Time(0), ros::Duration(0.2));
    }
    catch (tf2::TransformException & ex) 
    {
      ROS_WARN_THROTTLE(5, "cannot get transform");
      return;
    }

    if(clicked_velodyne_points_.size() > 0 && !isClickedImgPt) 
    {
      clicked_velodyne_points_.pop_back();
      clicked_sensor_points_.pop_back();
    }

    tf2::Vector3 lidar2sensor_vec(tfs.transform.translation.x, tfs.transform.translation.y, tfs.transform.translation.z);
    tf2::Matrix3x3 lidar2sensor_rot(tf2::Quaternion(tfs.transform.rotation.x, tfs.transform.rotation.y, tfs.transform.rotation.z,tfs.transform.rotation.w));
    tf2::Vector3 sensor_point = lidar2sensor_rot.inverse() * tf2::Vector3(in_clicked_point.point.x, in_clicked_point.point.y, in_clicked_point.point.z) + lidar2sensor_vec;

    clicked_velodyne_points_.push_back(cv::Point3f(in_clicked_point.point.x,
                                                    in_clicked_point.point.y,
                                                    in_clicked_point.point.z));

    clicked_sensor_points_.push_back(cv::Point3f(sensor_point.x(), 
                                                    sensor_point.y(), 
                                                    sensor_point.z()));

    std::cout << "Lidar point = " << cv::Point3f(in_clicked_point.point.x,
                              in_clicked_point.point.y,
                              in_clicked_point.point.z) << std::endl;

    std::cout << "Sensor_kit point = " << cv::Point3f(sensor_point.x(), 
                              sensor_point.y(),
                              sensor_point.z()) << std::endl;
    CalibrateSensors();
    isClickedImgPt = false;
  }
  void ImageClickedPointCallback(const geometry_msgs::PointStamped& in_clicked_point)
  {
    if(clicked_image_points_.size() > 0 && isClickedImgPt) clicked_image_points_.pop_back();
    clicked_image_points_.push_back(cv::Point2f(in_clicked_point.point.x,
                                                   in_clicked_point.point.y));

    std::cout <<  "Image point = " << cv::Point2f(in_clicked_point.point.x,
                             in_clicked_point.point.y) << std::endl;
    CalibrateSensors();
    isClickedImgPt = true;
  }

  void ImageCallback(const sensor_msgs::Image& in_image_sensor)
  {

    if (camera_instrinsics_.empty() || distortion_coefficients_.empty())
    {
      ROS_INFO("[%s] No Camera intrinsics loaded. Make sure to publish and set correctly camera_info source topic.", __APP_NAME__);
      return;
    }
  }

  void IntrinsicsCallback(const sensor_msgs::CameraInfo& in_message)
  {
    if (camera_instrinsics_.empty() || distortion_coefficients_.empty())
    {
      image_size_.height = in_message.height;
      image_size_.width = in_message.width;

      camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
      for (int row = 0; row < 3; row++)
      {
        for (int col = 0; col < 3; col++)
        {
          camera_instrinsics_.at<double>(row, col) = in_message.K[row * 3 + col];
        }
      }

      distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
      for (int col = 0; col < 5; col++)
      {
        distortion_coefficients_.at<double>(col) = in_message.D[col];
      }
      ROS_INFO("[%s] Camera Intrinsics Loaded", __APP_NAME__);
    }
  }

public:
  void Run()
  {
    isClickedImgPt = false;
    ros::NodeHandle private_node_handle("~");//to receive args

    std::string image_raw_topic_str, points_raw_topic_str, clusters_topic_str, camera_info_topic_str;
    std::string name_space_str = ros::this_node::getNamespace();

    private_node_handle.param<std::string>("image_src", image_raw_topic_str, "/image_rectified");
    private_node_handle.param<std::string>("camera_info_src", camera_info_topic_str, "camera_info");

    if (name_space_str != "/")
    {
      if (name_space_str.substr(0, 2) == "//")
      {
        name_space_str.erase(name_space_str.begin());
      }
      image_raw_topic_str = name_space_str + image_raw_topic_str;
      camera_info_topic_str = name_space_str + camera_info_topic_str;
    }

    ROS_INFO("[%s] image_src: %s",__APP_NAME__, image_raw_topic_str.c_str());
    ROS_INFO("[%s] camera_info_src: %s",__APP_NAME__, camera_info_topic_str.c_str());


    ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, image_raw_topic_str.c_str());
    subscriber_image_raw_ = node_handle_.subscribe(image_raw_topic_str, 1, &ROSCameraLidarApp::ImageCallback, this);


    ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, camera_info_topic_str.c_str());
    subscriber_intrinsics_ = node_handle_.subscribe(camera_info_topic_str, 1, &ROSCameraLidarApp::IntrinsicsCallback, this);

    ROS_INFO("[%s] Subscribing to PointCloud ClickedPoint from RVIZ... /clicked_point",__APP_NAME__);
    subscriber_clicked_point_ = node_handle_.subscribe("/clicked_point", 1, &ROSCameraLidarApp::RvizClickedPointCallback, this);

    ROS_INFO("[%s] Subscribing to Image ClickedPoint from JSK ImageView2... %s/screenpoint",__APP_NAME__, image_raw_topic_str.c_str());
    subscriber_image_point_ = node_handle_.subscribe(image_raw_topic_str+"/screenpoint", 1, &ROSCameraLidarApp::ImageClickedPointCallback, this);
    ROS_INFO("[%s] ClickedPoint: %s",__APP_NAME__, (image_raw_topic_str+"/screenpoint").c_str());

    ROS_INFO("[%s] Ready. Waiting for data...",__APP_NAME__);
    ros::spin();
    ROS_INFO("[%s] END",__APP_NAME__);
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, __APP_NAME__);

  ROSCameraLidarApp app;

  app.Run();

  return 0;
}
