#include <iostream>
#include <fstream>
#include <pwd.h>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Eigen>
#include <opencv2/highgui.hpp>

cv::Mat cameraMat;
cv::Mat distCoeff;
std::vector<cv::Point3f> clicked_lidar_points;
std::vector<cv::Point2f> clicked_image_points;
std::vector<double> re_error, R_error_degree, T_error;
double re_error_sum, re_error_vari, re_error_devi, R_error_sum, R_error_vari, R_error_devi, T_error_sum, T_error_vari, T_error_devi;
const double PI = std::acos(-1);

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
    return cv::Point3f(x, y, z);
}

void SaveCalibrationFile(cv::Mat in_intrinsic, cv::Mat in_dist_coeff, cv::Mat tran_sensor2optical, cv::Mat R_sensor2optical, double re_error, double R_error, double T_error)
{
    std::string path_filename_str;
    const char *homedir;

    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }
    path_filename_str = std::string(homedir) + "/lidar2cam-cali_ws/" + "RT_lidar_camera_calibration.yaml";

    cv::FileStorage fs(path_filename_str.c_str(), cv::FileStorage::WRITE);
    if(!fs.isOpened()){
        fprintf(stderr, "%s : cannot open file\n", path_filename_str.c_str());
        exit(EXIT_FAILURE);
    }

    cv::Mat R_optical2cam = (cv::Mat_<double>(3, 3) << 0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0);  // zyx rotation : [-pi/2 0 -pi/2]'

    fs << "CameraMat" << in_intrinsic;
    fs << "DistCoeff" << in_dist_coeff;
    fs << "DistModel" << "plumb_bob";
    fs << "T_Sensor2Cam" << tran_sensor2optical;
    fs << "R_Sensor2Cam" << get_ypr_from_matrix(R_sensor2optical * R_optical2cam);
    fs << "ReprojectionError" << re_error;
    fs << "RotationError" << R_error;
    fs << "TranslationError" << T_error;

    ROS_INFO("Wrote Autoware Calibration file in: %s", path_filename_str.c_str());
}

bool parse_params(ros::NodeHandle _nh)
{
    std::vector<float> vector_camMat, vector_distCoeff, vector_lidar_pts, vector_img_pts;
    int lidar_points_size, img_points_size;

    if(!_nh.getParam("/CameraMat/data", vector_camMat))
    {
        ROS_INFO("No parameter : cameraMat");
        return true;
    }
    cameraMat = (cv::Mat_<float>(3, 3) << vector_camMat[0], vector_camMat[1], vector_camMat[2],
            vector_camMat[3], vector_camMat[4], vector_camMat[5],
            vector_camMat[6], vector_camMat[7], vector_camMat[8]);

    if(!_nh.getParam("/DistCoeff/data", vector_distCoeff))
    {
        ROS_INFO("No parameter : distCoeff");
        return true;
    }
    distCoeff = (cv::Mat_<float>(5, 1) << vector_distCoeff[0], vector_distCoeff[1], vector_distCoeff[2], vector_distCoeff[3], vector_distCoeff[4]);

    if(!_nh.getParam("/lidar_pts/data", vector_lidar_pts))
    {
        ROS_INFO("No parameter : lidar_pts");
        return true;
    }

    for(int i=0; i<vector_lidar_pts.size(); i+=3)
    {
        clicked_lidar_points.push_back(cv::Point3f(vector_lidar_pts[i],vector_lidar_pts[i+1],vector_lidar_pts[i+2]));
    }

    if(!_nh.getParam("/img_pts/data", vector_img_pts))
    {
        ROS_INFO("No parameter : img_pts");
        return true;
    }

    for(int i=0; i<vector_img_pts.size(); i+=2)
    {
        clicked_image_points.push_back(cv::Point2f(vector_img_pts[i],vector_img_pts[i+1]));
    }

    return false;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "extract_RT");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    // parsing parameters
    if(parse_params(nh)) return -1;
    double input_point_size = clicked_image_points.size();
    cv::Mat rotation_vector, translation_vector;
    //NO Ransac PnP
    cv::solvePnP(clicked_lidar_points,
                 clicked_image_points,
                 cameraMat,
                 distCoeff,
                 rotation_vector,
                 translation_vector,
                 false,
                 cv::SOLVEPNP_ITERATIVE
                 );

   /* cv::solvePnPRefineVVS(clicked_lidar_points,
                          clicked_image_points,
                          cameraMat,
                          distCoeff,
                          rotation_vector,
                          translation_vector,
                          cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 20, FLT_EPSILON),1);*/

    //Ransac PNP
   /* cv::Mat inliers;
    std::vector<cv::Point2f> inlier2d;
    std::vector<cv::Point3f> inlier3d;

    cv::solvePnPRansac(clicked_lidar_points,
                 clicked_image_points,
                 cameraMat,
                 distCoeff,
                 rotation_vector,
                 translation_vector,
                 true,
                 100,
                 8.f,
                 0.99,
                 inliers,
                 cv::SOLVEPNP_ITERATIVE);

    for (int i = 0; i < inliers.rows; i++)
    {
        int index = inliers.at<int>(i, 0);
        inlier3d.push_back(cv::Point3f(clicked_lidar_points[index].x,clicked_lidar_points[index].y,clicked_lidar_points[index].z));
        inlier2d.push_back(cv::Point2f(clicked_image_points[index].x,clicked_image_points[index].y));
    }

    cv::solvePnPRefineVVS(inlier3d,
                         inlier2d,
                         cameraMat,
                         distCoeff,
                         rotation_vector,
                         translation_vector,
                         cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 20, FLT_EPSILON),1);*/

    //re_error
    std::vector<cv::Point2f> reproject_img_pts;
    cv::projectPoints(clicked_lidar_points, rotation_vector, translation_vector, cameraMat, distCoeff, reproject_img_pts);

    for(int i = 0; i<input_point_size; i++)
    {
        //l2norm
        double result = 0;
        double xDiff = clicked_image_points[i].x - reproject_img_pts[i].x;
        double yDiff = clicked_image_points[i].y - reproject_img_pts[i].y;
        result = std::pow(xDiff,2) + std::pow(yDiff,2);
        result = std::sqrt(result);
        re_error_sum += result;
        re_error.push_back(result);
    }

    cv::Mat sensor_rotation_matrix;
    cv::Rodrigues(rotation_vector, sensor_rotation_matrix);

    cv::Mat R_optical2cam = (cv::Mat_<double>(3, 3) << 0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0);  // zyx rotation : [-pi/2 0 -pi/2]'
    cv::Mat sensor_rotation = sensor_rotation_matrix.t();
    cv::Point3f ypr = get_ypr_from_matrix(sensor_rotation * R_optical2cam);
    std::cout << "Sensor to Optical Rotation(z, y, x) : " << "[" << ypr.z<< "; " << ypr.y << "; " << ypr.x  << "]" << std::endl;

    cv::Mat sensor_translation_vector = -sensor_rotation * translation_vector;
    std::cout << "Sensor to Optical translation(x, y, z) : " << sensor_translation_vector << std::endl;

    //*******************************unproject undist********************************************************
    std::vector<cv::Point2f> points_undistorted;
    cv::undistortPoints(clicked_image_points, points_undistorted, cameraMat, distCoeff, cv::noArray(), cv::noArray());
    for(int i=0; i<points_undistorted.size(); i++)
    {
        cv::Mat cam3d_tmp = (cv::Mat_<double>(3, 1) << clicked_lidar_points[i].x,
                         clicked_lidar_points[i].y,
                         clicked_lidar_points[i].z);
        cam3d_tmp = sensor_rotation_matrix * cam3d_tmp + translation_vector;     

        cv::Mat cam3d = (cv::Mat_<double>(3, 1) << points_undistorted[i].x * cam3d_tmp.at<double>(2.0),
                         points_undistorted[i].y * cam3d_tmp.at<double>(2.0),
                         cam3d_tmp.at<double>(2.0));
        cv::Mat lidar3d = sensor_rotation_matrix.t() * (cam3d - translation_vector);

        //R error
        double inner_product = clicked_lidar_points[i].x * lidar3d.at<double>(0,0) +
                               clicked_lidar_points[i].y * lidar3d.at<double>(1,0) +
                               clicked_lidar_points[i].z * lidar3d.at<double>(2,0);
        double l2norm_clicked3d = std::pow(clicked_lidar_points[i].x, 2) + std::pow(clicked_lidar_points[i].y, 2) + std::pow(clicked_lidar_points[i].z, 2);
        l2norm_clicked3d = std::sqrt(l2norm_clicked3d);
        double l2norm_re_points3d = std::pow(lidar3d.at<double>(0,0), 2) + std::pow(lidar3d.at<double>(1,0), 2) + std::pow(lidar3d.at<double>(2,0), 2);
        l2norm_re_points3d = std::sqrt(l2norm_re_points3d);
        double R_error_radian = acos(inner_product/(l2norm_clicked3d*l2norm_re_points3d));
        double R_error_degree_tmp = R_error_radian * 180 / PI;
        R_error_degree.push_back(R_error_degree_tmp);
        R_error_sum = R_error_sum + R_error_degree_tmp;

        //T error
        double result = 0;
        double xdiff = clicked_lidar_points[i].x - lidar3d.at<double>(0,0);
        double ydiff = clicked_lidar_points[i].y - lidar3d.at<double>(1,0);
        double zdiff = clicked_lidar_points[i].z - lidar3d.at<double>(2,0);
        result = std::pow(xdiff, 2) + std::pow(ydiff, 2) + std::pow(zdiff, 2);
        result = std::sqrt(result);

        T_error_sum += result;
        T_error.push_back(result);

    }

    //error_avg_print
    double re_error_avg = re_error_sum/input_point_size;
    double R_error_avg = R_error_sum/input_point_size;
    double T_error_avg = T_error_sum/input_point_size;
    std::cout<<"re_error_avg: " <<re_error_avg << std::endl;
    std::cout<<"R_error_avg: " <<R_error_avg <<std::endl;
    std::cout<<"T_error_avg: " <<T_error_avg << std::endl;

    //error standard deviation
    for(int i =0; i<input_point_size; i++)
    {
        re_error_vari += std::pow(re_error[i] - re_error_avg, 2);
        R_error_vari += std::pow(R_error_degree[i] - R_error_avg, 2);
        T_error_vari += std::pow(T_error[i] - T_error_avg, 2);
    }
    re_error_vari = re_error_vari / input_point_size;
    re_error_devi = sqrt(re_error_vari);
    R_error_vari = R_error_vari / input_point_size;
    R_error_devi = sqrt(R_error_vari);
    T_error_vari = T_error_vari / input_point_size;
    T_error_devi = sqrt(T_error_vari);
    std::cout<<"re_error_devi: " <<re_error_devi <<std::endl;
    std::cout<<"R_error_devi: " <<R_error_devi <<std::endl;
    std::cout<<"T_error_devi: " <<T_error_devi <<std::endl;

    SaveCalibrationFile(cameraMat, distCoeff, sensor_translation_vector, sensor_rotation, re_error_avg, R_error_avg, T_error_avg);

    return 0;
}

