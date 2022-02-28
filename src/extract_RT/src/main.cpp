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
double R_err_tmp = 0;
const double PI = std::acos(-1);

double inner_product(std::vector<cv::Point3f> a, std::vector<cv::Point3f> b){

    double result = 0;
    result= a.data()->x * b.data()->x + a.data()->y * b.data()->y + a.data()->z * b.data()->z;
    return result;
}

double l2norm(std::vector<cv::Point3f> a){

    double result = 0;
    result = a.data()->x * a.data()->x + a.data()->y * a.data()->y + a.data()->z * a.data()->z;
    result = sqrt(result);

    return result;
}

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

    cv::Mat rotation_vector, translation_vector;

    //NO Ransac PnP
    /*cv::solvePnP(clicked_lidar_points,
                 clicked_image_points,
                 cameraMat,
                 distCoeff,
                 rotation_vector,
                 translation_vector,
                 false,
                 cv::SOLVEPNP_ITERATIVE
                 );
    cv::solvePnPRefineVVS(clicked_lidar_points,
                          clicked_image_points,
                          cameraMat,
                          distCoeff,
                          rotation_vector,
                          translation_vector,
                          cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 20, FLT_EPSILON),1);*/

    //Ransac PNP
    cv::Mat inliers;
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
                 cv::SOLVEPNP_SQPNP);

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
                         cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 20, FLT_EPSILON),1);

    std::vector<cv::Point2f> reproject_img_pts;
    cv::projectPoints(clicked_lidar_points, rotation_vector, translation_vector, cameraMat, distCoeff, reproject_img_pts);
    double re_error = cv::norm(cv::Mat(clicked_image_points), cv::Mat(reproject_img_pts), cv::NORM_L2)/ reproject_img_pts.size();
    std::cout<<"re_error: "<<re_error<<std::endl;

    cv::Mat sensor_rotation_matrix;
    cv::Rodrigues(rotation_vector, sensor_rotation_matrix);

    cv::Mat R_optical2cam = (cv::Mat_<double>(3, 3) << 0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0);  // zyx rotation : [-pi/2 0 -pi/2]'
    cv::Mat sensor_rotation = sensor_rotation_matrix.t();
    cv::Point3f ypr = get_ypr_from_matrix(sensor_rotation * R_optical2cam);
    std::cout << "Sensor to Optical Rotation(z, y, x) : " << "[" << ypr.z<< "; " << ypr.y << "; " << ypr.x  << "]" << std::endl;

    cv::Mat sensor_translation_vector = -sensor_rotation * translation_vector;
    std::cout << "Sensor to Optical translation(x, y, z) : " << sensor_translation_vector << std::endl;


    //*******************************unproject #1 undist********************************************************
    std::vector<cv::Point2f> points_undistorted;
    cv::undistortPoints(clicked_image_points, points_undistorted, cameraMat, distCoeff, cv::noArray(), cameraMat);
    //std::cout <<points_undistorted<< std::endl;
    //std::cout<<"****************************"<<std::endl;

    std::vector<cv::Point3f> re_points_3d, clicked_3d_points;
    for(int i=0; i<points_undistorted.size(); i++)
    {
        cv::Mat cam3d = (cv::Mat_<double>(3, 1) << (points_undistorted[i].x - cameraMat.at<float>(0,2)) / cameraMat.at<float>(0,0) * clicked_lidar_points[i].x,
                         (points_undistorted[i].y - cameraMat.at<float>(1,2)) / cameraMat.at<float>(1,1) * clicked_lidar_points[i].x,
                         clicked_lidar_points[i].x);

        cam3d = cam3d - translation_vector;
        cam3d = sensor_rotation_matrix.t()*cam3d;

        //R error
        re_points_3d.push_back(cv::Point3f(cam3d.at<double>(0,0),cam3d.at<double>(1,0),cam3d.at<double>(2,0)));
        clicked_3d_points.push_back(cv::Point3f(clicked_lidar_points[i].x,clicked_lidar_points[i].y,clicked_lidar_points[i].z));
        auto radian = acos(inner_product(clicked_3d_points,re_points_3d)/(l2norm(clicked_3d_points)*l2norm(re_points_3d)));
        auto R_err = radian * 180 / PI;
        //std::cout << "degree_err"<< degree_err << std::endl;
        R_err_tmp = R_err_tmp + R_err;

    }

    //T error
    double T_error = cv::norm(cv::Mat(clicked_lidar_points), cv::Mat(re_points_3d), cv::NORM_L2)/ clicked_lidar_points.size();
    double R_error = R_err_tmp/points_undistorted.size();
    std::cout<<"T_error: " <<T_error << std::endl;
    std::cout<<"R_error: " <<R_error <<std::endl;

    SaveCalibrationFile(cameraMat, distCoeff, sensor_translation_vector, sensor_rotation, re_error, R_error, T_error);

    return 0;
}

