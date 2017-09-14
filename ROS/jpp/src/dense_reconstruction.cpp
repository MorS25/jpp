#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <jpp/CamToRobotCalibParamsConfig.h>
#include <libconfig.h>
#include "jpp.h"
#include "popt_pp.h"

jpp::CamToRobotCalibParamsConfig config;
config_t cfg, *cf;
FileStorage calib_file;
int debug = 0;

image_transport::Publisher dmap_pub;
ros::Publisher pcl_pub;

Mat composeRotationCamToRobot(float x, float y, float z) {
  Mat X = Mat::eye(3, 3, CV_64FC1);
  Mat Y = Mat::eye(3, 3, CV_64FC1);
  Mat Z = Mat::eye(3, 3, CV_64FC1);
  
  X.at<double>(1,1) = cos(x);
  X.at<double>(1,2) = -sin(x);
  X.at<double>(2,1) = sin(x);
  X.at<double>(2,2) = cos(x);

  Y.at<double>(0,0) = cos(y);
  Y.at<double>(0,2) = sin(y);
  Y.at<double>(2,0) = -sin(y);
  Y.at<double>(2,2) = cos(y);

  Z.at<double>(0,0) = cos(z);
  Z.at<double>(0,1) = -sin(z);
  Z.at<double>(1,0) = sin(z);
  Z.at<double>(1,1) = cos(z);
  
  return Z*Y*X;
}

Mat composeTranslationCamToRobot(float x, float y, float z) {
  return (Mat_<double>(3,1) << x, y, z);
}

void publishPointCloud(Mat& img_left, Mat& dmap, Mat& Q) {
  Mat XR, XT;
  calib_file["XR"] >> XR;
  calib_file["XT"] >> XT;
  if (debug == 1) {
    XR = composeRotationCamToRobot(config.PHI_X,config.PHI_Y,config.PHI_Z);
    XT = composeTranslationCamToRobot(config.TRANS_X,config.TRANS_Y,config.TRANS_Z);
    cout << "Rotation matrix: " << XR << endl;
    cout << "Translation matrix: " << XT << endl;
  }
  Mat V = Mat(4, 1, CV_64FC1);
  Mat pos = Mat(4, 1, CV_64FC1);
  vector< Point3d > points;
  sensor_msgs::PointCloud pc;
  sensor_msgs::ChannelFloat32 ch;
  ch.name = "rgb";
  pc.header.frame_id = "jackal";
  pc.header.stamp = ros::Time::now();
  for (int i = 0; i < img_left.cols; i++) {
    for (int j = 0; j < img_left.rows; j++) {
      int d = dmap.at<uchar>(j,i);
      // if low disparity, then ignore
      if (d < 2) {
        continue;
      }
      // V is the vector to be multiplied to Q to get
      // the 3D homogenous coordinates of the image point
      V.at<double>(0,0) = (double)(i);
      V.at<double>(1,0) = (double)(j);
      V.at<double>(2,0) = (double)d;
      V.at<double>(3,0) = 1.;
      pos = Q * V; // 3D homogeneous coordinate
      double X = pos.at<double>(0,0) / pos.at<double>(3,0);
      double Y = pos.at<double>(1,0) / pos.at<double>(3,0);
      double Z = pos.at<double>(2,0) / pos.at<double>(3,0);
      Mat point3d_cam = Mat(3, 1, CV_64FC1);
      point3d_cam.at<double>(0,0) = X;
      point3d_cam.at<double>(1,0) = Y;
      point3d_cam.at<double>(2,0) = Z;
      // transform 3D point from camera frame to robot frame
      Mat point3d_robot = XR * point3d_cam + XT;
      points.push_back(Point3d(point3d_robot));
      geometry_msgs::Point32 pt;
      pt.x = point3d_robot.at<double>(0,0);
      pt.y = point3d_robot.at<double>(1,0);
      pt.z = point3d_robot.at<double>(2,0);
      pc.points.push_back(pt);
      int32_t red, blue, green;
      red = img_left.at<Vec3b>(j,i)[2];
      green = img_left.at<Vec3b>(j,i)[1];
      blue = img_left.at<Vec3b>(j,i)[0];
      int32_t rgb = (red << 16 | green << 8 | blue);
      ch.values.push_back(*reinterpret_cast<float*>(&rgb));
    }
  }
  pc.channels.push_back(ch);
  pcl_pub.publish(pc);
}

void imgCallback(const sensor_msgs::ImageConstPtr& msg_left, const sensor_msgs::ImageConstPtr& msg_right)
{
  Mat imgL = cv_bridge::toCvShare(msg_left, "bgr8")->image;
  Mat imgR = cv_bridge::toCvShare(msg_right, "bgr8")->image;
  if (imgL.empty() || imgR.empty())
    return;
  JPP jpp(imgL, imgR, calib_file, cf);
  Mat dmap = jpp.get_disparity_map("elas");
  Mat imgL_rect = jpp.get_img_left();
  Mat imgL_rect_color;
  cvtColor(imgL_rect, imgL_rect_color, CV_GRAY2BGR);
  Mat Q = jpp.get_Q_matrix();
  publishPointCloud(imgL_rect_color, dmap, Q);
  sensor_msgs::ImagePtr dmap_msg = cv_bridge::CvImage(msg_left->header, "mono8", dmap).toImageMsg();
  dmap_pub.publish(dmap_msg);
  
  imshow("IMGL", imgL);
  imshow("IMGR", imgR);
  imshow("DISP", dmap);
  waitKey(30);
}

void paramsCallback(jpp::CamToRobotCalibParamsConfig &conf, uint32_t level) {
  config = conf;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "jpp_dense_reconstruction");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  
  const char* left_img_topic;
  const char* right_img_topic;
  const char* calib_file_name;
  const char* jpp_config_file;
  
  static struct poptOption options[] = {
    { "left_topic",'l',POPT_ARG_STRING,&left_img_topic,0,"Left image topic name","STR" },
    { "right_topic",'r',POPT_ARG_STRING,&right_img_topic,0,"Right image topic name","STR" },
    { "calib_file",'c',POPT_ARG_STRING,&calib_file_name,0,"Stereo calibration file name","STR" },
    { "jpp_config_file",'j',POPT_ARG_STRING,&jpp_config_file,0,"JPP config file name","STR" },
    { "debug",'d',POPT_ARG_INT,&debug,0,"Set d=1 for cam to robot frame calibration","NUM" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {}
  
  calib_file = FileStorage(calib_file_name, FileStorage::READ);
  
  // Read JPP config
  cf = &cfg;
  config_init(cf);
  if (!config_read_file(cf, jpp_config_file)) {
    cout << "Could not read config file!" << endl;
    config_destroy(cf);
    return(EXIT_FAILURE);
  }
 
  message_filters::Subscriber<sensor_msgs::Image> sub_img_left(nh, left_img_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> sub_img_right(nh, right_img_topic, 1);
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_img_left, sub_img_right);
  sync.registerCallback(boost::bind(&imgCallback, _1, _2));
  
  dynamic_reconfigure::Server<jpp::CamToRobotCalibParamsConfig> server;
  dynamic_reconfigure::Server<jpp::CamToRobotCalibParamsConfig>::CallbackType f;

  f = boost::bind(&paramsCallback, _1, _2);
  server.setCallback(f);
  
  dmap_pub = it.advertise("/camera/left/disparity_map", 1);
  pcl_pub = nh.advertise<sensor_msgs::PointCloud>("/camera/left/point_cloud",1);

  ros::spin();
  return 0;
}