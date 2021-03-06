#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <jpp/ParamsConfig.h>
#include <libconfig.h>
#include "jpp.h"
#include "popt_pp.h"

JPP *jpp_obj;
JPP_Config jpp_config;
config_t cfg, *cf;
FileStorage calib_file;
const char* output = "astar";
int w = 0;
int counter = 0;

void imgCallback(const sensor_msgs::ImageConstPtr& msg_left, const sensor_msgs::ImageConstPtr& msg_right)
{
  counter++;
  Mat img_left = cv_bridge::toCvShare(msg_left, "bgr8")->image;
  Mat img_right = cv_bridge::toCvShare(msg_right, "bgr8")->image;
  if (img_left.empty() || img_right.empty())
    return;
  
  char rrt_file_prefix[100], astar_file_prefix[100];
  sprintf(astar_file_prefix, "%s%d", "astar", counter);
  sprintf(rrt_file_prefix, "%s%d", "rrt", counter);
  
  jpp_obj->load_images(img_left, img_right);
  jpp_obj->update_jpp_config(jpp_config);
  
  if (strcmp(output, "astar") == 0) {
    pair< Mat, Mat > vis;
    if (w == 1)
      vis = jpp_obj->plan_astar(astar_file_prefix);
    else
      vis = jpp_obj->plan_astar();
    imshow("PATH", vis.first);
    imshow("CONFIDENCE MATCHING", vis.second);
  } else if (strcmp(output, "rrt") == 0) {
    pair< Mat, Mat > vis;
    if (w == 1) 
      vis = jpp_obj->plan_rrt(rrt_file_prefix);
    else
      vis = jpp_obj->plan_rrt();
    imshow("PATH", vis.first);
    imshow("CONFIDENCE MATCHING", vis.second);
  } else if (strcmp(output, "debug") == 0) {
    Mat conf_pos = jpp_obj->visualize_conf_pos();
    Mat conf_neg = jpp_obj->visualize_conf_neg();
    imshow("CONF POS", conf_pos);
    imshow("CONF NEG", conf_neg);
  }
  waitKey(30);
}

void paramsCallback(jpp::ParamsConfig &conf, uint32_t level) {
  jpp_config.DR = conf.DR;
  jpp_config.DQ = conf.DQ;
  jpp_config.DT = conf.DT;
  jpp_config.DH = conf.DH;
  jpp_config.BOT_LENGTH = conf.LENGTH;
  jpp_config.BOT_WIDTH = conf.WIDTH;
  jpp_config.BOT_HEIGHT = conf.HEIGHT;
  jpp_config.GRID_SIZE = conf.GRID_SIZE;
  jpp_config.CONF_POS_THRESH = conf.CONF_POS_THRESH;
  jpp_config.CONF_NEG_THRESH = conf.CONF_NEG_THRESH;
  jpp_config.SAD_WINDOW_SIZE = conf.SAD_WINDOW_SIZE;
  jpp_config.SPATIAL_FILTER_WINDOW = conf.SPATIAL_FILTER_WINDOW;
  jpp_config.SPATIAL_FILTER_INC = conf.SPATIAL_FILTER_INC;
  jpp_config.SPATIAL_FILTER_RATIO = conf.SPATIAL_FILTER_RATIO;
  jpp_config.CONF_NEG_INC = conf.CONF_NEG_INC;
  jpp_config.CONF_NEG_FILTER_RATIO = conf.CONF_NEG_FILTER_RATIO;
  jpp_config.START_X = conf.START_X;
  jpp_config.MAX_X = conf.MAX_X;
  jpp_config.MAX_Y = conf.MAX_Y;
  jpp_config.CONVEX_WORLD = conf.CONVEX_WORLD;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "jpp_navigation");
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
    { "output",'o',POPT_ARG_STRING,&output,0,"Output - astar, rrt, debug","STR" },
    { "write_files",'w',POPT_ARG_INT,&w,0,"Set w=1 for writing visualizations to files","NUM" },
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
  jpp_config = JPP_Config(cf);
  
  jpp_obj = new JPP(calib_file, cf);
  
  message_filters::Subscriber<sensor_msgs::Image> sub_img_left(nh, left_img_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> sub_img_right(nh, right_img_topic, 1);
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_img_left, sub_img_right);
  sync.registerCallback(boost::bind(&imgCallback, _1, _2));
  
  dynamic_reconfigure::Server<jpp::ParamsConfig> server;
  dynamic_reconfigure::Server<jpp::ParamsConfig>::CallbackType f;

  f = boost::bind(&paramsCallback, _1, _2);
  server.setCallback(f);

  ros::spin();
  return 0;
}