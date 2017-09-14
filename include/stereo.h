#ifndef STEREO_H
#define STEREO_H

#include "util.h"

using namespace std;
using namespace cv;
using namespace kutility;

class Stereo {
private:
  JPP_Config _jpp_config;
  Mat _img_left, _img_right;
  Mat _XR, _XRINV, _XT, _Q, _P1, _P2;
  Mat _R1, _R2, _K1, _K2, _D1, _D2, _R;
  Mat _lmapx, _lmapy, _rmapx, _rmapy;
  Vec3d _T;
  daisy *_desc_left, *_desc_right;
  Mat _obstacleCache;
  Mat _obstacleRangeCache;
  Mat _colCache;
  Mat _confNegCache;
  Mat _descLeftCache, _descRightCache;
  Mat _descLeftSet, _descRightSet;
  Mat _cacheVis;
  long _computation_count;
  
  void _get_rectification_map(Mat& left, Mat& right, FileStorage& fs);
  void _compute_dense_descriptors();
  double _desc_cost(Point left, Point right, int w);
public:
  Stereo();
  Stereo(Mat& l, Mat& r, FileStorage& fs, JPP_Config& config);
  Stereo& operator=(Stereo& s);
  ~Stereo();
  bool in_img(int x, int y);
  Point project_point_cam(const Point3f p, int cam);
  Mat get_disparity_map_elas();
  void init_daisy_descriptors(int rad, int radq, int thq, int histq, int nrm_type);
  bool conf_positive(const Point3f p);
  bool conf_negative(const Point3f p);
  bool is_obstacle_free_region(const Point3f p);
  bool is_empty_col(const Point3f p);
  bool is_bot_clear(const Point3f p, float safe_radius, float inc, bool col_check);
  void jpp_visualizations(Mat& confPos, Mat& confNeg);
  void blend_images(Mat& src1, Mat& src2, float alpha, Mat& dst);
  void update_jpp_config(JPP_Config& config);
  Mat get_img_left();
  Mat get_img_right();
  Mat get_Q_matrix();
};

#endif // STEREO_H