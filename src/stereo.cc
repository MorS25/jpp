#include "stereo.h"

Stereo::Stereo()
{
  _computation_count = 0;
}

Stereo::Stereo(Mat& l, Mat& r, FileStorage& fs, JPP_Config& config)
{
  _jpp_config = config;
  _get_rectification_map(l, r, fs);
  _computation_count = 0;
  _obstacleCache = Mat(_img_left.rows, _img_left.cols, CV_8UC1, Scalar(0));
  _obstacleRangeCache = Mat(_img_left.rows, _img_left.cols, CV_8UC1, Scalar(0));
  _colCache = Mat(_img_left.rows, _img_left.cols, CV_8UC1, Scalar(0));
  //_disparityMap = Mat(_img_left.rows, _img_left.cols, CV_8UC2, Scalar(0,0));
  _confNegCache = Mat(_img_left.rows, _img_left.cols, CV_8UC1, Scalar(0));
  //_dMapVis = Mat(img_left.rows, img_left.cols, CV_8UC3, Scalar(0,0,0));
  _descLeftSet = Mat(_img_left.rows, _img_left.cols, CV_8UC1, Scalar(0));
  _descRightSet = Mat(_img_left.rows, _img_left.cols, CV_8UC1, Scalar(0));
  //_cacheVis = Mat(_img_left.rows, _img_left.cols, CV_8UC3, Scalar(0,0,0));
  cvtColor(_img_left, _cacheVis, CV_GRAY2BGR);
  _desc_left = new daisy;
  _desc_right = new daisy;
}

Stereo& Stereo::operator=(Stereo& s)
{
  if (this == &s) return *this;
  this->_jpp_config = s._jpp_config;
  this->_img_left = s._img_left.clone();
  this->_img_right = s._img_right.clone();
  this->_XR = s._XR.clone();
  this->_XRINV = s._XRINV.clone();
  this->_XT = s._XT.clone();
  this->_Q = s._Q.clone();
  this->_P1 = s._P1.clone();
  this->_P2 = s._P2.clone();
  this->_R1 = s._R1.clone();
  this->_R2 = s._R2.clone();
  this->_D1 = s._D1.clone();
  this->_D2 = s._D2.clone();
  this->_R = s._R.clone();
  this->_lmapx = s._lmapx.clone();
  this->_lmapy = s._lmapy.clone();
  this->_rmapx = s._rmapx.clone();
  this->_rmapy = s._rmapy.clone();
  this->_T = s._T;
  this->_obstacleCache = s._obstacleCache.clone();
  this->_obstacleRangeCache = s._obstacleRangeCache.clone();
  this->_colCache = s._colCache.clone();
  this->_confNegCache = s._confNegCache.clone();
  this->_descLeftCache = s._descLeftCache.clone();
  this->_descRightCache = s._descRightCache.clone();
  this->_descLeftSet = s._descRightSet.clone();
  this->_cacheVis = s._cacheVis.clone();
  this->_computation_count = s._computation_count;
  this->_desc_left = new daisy(*s._desc_left);
  this->_desc_right = new daisy(*s._desc_right);
  return *this;
}

Stereo::~Stereo()
{
  if (_desc_left != NULL)
    delete _desc_left;
  if (_desc_right != NULL)
    delete _desc_right;
}

bool Stereo::in_img(int x, int y)
{
  if (x >= 0 && x < _img_left.cols && y >= 0 && y < _img_left.rows)
    return true;
  return false;
}

Point Stereo::project_point_cam(const Point3f p, int cam)
{
  Mat pt3d = (Mat_<double>(3, 1) << p.x, p.y, p.z);
  Mat pt3d_cam = _XRINV*(pt3d - _XT);
  Mat pt3d_cam_hom = (Mat_<double>(4, 1) << pt3d_cam.at<double>(0,0), 
                      pt3d_cam.at<double>(1,0), pt3d_cam.at<double>(2,0), 1.);
  Mat img_coord;
  if (cam == 0)
    img_coord = _P1 * pt3d_cam_hom;
  else
    img_coord = _P2 * pt3d_cam_hom;
  Point imgc;
  imgc.x = img_coord.at<double>(0,0)/img_coord.at<double>(2,0);
  imgc.y = img_coord.at<double>(1,0)/img_coord.at<double>(2,0);
  return imgc;
}

Mat Stereo::get_disparity_map_elas()
{
  if (_img_left.empty() || _img_right.empty()) 
    return _img_left;
  const Size imsize = _img_left.size();
  const int32_t dims[3] = {imsize.width, imsize.height, imsize.width};
  Mat leftdpf = Mat::zeros(imsize, CV_32F);
  Mat rightdpf = Mat::zeros(imsize, CV_32F);

  Elas::parameters param(Elas::MIDDLEBURY);
  param.postprocess_only_left = true;
  Elas elas(param);
  elas.process(_img_left.data, _img_right.data, leftdpf.ptr<float>(0), rightdpf.ptr<float>(0), dims);
  Mat dmap = Mat(Size(_jpp_config.RECT_IMG_WIDTH, _jpp_config.RECT_IMG_HEIGHT), CV_8UC1, Scalar(0));
  leftdpf.convertTo(dmap, CV_8U, 1.);
  return dmap;
}

void Stereo::init_daisy_descriptors(int rad, int radq, int thq, int histq, int nrm_type)
{
  int verbose_level = 0;
  bool disable_interpolation = true;
  // associate pointer
  uchar *imL = _img_left.data;
  uchar *imR = _img_right.data;
  int h = _img_left.rows;
  int w = _img_left.cols;

  _desc_left->reset();
  _desc_left->set_image(imL, h, w);
  _desc_left->verbose(verbose_level);
  _desc_left->set_parameters(rad, radq, thq, histq);
  _desc_left->set_normalization(nrm_type);
  _desc_left->initialize_single_descriptor_mode();
  
  _desc_right->reset();
  _desc_right->set_image(imR, h, w);
  _desc_right->verbose(verbose_level);
  _desc_right->set_parameters(rad, radq, thq, histq);
  _desc_right->set_normalization(nrm_type);
  _desc_right->initialize_single_descriptor_mode();
  
  _descLeftCache.create(_img_left.cols * _img_left.rows, _desc_left->descriptor_size(), CV_32FC1);
  _descRightCache.create(_img_right.cols * _img_right.rows, _desc_right->descriptor_size(), CV_32FC1);
}

bool Stereo::conf_positive(const Point3f p)
{
  Point ptl = project_point_cam(p, 0);
  Point ptr = project_point_cam(p, 1);
  if (!in_img(ptl.x,ptl.y) || !in_img(ptr.x,ptr.y)) {
    return false;
  }
  if ((int)_obstacleCache.at<uchar>(ptl) == 1) // obstacle free
    return true;
  if ((int)_obstacleCache.at<uchar>(ptl) == 2) // obstacle
    return false;
  
  int w = _jpp_config.SAD_WINDOW_SIZE;
  double cost = _desc_cost(ptl, ptr, w);
  cost /= (double)((2*w+1)*(2*w+1));
  if (cost < _jpp_config.CONF_POS_THRESH) {
    _obstacleCache.at<uchar>(ptl) = 1;
    return true;
  } else {
    _obstacleCache.at<uchar>(ptl) = 2;
  }
  return false;
}

bool Stereo::conf_negative(const Point3f p)
{
  Point ptl = project_point_cam(p, 0);
  Point ptr = project_point_cam(p, 1);
  if (!in_img(ptl.x,ptl.y) || !in_img(ptr.x,ptr.y)) {
    return false;
  }
  if ((int)_confNegCache.at<uchar>(ptl) == 1) // obstacle free
    return true;
  if ((int)_confNegCache.at<uchar>(ptl) == 2) // obstacle
    return false;
  int w = _jpp_config.SAD_WINDOW_SIZE;
  double cost = _desc_cost(ptl, ptr, w);
  cost /= (double)((2*w+1)*(2*w+1));
  if (cost > _jpp_config.CONF_NEG_THRESH) {
    _confNegCache.at<uchar>(ptl) = 1;
    return true;
  }
  _confNegCache.at<uchar>(ptl) = 2;
  return false;
}

bool Stereo::is_obstacle_free_region(const Point3f p)
{
  Point ptl = project_point_cam(p, 0);
  if ((int)_obstacleRangeCache.at<uchar>(ptl) == 1) // obstacle free range
    return true;
  if ((int)_obstacleRangeCache.at<uchar>(ptl) == 2) // obstacle range
    return false;
  int count = 0;
  int total_points = 0;
  float w = (float)_jpp_config.SPATIAL_FILTER_WINDOW/1000.;
  float inc = (float)_jpp_config.SPATIAL_FILTER_INC/1000.;
  for (float x = 0; x <= w; x += inc) {
    for (float y = -w; y <= w; y += inc) {
      Point3f q(p.x+x, p.y+y, 0);
      if (conf_positive(q))
        count++;
      total_points++;
    }
  }
  float ratio = _jpp_config.SPATIAL_FILTER_RATIO;
  if (count > (float)total_points * ratio ) {
    _obstacleRangeCache.at<uchar>(ptl) = 1;
    return true;
  } else {
    _obstacleRangeCache.at<uchar>(ptl) = 2;
  }
  return false;
}

bool Stereo::is_empty_col(const Point3f p)
{
  Point ptl = project_point_cam(p, 0);
  if ((int)_colCache.at<uchar>(ptl) == 1) // obstacle free col
    return true;
  if ((int)_colCache.at<uchar>(ptl) == 2) // obstacle col
    return false;
  float inc = (float)_jpp_config.CONF_NEG_INC/1000.;
  int total = 0;
  int match = 0;
  float h = (float)_jpp_config.BOT_HEIGHT/1000.;
  for (float z = inc; z <= h; z += inc) {
    Point3f q(p.x,p.y,z);
    Point ptl = project_point_cam(q, 0);
    Point ptr = project_point_cam(q, 1);
    if (!in_img(ptl.x,ptl.y) || !in_img(ptr.x,ptr.y)) {
      continue;  
    }
    total++;
    if (!conf_negative(q)) {
      match++;
    }
  }
  if (total < 3) {
    _colCache.at<uchar>(ptl) = 2;
    return false;
  }
  if (match > (float)total * _jpp_config.CONF_NEG_FILTER_RATIO) {
    _colCache.at<uchar>(ptl) = 1;
    return true;
  }
  _colCache.at<uchar>(ptl) = 2;
  return false;
}

bool Stereo::is_bot_clear(const Point3f p, float safe_radius, float inc, bool col_check)
{
  //return _conf_positive(p);
  bool isFree = true;
  for (float y = -safe_radius; y <= safe_radius; y += inc) {
    for (float x = 0; x <= safe_radius; x += inc) {
      Point3f q(p.x+x,p.y+y,0.0);
      if (!conf_positive(q)) {
        isFree = false;
        break;
      } else {
        if (col_check) {
          if (!is_empty_col(q)) {
            isFree = false;
            break;
          }
        }
      }
    }
  }
  return isFree;
}

void Stereo::jpp_visualizations(Mat& confPos, Mat& confNeg)
{
  for (int i = 0; i < confPos.cols; i++) {
    for (int j = 0; j < confPos.rows; j++) {
      if ((int)_obstacleCache.at<uchar>(j,i) == 1) { // obstacle free
        if (!_jpp_config.CONVEX_WORLD && (int)_colCache.at<uchar>(j,i) == 2) {
          circle(confPos,Point(i,j),2,Scalar(0,0,255),-1,8,0);
        } else {
          circle(confPos,Point(i,j),2,Scalar(0,255,0),-1,8,0);
        }
        //cacheVis.at<Vec3b>(j,i) = Vec3b(0,255,0);
      }
      else if ((int)_obstacleCache.at<uchar>(j,i) == 2) { // obstacle
        circle(confPos,Point(i,j),3,Scalar(0,0,255),-1,8,0);
        //cacheVis.at<Vec3b>(j,i) = Vec3b(0,0,255);
      } else {
        //int col = (int)img_left.at<uchar>(j,i);
        //cacheVis.at<Vec3b>(j,i) = Vec3b(col,col,col);
      }
      if ((int)_confNegCache.at<uchar>(j,i) == 2) { // obstacle free
        circle(confNeg,Point(i,j),1,Scalar(0,200,200),-1,8,0);
      }
      else if ((int)_confNegCache.at<uchar>(j,i) == 1) { // obstacle
        circle(confNeg,Point(i,j),1,Scalar(255,0,255),-1,8,0);
        //cacheVis.at<Vec3b>(j,i) = Vec3b(0,0,255);
      } else {
      }
    }
  }
}

void Stereo::blend_images(Mat& src1, Mat& src2, float alpha, Mat& dst)
{
  float beta = (1.0 - alpha);
  addWeighted( src1, alpha, src2, beta, 0.0, dst);
}

void Stereo::update_jpp_config(JPP_Config& config)
{
  _jpp_config = config;
}

Mat Stereo::get_img_left()
{
  return _img_left;
}

Mat Stereo::get_img_right()
{
  return _img_right;
}

Mat Stereo::get_Q_matrix()
{
  return _Q;
}

///////// PRIVATE FUNCTIONS /////////

void Stereo::_get_rectification_map(Mat& left, Mat& right, FileStorage& fs)
{
  fs["K1"] >> _K1;
  fs["K2"] >> _K2;
  fs["D1"] >> _D1;
  fs["D2"] >> _D2;
  fs["R"] >> _R;
  fs["T"] >> _T;
  fs["XR"] >> _XR;
  fs["XT"] >> _XT;
  Rect validRoi[2];
  Size calib_img_size = Size(_jpp_config.CALIB_IMG_WIDTH, _jpp_config.CALIB_IMG_HEIGHT);
  Size rect_img_size = Size(_jpp_config.RECT_IMG_WIDTH, _jpp_config.RECT_IMG_HEIGHT);
  
  _XRINV = _XR.inv();
  stereoRectify(_K1, _D1, _K2, _D2, calib_img_size, _R, Mat(_T), _R1, _R2, _P1, _P2, _Q, 
                CV_CALIB_ZERO_DISPARITY, 0, rect_img_size, &validRoi[0], &validRoi[1]);
  initUndistortRectifyMap(_K1, _D1, _R1, _P1, rect_img_size, CV_32F, _lmapx, _lmapy);
  initUndistortRectifyMap(_K2, _D2, _R2, _P2, rect_img_size, CV_32F, _rmapx, _rmapy);
  remap(left, _img_left, _lmapx, _lmapy, cv::INTER_LINEAR);
  remap(right, _img_right, _rmapx, _rmapy, cv::INTER_LINEAR);
  cvtColor(_img_left, _img_left, CV_BGR2GRAY);
  cvtColor(_img_right, _img_right, CV_BGR2GRAY);
}

void Stereo::_compute_dense_descriptors()
{
  _desc_left->compute_descriptors();
  _desc_left->normalize_descriptors();
  int descSize;
  int h = _img_left.rows;
  int w = _img_left.cols;
  descSize = _desc_left->descriptor_size();
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      float* thor = NULL;
      _desc_left->get_descriptor(y, x, thor);
      memcpy(_descLeftCache.ptr(y*w+x), thor, descSize*sizeof(float));
    }
  }
  _desc_right->compute_descriptors();
  _desc_right->normalize_descriptors();
  h = _img_right.rows;
  w = _img_right.cols;
  descSize = _desc_right->descriptor_size();
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      float* thor = NULL;
      _desc_right->get_descriptor(y, x, thor);
      memcpy(_descRightCache.ptr(y*w+x), thor, descSize*sizeof(float));
    }
  }
  _descLeftSet = Mat(_img_left.rows, _img_left.cols, CV_8UC1, Scalar(1));
  _descRightSet = Mat(_img_left.rows, _img_left.cols, CV_8UC1, Scalar(1));
}

double Stereo::_desc_cost(Point left, Point right, int w)
{
  int width = _img_left.cols;
  double cost = 0;
  for (int j = -w; j <= w; j++) {
    for (int k = -w; k <= w; k++) {
      if (!in_img(left.x+j, left.y+k) || !in_img(right.x+j, right.y+k))
        continue;
      if ((int)_descLeftSet.at<uchar>(left.y+k,left.x+j) == 1 && 
          (int)_descRightSet.at<uchar>(right.y+k,right.x+j) == 1) {
        Mat dl_cache = _descLeftCache.row((left.y+k)*width+(left.x+j));
        Mat dr_cache = _descRightCache.row((right.y+k)*width+(right.x+j));
        cost += norm(dl_cache, dr_cache, CV_L1);
        continue;
      }
      float* dl = new float[_desc_left->descriptor_size()];
      float* dr = new float[_desc_right->descriptor_size()];
      _desc_left->get_descriptor(left.y+k, left.x+j, 0, dl);
      _desc_right->get_descriptor(right.y+k, right.x+j, 0, dr);
      memcpy(_descLeftCache.ptr((left.y+k)*width+(left.x+j)), dl, _desc_left->descriptor_size()*sizeof(float));
      memcpy(_descRightCache.ptr((right.y+k)*width+(right.x+j)), dr, _desc_right->descriptor_size()*sizeof(float));
      _descLeftSet.at<uchar>(left.y+k,left.x+j) = 1;
      _descRightSet.at<uchar>(right.y+k,right.x+j) = 1;
      for (int zz = 0; zz < _desc_left->descriptor_size(); zz++) {
        cost += fabs(dl[zz] - dr[zz]);
      }
      delete[] dl;
      delete[] dr;
    }
  }
  _computation_count++;
  return cost;
}