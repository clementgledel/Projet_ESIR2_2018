#include <iostream>
#include <sstream>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/sfm/fundamental.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <vector>

using namespace std;
using namespace cv;
using namespace xfeatures2d;
using namespace sfm;


class CalibParam{

public:
  Mat t;
  Mat R;
  double compactR1[3];
  double m_compactR[3];
  double m_t[3];

  CalibParam():
  t(),R(),compactR1(),m_compactR(),m_t()
  {}
};
