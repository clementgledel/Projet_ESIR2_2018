#include "opencv2/opencv.hpp"
#include <iostream>
#include <sstream>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <vector>
#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "GeoCentre.h"

#include <ctype.h>

using namespace cv;
using namespace std;
using namespace xfeatures2d;

#pragma once
class Tracking
{
private:
	VideoCapture m_cap;
	Mat m_roi_hist;
	Rect m_track_window;

	GeoCentre * m_geo;

	std::deque<Point> m_li_center;
	std::deque<Point> m_li_center_afterlost;

public:
	Tracking(string videoName);
	~Tracking();

	void savePic(Mat * im, int i); 

	void initializeHistogram(int x, int y, int width, int height, const int channels[], const int histSize[], float range[], const float *ranges[]);
	void calculHistogram(Mat &roi, const int channels[], const int histSize[], float range[], const float *ranges[], bool usemask = false);
	Point camShiftTracking(const int channels[], const int histSize[], float range[], const float *ranges[]);
	Point meanShiftTracking(const int channels[], const int histSize[], float range[], const float *ranges[]);
	void blobDetection(string videoName);
	void traceRoute(Mat& frame, bool foundTarget);
};

