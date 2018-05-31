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
	Mat m_current_frame;
	Mat m_roi_hist;
	Rect m_track_window;

	GeoCentre m_geo;

	std::deque<Point> m_li_center;
	std::deque<Point> m_li_center_afterlost;

	int cpt_reinit_histo;

	KalmanFilter KF;
	Mat_<float> measurement;
	Point KFPredictCenter;
	Point KFCorrectCenter;
	Point m_center_of_rect;
	Point lastCenter;

	bool isLost;

public:
	Tracking(string videoName);
	~Tracking();

	void savePic(Mat * im, int i); 

	Point2f searchTarget();
	void initializeHistogram(int width, int height, const int channels[], const int histSize[], float range[], const float *ranges[]);
	void calculHistogram(Mat &roi, const int channels[], const int histSize[], float range[], const float *ranges[], bool usemask = false);
	Point camShiftTracking(const int channels[], const int histSize[], float range[], const float *ranges[]);
	Point meanShiftTracking(const int channels[], const int histSize[], float range[], const float *ranges[]);
	void traceRoute(Mat& frame, bool foundTarget);


	void initKalman(double interval);
	Point getCurrentState() const;
	void setCurrentTrackWindow();
};

