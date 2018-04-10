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

#include <ctype.h>

using namespace cv;
using namespace std;
using namespace xfeatures2d;

#pragma once
class Tracking
{
public:
	Tracking();
	~Tracking();

	void savePic(Mat * im, int i);
	void getHistogram(Mat &roi, Mat &hist, const int channels[], const int histSize[], float range[], bool usemask = false);
	void meanShiftAlgo(string videoName, int x, int y, int width, int height);
	void blobDetection(string videoName);
};

