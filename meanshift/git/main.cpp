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
#include "build\Tracking.h"

using namespace cv;
using namespace std;
using namespace xfeatures2d;

int main() {

	//string videoName = "../../../videos/Videos RSV Irisa/MVI_1111_trim_pie.mov";
	string videoName = "../../../videos/Videos RSV Irisa/MVI_1189_trim_cormorant.mov";
	Tracking track;

	//track.meanShiftAlgo(videoName, 325, 830, 100, 100);
	track.meanShiftAlgo(videoName, 1600, 415, 80, 80);

	//track.blobDetection(videoName);
	
	return 0;
}
