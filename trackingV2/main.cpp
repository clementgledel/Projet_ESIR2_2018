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
//#include "build\CalibInit.h"
//#include "build\CalibParam.h"
#include "build\MainWindow.h"

using namespace cv;
using namespace std;
using namespace xfeatures2d;

int main() {

	//string videoName = "../../../videos/Videos RSV Irisa/MVI_1111_trim_pie.mov";
	string videoName = "../../../videos/Videos RSV Irisa/MVI_1189_trim_cormorant.mov";
	Tracking track(videoName);
	int width = 60, height = 60;
	int k = 0;

	ofstream myfile("coordonnees.txt");
	if (myfile.is_open())
	{
		const int channels[] = { 0, 1 };
		const int histSize[] = { 64, 64 };
		float range[] = { 0, 256 };
		const float *ranges[] = { range, range };

		track.initializeHistogram(width, height, channels, histSize, range, ranges);

		while (true)
		{
			Point center_of_rect = track.meanShiftTracking(channels, histSize, range, ranges);
			//Point center_of_rect = track.camShiftTracking(channels, histSize, range, ranges);

			// Enregistrement des centres de chaque rectangle
			myfile << "[" << center_of_rect.x << ";" << center_of_rect.y << "]" << "\n";

			k = waitKey(60);
			if (k == 27)
				break;

		}

		myfile.close();
	}
	else cout << "Unable to open file";



	//track.blobDetection(videoName);
	
	return 0;
}
