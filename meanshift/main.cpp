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

Mat image;

bool backprojMode = false;
bool selectObject = false;
int trackObject = 0;
bool showHist = true;
Point origin;
Rect selection;
int vmin = 10, vmax = 256, smin = 30;

// save a picture, the name of the picture is its frame position in the video
void savePic(Mat * im, int i) {
	std::ostringstream oss;
	oss << "./save/" << i << ".jpg";
	imwrite(oss.str(), *im);
}

// User draws box around object to track. This triggers CAMShift to start tracking
static void onMouse( int event, int x, int y, int, void* )
{
    if( selectObject )
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);

        selection &= Rect(0, 0, image.cols, image.rows);
    }

    switch( event )
    {
    case EVENT_LBUTTONDOWN:
        origin = Point(x,y);
        selection = Rect(x,y,0,0);
        selectObject = true;
        break;
    case EVENT_LBUTTONUP:
        selectObject = false;
        if( selection.width > 0 && selection.height > 0 )
            trackObject = -1;   // Set up CAMShift properties in main() loop
        break;
    }
}

int main() {

	string videoName = "../../../videos/Videos RSV Irisa/MVI_1111_trim_pie.mov";
	// Create a VideoCapture object and open the input file
	// If the input is the web camera, pass 0 instead of the video file name
	VideoCapture cap(videoName);

	int hsize[] = {16,16,16};
    float hranges[] = { 0,180 };
    const float* phranges[] = {hranges,hranges,hranges};
	
    Mat frame;
    cap>>frame;
    Rect r(325, 830, 100, 100);  
    Rect track_window = r;
    Mat roi = frame(r);
    Mat hsv_roi,hsv,dst;
    cvtColor(roi,hsv_roi,COLOR_BGR2HSV);
    Mat maskroi;
    inRange(hsv_roi, Scalar(0., 60., 32.), Scalar(180., 255., 255.),maskroi);
    Mat roi_hist;
    int ch[] = { 0, 1, 2 };
    calcHist(&roi, 1, ch, maskroi, roi_hist, 1, hsize, phranges);
    normalize(roi_hist, roi_hist, 0, 255, NORM_MINMAX);
    int k=0;
    while (true)
    {
        cap>>frame;
        if (!frame.empty())
        {
            cvtColor(frame, hsv,COLOR_BGR2HSV);
            calcBackProject(&hsv, 1, ch, roi_hist, dst, phranges);
            meanShift(dst, track_window, TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 10, 1));

            rectangle(frame, track_window, Scalar(255,128,128), 2);
            imshow("img2", frame);
            k = waitKey(60);
            if (k == 27)
                break;
            /*else
                imwrite("a.jpg", frame);
*/
        }

    }
	destroyAllWindows();
	cap.release();

	return 0;
}
