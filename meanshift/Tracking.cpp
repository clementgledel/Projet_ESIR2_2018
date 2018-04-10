#include "Tracking.h"

Tracking::Tracking(string videoName)
{
	// Create a VideoCapture object and open the input file
	// If the input is the web camera, pass 0 instead of the video file name
	cap = VideoCapture(videoName);

	Mat frame;
	// Condition video 2
	if (videoName.find("MVI_1189_trim_cormorant.mov") != -1)
	{
		for (int i = 0; i < 45; i++)
		{
			cap >> frame;
		}
	}
}

Tracking::~Tracking()
{
	destroyAllWindows();
	cap.release();
}

// save a picture, the name of the picture is its frame position in the video
void Tracking::savePic(Mat * im, int i) {
	std::ostringstream oss;
	oss << "./save/" << i << ".jpg";
	imwrite(oss.str(), *im);
}

void Tracking::calculHistogram(Mat &roi, const int channels[], const int histSize[], float range[], const float *ranges[], bool usemask)
{

	if (usemask)
	{
		//imwrite("roi.jpg", roi);
		Mat hsv_roi;
		cvtColor(roi, hsv_roi, COLOR_BGR2HSV);
		//imwrite("hsv_roi.jpg", hsv_roi);
		Mat maskroi;
		inRange(hsv_roi, Scalar(140., 0., 0.), Scalar(255., 255., 100.), maskroi); // Créer un masque contenant 255 lorsque la valeur est dans l'interval
		//imwrite("mask_roi.jpg", maskroi);

		calcHist(&roi, 1, channels, maskroi, roi_hist, 2, histSize, ranges, true, false);
	}
	else 
	{
		calcHist(&roi, 1, channels, noArray(), roi_hist, 2, histSize, ranges, true, false);
	}

	normalize(roi_hist, roi_hist, 0, 255, NORM_MINMAX);
}

void Tracking::initializeHistogram(int x, int y, int width, int height, const int channels[], const int histSize[], float range[], const float *ranges[])
{
	Mat frame;
	Mat hsv, dst;
	int k = 0;
	track_window = Rect(x, y, width, height);

	/**********************************************************/
	/*******************Calcul de l'Histogramme****************/
	/**********************************************************/
	cap >> frame;
	Mat roi = frame(track_window);

	// Calcul roi histogram
	calculHistogram(roi, channels, histSize, range, ranges, true);
	/**********************************************************/

}

Point Tracking::trackingCamShift(const int channels[], const int histSize[], float range[], const float *ranges[])
{
	Mat frame;
	Mat hsv, dst;

	cap >> frame;

	Point center_of_rect;

	if (!frame.empty())
	{

		cvtColor(frame, hsv, COLOR_BGR2HSV);
		calcBackProject(&frame, 1, channels, roi_hist, dst, ranges);

		/***********************************************************/
		/************************Camshift**************************/
		/**********************************************************/
		RotatedRect rot = CamShift(dst, track_window, TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 100, 0.01));

		if (track_window.area() <= 1)
		{
			int cols = dst.cols, rows = dst.rows, r = (MIN(cols, rows) + 5) / 6;
			//cout << cols << " " << rows << endl;
			track_window = Rect(track_window.x - r, track_window.y - r,
				track_window.x + r, track_window.y + r) &
				Rect(0, 0, cols, rows);
		}
		//cout << rot.size << " " << rot.boundingRect().br() << " " << rot.boundingRect().tl() << endl;
		if (rot.size.width >= 0 && rot.size.height >= 0)
		{
			/*if (cpt_frame <= 20)
			{
				cpt_frame++;
				som += rot.size.height*rot.size.width;
			}
			else
			{
				moy = som / 20;
				som = 0;
				std::cout << moy << endl;
				cpt_frame = 0;
			}*/
			ellipse(frame, rot, Scalar(0, 0, 255), 2, LINE_AA);
		}

		rectangle(frame, track_window, Scalar(255, 128, 128), 2);
		/********************************************************/

		center_of_rect = (track_window.br() + track_window.tl())*0.5;
		circle(frame, center_of_rect, 1, Scalar(0, 0, 255), 2 );

		imshow("img2", frame);

		//savePic(&frame, save_index);
		//save_index++;

		// Affichage Back Projection
		//imshow("dst", dst);
	}
	return center_of_rect;
}

void Tracking::trackingMeanShift(int x, int y, int width, int height)
{
	ofstream myfile("coordonnees.txt");
	if (myfile.is_open())
	{
		Mat frame;
		const int channels[] = { 0, 1 };
		const int histSize[] = { 64, 64 };
		float range[] = { 0, 256 };
		const float *ranges[] = { range, range };
		Mat hsv, dst;
		int k = 0;
		Rect track_window(x, y, width, height);

		/**********************************************************/
		/*******************Calcul de l'Histogramme****************/
		/**********************************************************/
		cap >> frame;
		Mat roi = frame(track_window);
		Mat roi_hist;

		// Calcul roi histogram
		calculHistogram(roi, channels, histSize, range, ranges, true);
		/**********************************************************/

		//int save_index = 0;
		int cpt_frame = 0;
		double moy = 0, som = 0;
		while (true)
		{
			cap >> frame;

			if (!frame.empty())
			{

				cvtColor(frame, hsv, COLOR_BGR2HSV);
				calcBackProject(&frame, 1, channels, roi_hist, dst, ranges);

				/***********************************************************/
				/************************Meanshift*************************/
				/**********************************************************/

				/*The functions calcBackProject calculate the back project of the histogram.
				That is, similarly to calcHist , at each location (x, y) the function collects the values from the selected channels in the input images and finds the corresponding histogram bin.
				But instead of incrementing it, the function reads the bin value, scales it by scale , and stores in backProject(x,y) .
				In terms of statistics, the function computes probability of each element value in respect with the empirical probability distribution represented by the histogram. */
				meanShift(dst, track_window, TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 100, 0.01));

				//void rectangle(Mat& img, Rect rec, const Scalar& color, int thickness=1, int lineType=8, int shift=0 )
				rectangle(frame, track_window, Scalar(255, 128, 128), 2);

				/********************************************************/
				

				Point center_of_rect = (track_window.br() + track_window.tl())*0.5;
				circle(frame, center_of_rect, 1, Scalar(0, 0, 255), 2);

				// Enregistrement des centres de chaque rectangle
				myfile << "[" << center_of_rect.x << ";" << center_of_rect.y << "]" << "\n";

				imshow("img2", frame);
				//savePic(&frame, save_index);
				//save_index++;

				// Affichage Back Projection
				//imshow("dst", dst);

				k = waitKey(60);
				if (k == 27)
					break;
			}
		}

		destroyAllWindows();
		cap.release();

		myfile.close();

	}
	else cout << "Unable to open file";

}

void Tracking::blobDetection(string videoName)
{
	// Create a VideoCapture object and open the input file
	// If the input is the web camera, pass 0 instead of the video file name
	VideoCapture cap(videoName);
	Mat im;
	// Set up the detector with default parameters.
	SimpleBlobDetector detector;
	std::vector<KeyPoint> keypoints;
	Mat im_with_keypoints;

	//while (true)
	//{
		cap >> im;
		//if (!im.empty())
		//{
			// Detect blobs.
			detector.detect(im, keypoints);

			// Draw detected blobs as red circles.
			// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
			drawKeypoints(im, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

			// Show blobs
			imshow("keypoints", im_with_keypoints);
		//}
	//}

	destroyAllWindows();
	cap.release();

	//waitKey(0);
}
