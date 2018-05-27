#include "Tracking.h"

Tracking::Tracking(string videoName)
{
	// Create a VideoCapture object and open the input file
	// If the input is the web camera, pass 0 instead of the video file name
	m_geo = new GeoCentre();
	m_cap = VideoCapture(videoName);

	Mat frame;
	// Condition video 2
	if (videoName.find("MVI_1189_trim_cormorant.mov") != -1)
	{
		for (int i = 0; i < 45; i++)
		{
			m_cap >> frame;
		}
	}
}

Tracking::~Tracking()
{
	destroyAllWindows();
	m_cap.release();
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

		calcHist(&roi, 1, channels, maskroi, m_roi_hist, 2, histSize, ranges, true, false);
	}
	else 
	{
		calcHist(&roi, 1, channels, noArray(), m_roi_hist, 2, histSize, ranges, true, false);
	}

	normalize(m_roi_hist, m_roi_hist, 0, 255, NORM_MINMAX);
}

void Tracking::initializeHistogram(int x, int y, int width, int height, const int channels[], const int histSize[], float range[], const float *ranges[])
{
	Mat frame;
	Mat hsv, dst;
	int k = 0;
	m_track_window = Rect(x, y, width, height);

	/**********************************************************/
	/*******************Calcul de l'Histogramme****************/
	/**********************************************************/
	m_cap >> frame;
	Mat roi = frame(m_track_window);

	// Calcul roi histogram
	calculHistogram(roi, channels, histSize, range, ranges, true);
	/**********************************************************/

}

void Tracking::traceRoute(Mat& frame, bool foundTarget)
{
	Point center_of_rect;

	center_of_rect = Point(m_track_window.x + m_track_window.width / 2, m_track_window.y + m_track_window.height / 2);

	// Si une cible est tracké
	if (foundTarget)
	{
		// Ajout à la liste des centres de fenetres de tracking le centre actuel
		m_li_center.push_back(center_of_rect);
		// On pose une limite de 15 centres dans la liste afin de faire disparaitre la trainée de parcours derrière l'oiseau
		if (m_li_center.size() == 16) m_li_center.pop_front();
		// On retire de la liste de centre avant perte de cible le dernier centre trouvé
		if (m_li_center_afterlost.size() != 0) m_li_center_afterlost.pop_front();
	}
	else {
		// Si nous perdons la cible et que la liste de centre n'est pas vide ainsi que celle des centres après perte
		// nous supprimons les centres de la liste après perte avant de récupérer celle de la liste de centre afin d'éviter des liaisons entre les points
		// avant la première perte et la seconde
		if (m_li_center.size() != 0 && m_li_center_afterlost.size() != 0) m_li_center_afterlost.clear();
		m_li_center_afterlost.insert(m_li_center_afterlost.begin(), m_li_center.begin(), m_li_center.end());
		m_li_center.clear();
	}

	double dist = 0;
	int valcolor = 0;

	// Parcours des points avant perte s'il y en a
	if (m_li_center_afterlost.size() != 0)
	{
		Point prec_lost = m_li_center_afterlost.front();

		// Affichage sous forme de lignes avec le point précedent de couleur variant du bleu au rouge en fonction de la distance parcourue
		for (auto it = m_li_center_afterlost.begin(), end = m_li_center_afterlost.end(); it != end; it++)
		{
			if (prec_lost != *it) {
				dist = sqrt(pow((*it - prec_lost).x, 2) + pow((*it - prec_lost).y, 2));
				valcolor = dist / 30 * 255;
				if (valcolor > 255) valcolor = 255;
			}
			if (prec_lost != *it) line(frame, prec_lost, *it, Scalar(255 - valcolor, 0, valcolor), 3);
			prec_lost = *it;
		}
	}

	// Parours des centres obtenues
	if (m_li_center.size() != 0)
	{
		Point prec = m_li_center.front();

		// Affichage sous forme de lignes avec le point précedent de couleur variant du bleu au rouge en fonction de la distance parcourue
		for (auto it = m_li_center.begin(), end = m_li_center.end(); it != end; it++)
		{
			if (prec != *it) {
				dist = sqrt(pow((*it - prec).x, 2) + pow((*it - prec).y, 2));
				valcolor = dist / 30 * 255;
				if (valcolor > 255) valcolor = 255;
			}
			if (prec != *it) line(frame, prec, *it, Scalar(255 - valcolor, 0, valcolor), 3);
			prec = *it;
		}
	}
}

Point Tracking::camShiftTracking(const int channels[], const int histSize[], float range[], const float *ranges[])
{
	Mat frame;
	Mat hsv, dst;

	m_cap >> frame;

	Point center_of_rect;

	if (!frame.empty())
	{

		cvtColor(frame, hsv, COLOR_BGR2HSV);
		calcBackProject(&frame, 1, channels, m_roi_hist, dst, ranges);

		RotatedRect rot = CamShift(dst, m_track_window, TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 100, 0.01));
		
		if (m_track_window.area() <= 1)
		{
			int cols = dst.cols, rows = dst.rows, r = (MIN(cols, rows) + 5) / 6;
			m_track_window = Rect(m_track_window.x - r, m_track_window.y - r,
				m_track_window.x + r, m_track_window.y + r) &
				Rect(0, 0, cols, rows);
		}

		rectangle(frame, m_track_window, Scalar(255, 128, 128), 2);

		center_of_rect = Point(m_track_window.x + m_track_window.width / 2, m_track_window.y + m_track_window.height / 2);

		// Affichage d'une ellipse représentant le rectangle rot renvoyé par le CamShift
		ellipse(frame, rot, Scalar(0, 0, 255), 2, LINE_AA);

		traceRoute(frame, rot.size.width > 0 && rot.size.height > 0);

		imshow("img2", frame);

		//savePic(&frame, save_index);
		//save_index++;

		/*// On récupère le point du centroid
		Mat roi = frame(m_track_window);
		vector<Point2f> centroid;

		Mat * res;

		//geo->setPictures(roi);
		//geo->DO();
		res = geo->getResult();
		if (!res->empty()) {
			centroid = geo->getCoords();
			//cout << centroid[0];
		}

		if(centroid.size() != 0)
			circle(frame, centroid[0], 10, Scalar(255, 0, 255), 2);*/
	}

	return center_of_rect;
}

Point Tracking::meanShiftTracking(const int channels[], const int histSize[], float range[], const float *ranges[])
{
	Mat frame;
	Mat hsv, dst;

	m_cap >> frame;

	Point center_of_rect;

	if (!frame.empty())
	{

		cvtColor(frame, hsv, COLOR_BGR2HSV);
		
		/*The functions calcBackProject calculate the back project of the histogram.
		That is, similarly to calcHist , at each location (x, y) the function collects the values from the selected channels in the input images and finds the corresponding histogram bin.
		But instead of incrementing it, the function reads the bin value, scales it by scale , and stores in backProject(x,y) .
		In terms of statistics, the function computes probability of each element value in respect with the empirical probability distribution represented by the histogram. */
		calcBackProject(&frame, 1, channels, m_roi_hist, dst, ranges);

		/***********************************************************/
		/************************Meanshift*************************/
		/**********************************************************/

		int val = meanShift(dst, m_track_window, TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 100, 0.01));
		
		//void rectangle(Mat& img, Rect rec, const Scalar& color, int thickness=1, int lineType=8, int shift=0 )
		rectangle(frame, m_track_window, Scalar(255, 128, 128), 2);

		/********************************************************/

		traceRoute(frame, val != 0);

		center_of_rect = (m_track_window.br() + m_track_window.tl())*0.5;
		circle(frame, center_of_rect, 1, Scalar(0, 0, 255), 2);

		imshow("img2", frame);

		//savePic(&frame, save_index);
		//save_index++;

		// Affichage Back Projection
		//imshow("dst", dst);
	}
	return center_of_rect;
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
