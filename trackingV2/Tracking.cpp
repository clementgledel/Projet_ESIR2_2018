#include "Tracking.h"

Tracking::Tracking(string videoName)
{
	// Create a VideoCapture object and open the input file
	// If the input is the web camera, pass 0 instead of the video file name
	m_cap = VideoCapture(videoName);
	m_cap >> m_current_frame;

	Mat frame;
	// Condition video 2
	/*if (videoName.find("MVI_1189_trim_cormorant.mov") != -1)
	{
		for (int i = 0; i < 45; i++)
		{
			m_cap >> frame;
		}
	}*/
	cpt_reinit_histo = 0;
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
		inRange(hsv_roi, Scalar(140., 0., 0.), Scalar(255., 255., 150.), maskroi); // Créer un masque contenant 255 lorsque la valeur est dans l'interval
		//imwrite("mask_roi.jpg", maskroi);

		//savePic(&maskroi, 3);
		calcHist(&roi, 1, channels, maskroi, m_roi_hist, 2, histSize, ranges, true, false);
	}
	else 
	{
		calcHist(&roi, 1, channels, noArray(), m_roi_hist, 2, histSize, ranges, true, false);
	}

	normalize(m_roi_hist, m_roi_hist, 0, 255, NORM_MINMAX);
}

Point2f Tracking::searchTarget()
{
	Point2f oiseau = Point2f(m_current_frame.cols-1, m_current_frame.rows-1);
	int cpt=0;
	cout << oiseau.x;
	while (oiseau.x > m_current_frame.cols / 2)
	{
		m_current_frame = Mat();
		// Capture frame-by-frame
		m_cap >> m_current_frame;
		// If the frame is empty, break immediately
		if (m_current_frame.empty() || cpt > 10)
			return Point2f(-1,-1);

		cout << "a" << endl;
		m_geo.setPictures(m_current_frame);

		cout << "b" << endl;
		m_geo.DO();
		oiseau = m_geo.getOiseau();

		cout << "c" << endl;
		imshow("img2", m_current_frame);
		cpt++;
		cout << oiseau.x;
	}
	return oiseau;
}

void Tracking::initializeHistogram(int width, int height, const int channels[], const int histSize[], float range[], const float *ranges[])
{
	Point2f target = searchTarget();
	if (target.x != -1)
	{
		Mat hsv, dst;
		int k = 0;
		m_track_window = Rect(target.x- width /2, target.y - height /2, width, height) & Rect(0,0,m_current_frame.cols, m_current_frame.rows);

		/**********************************************************/
		/*******************Calcul de l'Histogramme****************/
		/**********************************************************/
		Mat pic = m_geo.getPicture();
		Mat roi = pic(m_track_window);
		//savePic(&roi, 1);

		// Calcul roi histogram
		calculHistogram(roi, channels, histSize, range, ranges, true);
		/**********************************************************/

		lastCenter = Point(m_track_window.x, m_track_window.y);

		/*
		* initialize the kalman filter
		*/
		double interval = 1.0 / m_cap.get(CV_CAP_PROP_FPS);
		initKalman(interval);

		isLost = false;
	}
	else
	{
		m_track_window = Rect(m_current_frame.cols / 2, m_current_frame.rows / 2, width, height) & Rect(0, 0, m_current_frame.cols, m_current_frame.rows);
		isLost = true;
	}
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

void Tracking::initKalman(double interval)
{
	const int stateNum = 4;
	const int measureNum = 2;

	Mat statePost = (Mat_<float>(stateNum, 1) << m_track_window.x + m_track_window.width / 2.0,
		m_track_window.y + m_track_window.height / 2.0,
		0, 0);
	Mat transitionMatrix = (Mat_<float>(stateNum, stateNum) << 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1);

	KF.init(stateNum, measureNum);

	KF.transitionMatrix = transitionMatrix;
	KF.statePost = statePost;
	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(1e-1));
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-3));
	setIdentity(KF.errorCovPost, Scalar::all(0.1));

	measurement = Mat::zeros(measureNum, 1, CV_32F);
}

Point Tracking::getCurrentState() const
{
	Mat statePost = KF.statePost;
	return Point(statePost.at<float>(0), statePost.at<float>(1));
}

void Tracking::setCurrentTrackWindow()
{
	int cols = m_current_frame.cols;
	int rows = m_current_frame.rows;

	m_track_window.x = KFCorrectCenter.x - m_track_window.width / 2;
	m_track_window.y = KFCorrectCenter.y - m_track_window.height / 2;

	//    trackWindow.x = MAX(0, trackWindow.x);
	//    trackWindow.x = MIN(cols, trackWindow.width);
	//    trackWindow.y = MAX(0, trackWindow.y);
	//    trackWindow.y = MIN(rows, trackWindow.height);
	//cout << m_track_window << endl;

	m_track_window &= Rect(0, 0, cols, rows);

	if (m_track_window.width <= 0 || m_track_window.height <= 0) {
		int width = MIN(KFCorrectCenter.x, cols - KFCorrectCenter.x) * 2;
		int height = MIN(KFCorrectCenter.y, rows - KFCorrectCenter.y) * 2;

		m_track_window = Rect(KFCorrectCenter.x - width / 2, KFCorrectCenter.y - height / 2, width, height);
	}
}

Point Tracking::camShiftTracking(const int channels[], const int histSize[], float range[], const float *ranges[])
{
	Mat hsv, dst;

	m_current_frame = Mat();
	m_cap >> m_current_frame;
	// Ajout m_geo
	m_geo.setPictures(m_current_frame);
	
	if (!m_current_frame.empty())
	{
		
		cvtColor(m_current_frame, hsv, COLOR_BGR2HSV);
		calcBackProject(&m_current_frame, 1, channels, m_roi_hist, dst, ranges);

		RotatedRect rot = CamShift(dst, m_track_window, TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 100, 0.01));

		m_center_of_rect = Point(m_track_window.x + m_track_window.width / 2, m_track_window.y + m_track_window.height / 2);

		/*
		* do kalman prediction
		*/
		KF.predict();
		KFPredictCenter = getCurrentState();

		/*
		* set measurement
		*/
		measurement.at<float>(0) = m_center_of_rect.x;
		measurement.at<float>(1) = m_center_of_rect.y;

		/*
		* do kalman correction
		*/
		KF.correct(measurement);
		KFCorrectCenter = getCurrentState();
		
		lastCenter = KFCorrectCenter;

		setCurrentTrackWindow();

		if (!isLost)
		{
			rectangle(m_current_frame, m_track_window, Scalar(255, 128, 128), 2);
			circle(m_current_frame, m_center_of_rect, 1, Scalar(0, 0, 255), 2);
		}

		// Affichage d'une ellipse représentant le rectangle rot renvoyé par le CamShift
		//if (rot.size.width > 0 && rot.size.height > 0) ellipse(m_current_frame, rot, Scalar(0, 0, 255), 2, LINE_AA);

		traceRoute(m_current_frame, rot.size.width > 0 && rot.size.height > 0);

		if (rot.size.width == 0 && rot.size.height == 0)
		{
			isLost = true;
			cpt_reinit_histo++;
			if (cpt_reinit_histo >= 20)
			{
				initializeHistogram(40, 40, channels, histSize, range, ranges);
				cpt_reinit_histo = 0;
			}
		}
		else {
			cpt_reinit_histo = 0;
			isLost = false;
		}

		imshow("img2", m_current_frame);

	}

	return m_center_of_rect;
}

Point Tracking::meanShiftTracking(const int channels[], const int histSize[], float range[], const float *ranges[])
{
	Mat hsv, dst;

	m_current_frame = Mat();
	m_cap >> m_current_frame;
	// Ajout m_geo
	m_geo.setPictures(m_current_frame);

	if (!m_current_frame.empty())
	{

		cvtColor(m_current_frame, hsv, COLOR_BGR2HSV);
		
		/*The functions calcBackProject calculate the back project of the histogram.
		That is, similarly to calcHist , at each location (x, y) the function collects the values from the selected channels in the input images and finds the corresponding histogram bin.
		But instead of incrementing it, the function reads the bin value, scales it by scale , and stores in backProject(x,y) .
		In terms of statistics, the function computes probability of each element value in respect with the empirical probability distribution represented by the histogram. */
		calcBackProject(&m_current_frame, 1, channels, m_roi_hist, dst, ranges);

		/***********************************************************/
		/************************Meanshift*************************/
		/**********************************************************/

		int val = meanShift(dst, m_track_window, TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 100, 0.01));
		

		m_center_of_rect = Point(m_track_window.x + m_track_window.width / 2, m_track_window.y + m_track_window.height / 2);

		/*
		* do kalman prediction
		*/
		KF.predict();
		KFPredictCenter = getCurrentState();

		/*
		* set measurement
		*/
		measurement.at<float>(0) = m_center_of_rect.x;
		measurement.at<float>(1) = m_center_of_rect.y;

		/*
		* do kalman correction
		*/
		KF.correct(measurement);
		KFCorrectCenter = getCurrentState();

		lastCenter = KFCorrectCenter;

		setCurrentTrackWindow();

		if (!isLost)
		{
			rectangle(m_current_frame, m_track_window, Scalar(255, 128, 128), 2);
			circle(m_current_frame, m_center_of_rect, 1, Scalar(0, 0, 255), 2);
		}
		
		/********************************************************/

		traceRoute(m_current_frame, val != 0);

		if (val == 0)
		{
			isLost = true;
			cpt_reinit_histo++;
			if (cpt_reinit_histo >= 20)
			{
				initializeHistogram(60, 60, channels, histSize, range, ranges);
				cpt_reinit_histo = 0;
			}
		}
		else {
			cpt_reinit_histo = 0;
			isLost = false;
		}

		imshow("img2", m_current_frame);

		//savePic(&frame, save_index);
		//save_index++;

		// Affichage Back Projection
		//imshow("dst", dst);
	}
	return m_center_of_rect;
}

