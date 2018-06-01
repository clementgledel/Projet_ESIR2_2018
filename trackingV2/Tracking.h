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
	// Tracking
	VideoCapture m_cap;
	Rect m_track_window;
	Mat m_current_frame;

	// Histogramme
	Mat m_roi_hist;

	// Classe de d�tection de l'oiseau dans l'image
	GeoCentre m_geo;

	// Trac� de la position
	std::deque<Point> m_li_center;
	std::deque<Point> m_li_center_afterlost;

	// Compteur jusqu'� refaire un nouvel histogramme
	int cpt_reinit_histo;

	// Filtre de Kalman
	KalmanFilter m_KF;
	Mat_<float> m_measurement;
	Point m_KFPredictCenter;
	Point m_KFCorrectCenter;
	Point m_center_of_rect;

	bool m_isLost;
	
	// M�thode d'initialisation de l'histogramme
	void calculHistogram(Mat &roi, const int channels[], const int histSize[], float range[], const float *ranges[], bool usemask = false);

	// Trac� de la position
	void traceRoute(Mat& frame, bool foundTarget);

	// Filtre de Kalman
	void initKalman();
	Point getCurrentKalmanState() const;
	void setCurrentKalmanTrackWindow();

public:
	Tracking(string videoName);
	~Tracking();

	// M�thode de sauvegarde d'image
	void savePic(Mat * im, int i); 

	// M�thode d'initialisation de l'histogramme
	void initializeHistogram(int width, int height, const int channels[], const int histSize[], float range[], const float *ranges[]);

	// M�thode permettant de trouv� l'oiseau dans l'image
	Point2f searchTarget();

	// M�thode tracking
	Point camShiftTracking(const int channels[], const int histSize[], float range[], const float *ranges[]);
	Point meanShiftTracking(const int channels[], const int histSize[], float range[], const float *ranges[]);

};

