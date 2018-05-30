#include <iostream>
#include <sstream>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <vector>

using namespace std;
using namespace cv;
using namespace xfeatures2d;

class GeoCentre
{
private:

	Mat m_previous;
	Mat m_picture;
	Mat m_next;

	Mat m_motion1;
	Mat m_motion2;

	Mat m_result;

	vector<Point2f> m_points;
	Point2f oiseauGauche;

	// Calcule une image binaire de différence entre pic1 et pic2
	Mat motion(Mat & pic1, Mat & pic2) {
		Mat picresult;
		
		// Différence absolue
		absdiff(pic2, pic1, picresult);
		
		cvtColor(picresult, picresult, COLOR_RGB2GRAY);

		Mat foregroundMask = Mat::zeros(picresult.rows, picresult.cols, CV_8UC1);
		
		// Seuillage pour obenir une image binaire	
		float threshold = 20.0f;
		for (int k = 0; k < picresult.rows; ++k) {
			for (int l = 0; l < picresult.cols; ++l) {
				uchar pix = picresult.at<uchar>(k, l);
				if (pix > threshold) {
					foregroundMask.at<unsigned char>(k, l) = 255;
				}
			}
		}

		// Elimination du bruit
		morphologyEx(foregroundMask, foregroundMask, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(3, 3)));

		return foregroundMask;
	}

	
	// Calcul des centroides des composantes connexes et sélection de celui de l'oiseau dans l'image de gauche
	void massCenters() {
		cv::Mat foregroundMask = cv::Mat::zeros(m_motion1.rows, m_motion1.cols, CV_8UC1);
		
		// Conservation des zones en mouvement de l'image m_picture à partir des différences avec les images précédentes et suivantes
		for (int k = 0; k < m_motion1.rows; ++k) {
			for (int l = 0; l < m_motion1.cols; ++l) {
				uchar pix1 = m_motion1.at<uchar>(k, l);
				uchar pix2 = m_motion2.at<uchar>(k, l);
				if ((pix2 > 200) && (pix1 > 200)) {
					foregroundMask.at<unsigned char>(k, l) = 255;
				}
			}
		}

		// Fermeture pour rassembler les différentes composantes correspondant à un meme oiseau
		morphologyEx(foregroundMask, foregroundMask, MORPH_CLOSE, getStructuringElement(MORPH_RECT, Size(25, 25)));

		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		Mat canny_output;
		
		// Detect edges using canny
		Canny(foregroundMask, canny_output, 50, 150, 3);
		
		// Find contours
		findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		
		// Get the moments of image
		vector<Moments> mu(contours.size());
		for (unsigned int i = 0; i < contours.size(); i++) {
			mu[i] = moments(contours[i], false);
		}
		
		// Get the mass centers (image has multiple contours):
		vector<Point2f> mc(contours.size());
		for (unsigned int i = 0; i < contours.size(); i++) {
			// Compute the centers of mass of each contour in the image
			mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
			circle(foregroundMask, mc[i], 2, Scalar(128, 0, 0), 2);
		}

		m_result = foregroundMask;
		m_points = mc;
		cout << m_points << endl;

		// Determiner le point correspondant à l'oiseau (image gauche)
		oiseauGauche = Point2f(m_picture.cols, m_picture.rows);
		if (m_points.size() < 5) {
			for (int i = 0; i<m_points.size(); i++) {
				// on garde le point le plus à gauche
				if (m_points[i].x < oiseauGauche.x && !isnan(m_points[i].x) && !isnan(m_points[i].y)) {
					oiseauGauche = m_points[i];
				}
			}
		}
		
		/*
		// Test image par image
		if (oiseauGauche.x < m_picture.cols / 2) {
			cout << "Point retenu : " << oiseauGauche << endl << endl;
		}
		cv::waitKey();
		*/
	}

public:

	GeoCentre() :
		m_previous(), m_picture(), m_next(), m_motion1(), m_motion2(), m_result(), m_points()
	{
		oiseauGauche = Point2f(1919, 1087); // Point situé dans l'image de droite
	}

	void compute() {
		if (m_previous.empty() || m_picture.empty() || m_next.empty()) {
			cout << "Pas prêt" << endl;
		}
		else {
			// Différences entre les images successives
			m_motion1 = motion(m_picture, m_previous);
			m_motion2 = motion(m_next, m_picture);
			
			// Calcul du centroide de l'oiseau dans l'image de gauche
			massCenters();

			resize(m_result, m_result, Size(800, 400));
		}
	}

	void setPicture(Mat & m) { m_picture = m; }
	
	void setPrevious(Mat & m) { m_previous = m; }
	
	void setNext(Mat & m) { m_next = m; }

	void setPictures(Mat & m) {
		setPrevious(m_picture);
		setPicture(m_next);
		setNext(m);
	}

	Mat * getResult() { return &m_result; }

	Mat getPicture() { return m_picture; }
	
	Point2f getOiseau() { return oiseauGauche; }

	vector<Point2f> getCoords() { return m_points; }
};
