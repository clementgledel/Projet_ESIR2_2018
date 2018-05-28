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

	Mat motion(Mat & pic1, Mat & pic2) {
		Mat picresult;
		Mat diffImage;
		absdiff(pic2, pic1, picresult);
		//absdiff(backgroundImage, currentImage, diffImage);

		Mat foregroundMask = Mat::zeros(picresult.rows, picresult.cols, CV_8UC1);

		float threshold = 20.0f;

		cvtColor(picresult, picresult, COLOR_RGB2GRAY);
		for (int k = 0; k<picresult.rows; ++k) {
			for (int l = 0; l<picresult.cols; ++l)
			{
				uchar pix = picresult.at<uchar>(k, l);
				if (pix>threshold)
				{
					foregroundMask.at<unsigned char>(k, l) = 255;
				}
			}
		}


		//threshold(picresult, picresult, 80, 255, THRESH_BINARY);
		//cvtColor(picresult, picresult, COLOR_RGB2GRAY);
		//GaussianBlur(picresult,picresult,Size_<float>::Size_(7,7),0);

		//enlever le bruit
		morphologyEx(foregroundMask, foregroundMask, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(3, 3)));

		return foregroundMask;
	}

	void mean() {
		cv::Mat foregroundMask = cv::Mat::zeros(m_motion1.rows, m_motion1.cols, CV_8UC1);

		for (int k = 0; k<m_motion1.rows; ++k) {
			for (int l = 0; l<m_motion1.cols; ++l)
			{
				uchar pix1 = m_motion1.at<uchar>(k, l);
				uchar pix2 = m_motion2.at<uchar>(k, l);
				if ((pix2 > 200) && (pix1 > 200))
				{
					foregroundMask.at<unsigned char>(k, l) = 255;
				}
			}
		}

		// une seule composante pour l'oiseau
		morphologyEx(foregroundMask, foregroundMask, MORPH_CLOSE, getStructuringElement(MORPH_RECT, Size(25, 25)));

		vector<vector<Point> > contours; // image contour is set of salient points of the image we are interested in
		vector<Vec4i> hierarchy;
		Mat canny_output;
		//Detect edges using canny
		Canny(foregroundMask, canny_output, 50, 150, 3); // canny edges are low-level image structures that are used by contour detector
														 //Find contours
		findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		// Get the moments of image
		vector<Moments> mu(contours.size());
		for (unsigned int i = 0; i < contours.size(); i++)
		{
			mu[i] = moments(contours[i], false);
		}
		//Get the mass centers (image has multiple contours):
		vector<Point2f> mc(contours.size());
		for (unsigned int i = 0; i < contours.size(); i++)
		{
			mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
			circle(foregroundMask, mc[i], 2, Scalar(128, 0, 0), 2);
		} // compute the centers of mass of each contour in the image

		m_result = foregroundMask;
		m_points = mc;
		cout << m_points << endl;



		// Determiner le point correspondant à l'oiseau (image gauche)
		oiseauGauche = Point2f(m_picture.cols, m_picture.rows);
		if (m_points.size() < 5)
		{
			for (int i = 0; i<m_points.size(); i++) {
				if (m_points[i].x < oiseauGauche.x && !isnan(m_points[i].x) && !isnan(m_points[i].y)) {
					// on garde le point le plus à gauche
					oiseauGauche = m_points[i];
				}
			}
		}
		
		if (oiseauGauche.x < m_picture.cols / 2) {
			cout << "Point retenu : " << oiseauGauche << endl << endl;
		}
		//cv::waitKey();

	}

public:

	GeoCentre() :
		m_previous(), m_picture(), m_next(), m_motion1(), m_motion2(), m_result(), m_points()
	{
		oiseauGauche = Point2f(1919, 1087);
	}

	void DO() {
		if (m_previous.empty() || m_picture.empty() || m_next.empty()) {
			cout << "Pas prêt" << endl;
		}
		else {
			m_motion1 = motion(m_picture, m_previous);
			m_motion2 = motion(m_next, m_picture);

			mean();

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

	vector<Point2f> getCoords() {
		return m_points;
	}
};
