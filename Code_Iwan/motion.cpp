#include <iostream>
#include <sstream>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <vector>

using namespace std;
using namespace cv;
using namespace xfeatures2d;


//cmake --build . --config Release
// save a picture, the name of the picture is its frame position in the video
void savePic(Mat * im, int i){
  std::ostringstream oss;
  oss<<"../save/" <<i<<".jpg";
  imwrite(oss.str(), *im);
}

int main(){
	int i =1;
	int j = i+1;
	int save = 100;
	while (1){
	std::ostringstream pic1Name;
	std::ostringstream pic2Name;
	Mat pic1;
	Mat pic2;
	Mat picresult;
	pic1Name<<"C:/Users/Iwan/Desktop/PROJET Colibri/Projet_ESIR2_2018-master/build/save/"<<i<<".jpg";
	pic2Name<<"C:/Users/Iwan/Desktop/PROJET Colibri/Projet_ESIR2_2018-master/build/save/"<<j<<".jpg";
	pic1 = imread(pic1Name.str(), IMREAD_COLOR); // Read the file
	pic2 = imread(pic2Name.str(), IMREAD_COLOR); // Read the file
	
    if( pic1.empty() )                      // Check for invalid input
    {
        cout <<  "Could not open or find the image picture 1" << std::endl ;
        return -1;
    }
	if( pic2.empty() )                      // Check for invalid input
    {
        cout <<  "Could not open or find the image picture 2" << std::endl ;
        return -1;
    }
	
	cv::absdiff(pic2, pic1, picresult);
	cv::Mat diffImage;
    //cv::absdiff(backgroundImage, currentImage, diffImage);
	
    cv::Mat foregroundMask = cv::Mat::zeros(picresult.rows, picresult.cols, CV_8UC1);

    float threshold = 50.0f;
    float dist;

    for(int j=0; j<picresult.rows; ++j){
        for(int i=0; i<picresult.cols; ++i)
        {
            cv::Vec3b pix = picresult.at<cv::Vec3b>(j,i);

            dist = (float)(pix[0]*pix[0] + pix[1]*pix[1] + pix[2]*pix[2]);
            dist = sqrt(dist);

            if(dist>threshold)
            {
                foregroundMask.at<unsigned char>(j,i) = 255;
            }
        }
	}
	
	//cv::threshold(picresult, picresult, 80, 255, cv::THRESH_BINARY);
	//cvtColor(picresult, picresult, cv::COLOR_RGB2GRAY);
	//cv::GaussianBlur(picresult,picresult,cv::Size_<float>::Size_(7,7),0);
	cv::erode(foregroundMask, foregroundMask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7,7)));
	cv::dilate(foregroundMask, foregroundMask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11,11)));
	savePic(&foregroundMask,save);
	save++;i++;j++;
	}
	return 0;
	}