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
	int i =100;
	int j = i+1;
	int save = 200;
	std::ostringstream pic1Name;
	Mat pic1;
	pic1Name<<"C:/Users/Iwan/Desktop/PROJET Colibri/Projet_ESIR2_2018-master/build/save/"<<i<<".jpg";
	pic1 = imread(pic1Name.str(), IMREAD_COLOR); // Read the file
	 if( pic1.empty() )                      // Check for invalid input
    {
        cout <<  "Could not open or find the image picture 1" << std::endl ;
        return -1;
    }
	while (1){
	
	std::ostringstream pic2Name;
	
	Mat pic2;
	Mat picresult= pic1;
	
	pic2Name<<"C:/Users/Iwan/Desktop/PROJET Colibri/Projet_ESIR2_2018-master/build/save/"<<j<<".jpg";
	
	pic2 = imread(pic2Name.str(), IMREAD_COLOR); // Read the file
	
   
	if( pic2.empty() )                      // Check for invalid input
    {
        cout <<  "Could not open or find the image picture 2" << std::endl ;
        return -1;
    }
	
	picresult = (picresult+pic2)*0.5;
	savePic(&picresult,save);
	j++;
	}
	return 0;
}