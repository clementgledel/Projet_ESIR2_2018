#include <iostream>
#include <sstream>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/sfm/fundamental.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <vector>
#include "CalibInit.h"
#include "MainWindow.h"
#include "GeoCentre.h"

using namespace std;
using namespace cv;
using namespace xfeatures2d;
using namespace sfm;

// save a picture, the name of the picture is its frame position in the video
void savePic(Mat * im, int i){
  std::ostringstream oss;
  oss<<"../saveHD/" <<i<<".jpg";
  imwrite(oss.str(), *im);
}
/*
int main(){
  string videoName;
  MainWindow m_window;
  std::cout << "Video path : ";
//  std::cin >> videoName;
  videoName = "../../MVI_1111_trim_pie.mov";
  // Create a VideoCapture object and open the input   file
  // If the input is the web camera, pass 0 instead of the video file name
  VideoCapture cap(videoName);

  // Check if camera opened successfully
  if(!cap.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }
  int i = 0;
  Mat frame;
  cap>>frame;
  Mat fund_mat;
  CalibInit init;
  init.init(&frame);
  cout<<init.getT()<<endl;
  cout<<init.getR()<<endl;
  while(1){

      //Mat frame;
      // Capture frame-by-frame
      cap >> frame;

      // If the frame is empty, break immediately
      if (frame.empty())
        break;

      m_window.setVideo(frame);
      m_window.refresh();
    //imshow("Output", outputImage);


      // Display the resulting frame
      //imshow( "Frame", frame);
   	  i++;
      //savePic(&outputImage,i);
      //cout<< "Save picture : "<<i<<endl;

      // Press  ESC on keyboard to exit
      char c=(char)waitKey(25);
      if(c==27)
        break;
  }

  // When everything done, release the video capture object
  cap.release();

  // Closes all the frames
  //destroyAllWindows();

  return 0;
}
*/

int main(int argc, char const *argv[]) {
  //Image 20 cm
  Mat im = imread("../I3_20.jpg");
  CalibInit calib;
  calib.init(&im);
  cout<<"===TRANSLATION"<<endl;
  cout<<calib.getT()<<endl;
  cout<<"===ROTATION==="<<endl;
  cout<<calib.getR()<<endl;
  // Image Zoom
  im = imread("../I3_zoom.jpg");
  calib.init(&im);
  cout<<"===TRANSLATION"<<endl;
  cout<<calib.getT()<<endl;
  cout<<"===ROTATION==="<<endl;
  cout<<calib.getR()<<endl;
  // Image avec rotation
  im = imread("../I3.jpg");
  calib.init(&im);
  cout<<"===TRANSLATION"<<endl;
  cout<<calib.getT()<<endl;
  cout<<"===ROTATION==="<<endl;
  cout<<calib.getR()<<endl;
  return 0;
}
/*
int main(){
  GeoCentre geo;
  Mat * res;
  string videoName;
//  std::cin >> videoName;
  videoName = "../../MVI_1189_trim_cormorant.mov";
  // Create a VideoCapture object and open the input file
  // If the input is the web camera, pass 0 instead of the video file name
  VideoCapture cap(videoName);

  // Check if camera opened successfully
  if(!cap.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }
  while(1){

    Mat frame;
    // Capture frame-by-frame
    cap >> frame;
    // If the frame is empty, break immediately
    if (frame.empty())
      break;
    geo.setPictures(frame);
    geo.DO();
    res = geo.getResult();
    if(!res->empty()){
      namedWindow("Output");
      imshow("Output", *res);
    }
  //namedWindow("Output");
  //imshow("Output", outputImage);


    // Press  ESC on keyboard to exit
    char c=(char)waitKey(25);
    if(c==27)
      break;
  }

  // When everything done, release the video capture object
  cap.release();

  // Closes all the frames
  destroyAllWindows();

  return 0;
}*/
