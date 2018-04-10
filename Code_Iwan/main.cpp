#include <iostream>
#include <sstream>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <vector>

using namespace std;
using namespace cv;
using namespace xfeatures2d;

// save a picture, the name of the picture is its frame position in the video
void savePic(Mat * im, int i){
  std::ostringstream oss;
  oss<<"../save/" <<i<<".jpg";
  imwrite(oss.str(), *im);
}


//

int main(){
  Ptr<SIFT> st = SIFT::create();
  vector<KeyPoint> keypoints;
  string videoName;
  std::cout << "Video path : ";
//  std::cin >> videoName;
  videoName = "G:/Videos RSV Irisa/MVI_1111_trim_pie.mov";
  // Create a VideoCapture object and open the input file
  // If the input is the web camera, pass 0 instead of the video file name
  VideoCapture cap(videoName);

  // Check if camera opened successfully
  if(!cap.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }
     int i = 0;
  while(1){

    Mat frame;
    // Capture frame-by-frame
    cap >> frame;

    // If the frame is empty, break immediately
    if (frame.empty())
      break;

    st->detect(frame,keypoints);

    //Similarly, we create a smart pointer to the SIFT extractor.
  //Ptr<DescriptorExtractor> featureExtractor = DescriptorExtractor::create("SIFT");

  // Compute the 128 dimension SIFT descriptor at each keypoint.
  // Each row in "descriptors" correspond to the SIFT descriptor for each keypoint
  Mat descriptors;
  st->compute(frame, keypoints, descriptors);

  // If you would like to draw the detected keypoint just to check
  Mat outputImage;
  Scalar keypointColor = Scalar(255, 0, 0);     // Blue keypoints.
  drawKeypoints(frame, keypoints, outputImage, keypointColor, DrawMatchesFlags::DEFAULT);

  //namedWindow("Output");
  //imshow("Output", outputImage);


    // Display the resulting frame
    //imshow( "Frame", frame);
 	  i++;
    savePic(&frame,i);
    cout<< "Save picture : "<<i<<endl;
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
}