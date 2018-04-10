#include <iostream>
#include <sstream>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/sfm/fundamental.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <vector>
#include "CalibParam.h"

using namespace std;
using namespace cv;
using namespace xfeatures2d;
using namespace sfm;


class CalibInit{

  private:
    double m_t[3];
    Mat m_R;
    Mat essential_matrix;
    bool m_isCalibDone;
    Mat m_CalibR;
    Mat m_CalibT;
    double m_compactR1[3];

    void DLTTriangulation(
           const Mat & R,
           const Mat & pt,
           const Mat & py1,
           const Mat & py2,
           Mat &x)
   {
    x = cv::Mat::zeros(3, 1, CV_64F);

    cv::Mat A = cv::Mat::zeros(4, 4, CV_64F);
    cv::Mat P1 = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat P2 = cv::Mat::zeros(3, 4, CV_64F);

    for (int i = 0; i < 3; i++) {
         P1.at<double>(i, i) = 1.0;
         for (int j = 0; j < 3; j++) {
             P2.at<double>(i, j) = R.at<double>(i, j);
         }
         P2.at<double>(i, 3) = pt.at<double>(i, 0);
    }

    double x1 = py1.at<double>(0, 0);
    double y1 = py1.at<double>(1, 0);
    double x2 = py2.at<double>(0, 0);
    double y2 = py2.at<double>(1, 0);

    for (int j = 0; j < 4; j++) {
         A.at<double>(0, j) = x1*P1.at<double>(2, j) - P1.at<double>(0, j);
         A.at<double>(1, j) = y1*P1.at<double>(2, j) - P1.at<double>(1, j);
         A.at<double>(2, j) = x2*P2.at<double>(2, j) - P2.at<double>(0, j);
         A.at<double>(3, j) = y2*P2.at<double>(2, j) - P2.at<double>(1, j);
    }

    cv::SVD A_SVD(A, cv::SVD::FULL_UV);

    x.at<double>(0, 0) = A_SVD.vt.at<double>(3, 0) / A_SVD.vt.at<double>(3, 3);
    x.at<double>(1, 0) = A_SVD.vt.at<double>(3, 1) / A_SVD.vt.at<double>(3, 3);
    x.at<double>(2, 0) = A_SVD.vt.at<double>(3, 2) / A_SVD.vt.at<double>(3, 3);
   }

    bool doCalibration(std::vector<Point2f> points1, std::vector<Point2f> points2){
      CalibParam* solutions = new CalibParam[4];
      cv::SVD essentialMatSvd(essential_matrix, cv::SVD::FULL_UV);

      // median singular value as constraint
      double singularMoy = (essentialMatSvd.w.at<double>(0, 0) + essentialMatSvd.w.at<double>(1, 0)) / 2.0;

      cv::Mat S(3, 3, CV_64F, 0.0);
      S.at<double>(0, 0) = singularMoy;// fundMatSvd.w.at<double>(0,0);
      S.at<double>(1, 1) = singularMoy;//  fundMatSvd.w.at<double>(1,0);
      S.at<double>(2, 2) = 0.0;// = fundMatSvd.w.at<double>(2,0);

      cv::Mat essential_matrix_estim = essentialMatSvd.u * S * essentialMatSvd.vt;
      cv::SVD essentialMatEstimSvd(essential_matrix_estim, cv::SVD::FULL_UV);

      // now we proceed to SR factorization of E
      // usefull matrix
      cv::Mat W(3, 3, CV_64F, 0.0);
      W.at<double>(0, 1) = -1.0;  //     | 0 -1  0 |
      W.at<double>(1, 0) = 1.0;  // W = | 1  0  0 |
      W.at<double>(2, 2) = 1.0;  //     | 0  0  1 |

      cv::Mat Z(3, 3, CV_64F, 0.0);
      Z.at<double>(0, 1) = 1.0;   //     | 0  1  0 |
      Z.at<double>(1, 0) = -1.0;  // Z = |-1  0  0 |
                                    //     | 0  0  0 |

      // get the 2 possible rotations
      cv::Mat Rot1 = essentialMatEstimSvd.u * W *  essentialMatEstimSvd.vt;
      cv::transpose(W, W);
      cv::Mat Rot2 = essentialMatEstimSvd.u * W *  essentialMatEstimSvd.vt;
      cv::transpose(W, W); // get back W in the good shape

      double detR1 = cv::determinant(Rot1);
      double detR2 = cv::determinant(Rot2);

      // if detR1 and detR2 = -1, we have to negate the essential matrix and recompute it
      if (detR1 < 0 || detR2<0)
      {
          cv::SVD essentialMatEstimSvd2(-essential_matrix_estim, cv::SVD::FULL_UV);
          Rot1 = essentialMatEstimSvd2.u * W *  essentialMatEstimSvd2.vt;
          cv::transpose(W, W);
          Rot2 = essentialMatEstimSvd2.u * W *  essentialMatEstimSvd2.vt;
          cv::transpose(W, W);
          detR1 = cv::determinant(Rot1);
          detR2 = cv::determinant(Rot2);
      }
      //        ASSERT(detR1>0 && detR2>0);
      // get the translation vector as the last column of U
      cv::Mat t(3, 1, CV_64F, 0.0);
      t.at<double>(0, 0) = essentialMatEstimSvd.u.at<double>(0, 2);
      t.at<double>(1, 0) = essentialMatEstimSvd.u.at<double>(1, 2);
      t.at<double>(2, 0) = essentialMatEstimSvd.u.at<double>(2, 2);

      // extract the good solution among the 4 possibles (R1|R2, +/-t)
      // the four possible solutions
      solutions[0].R = Rot1;        solutions[0].t = t;
      solutions[1].R = Rot1;        solutions[1].t = -t;
      solutions[2].R = Rot2;        solutions[2].t = t;
      solutions[3].R = Rot2;        solutions[3].t = -t;

      // test the four possibdouble sol[3]utions against one point
      int goodSolIndex = -1;
      for (int i = 0; i < 4; i++)
      {
          // solution of the triangulation
          Mat X(3, 1, CV_64F, 0.0);
          // the 2 point in the 2 image planes
          Mat y1(2, 1, CV_64F, 0.0);
          Mat y2(2, 1, CV_64F, 0.0);

          bool thisIsTheGoodSol = true;
          for (unsigned int j = 0; j<points1.size(); j++)
          {
              y1.at<double>(0, 0) = points1[j].x;
              y1.at<double>(1, 0) = points1[j].y;
              y2.at<double>(0, 0) = points2[j].x;
              y2.at<double>(1, 0) = points2[j].y;

              DLTTriangulation(
                  solutions[i].R, solutions[i].t,
                  y1, y2,
                  X);

              // point in front of the 1st cam, look good
              if (X.at<double>(2, 0)>0.0)
              {
                  cv::Mat rTranspose(3, 3, CV_64F, 0.0);
                  cv::transpose(solutions[i].R, rTranspose);
                  // test if the point is in front of the 2nd camera
                  cv::Mat XinC2 = (rTranspose*(X - solutions[i].t));

                  // is the point in front of the 2nd cam ?
                  if (XinC2.at<double>(2, 0) > 0.0)
                  {
                      // yes it is
                  }
                  else
                      thisIsTheGoodSol = false;
              }
              else
                  thisIsTheGoodSol = false;
              }// end  test against collected points
              if (thisIsTheGoodSol)
                goodSolIndex = i;
        }

        // end search good sol
        for (int i = 0; i < 4; i++)
        {
          // prepare for opengl draw
          cv::Mat compactR1;
          cv::Rodrigues(solutions[i].R, compactR1);

          solutions[i].m_compactR[0] = compactR1.at<double>(0, 0);
          solutions[i].m_compactR[1] = compactR1.at<double>(1, 0);
          solutions[i].m_compactR[2] = compactR1.at<double>(2, 0);

          solutions[i].m_t[0] = solutions[i].t.at<double>(0, 0);
          solutions[i].m_t[1] = solutions[i].t.at<double>(1, 0);
          solutions[i].m_t[2] = solutions[i].t.at<double>(2, 0);
        }



        //        goodSolIndex = 0;
        //cout<<goodSolIndex<<endl;
        if (goodSolIndex != -1)
        {
          m_CalibR = solutions[goodSolIndex].R.clone();
          m_CalibT = solutions[goodSolIndex].t.clone();
          //cout<<solutions[goodSolIndex].R<<endl;

          m_compactR1[0] = solutions[goodSolIndex].m_compactR[0];
          m_compactR1[1] = solutions[goodSolIndex].m_compactR[1];
          m_compactR1[2] = solutions[goodSolIndex].m_compactR[2];

          m_t[0] = solutions[goodSolIndex].t.at<double>(0, 0);
          m_t[1] = solutions[goodSolIndex].t.at<double>(1, 0);
          m_t[2] = solutions[goodSolIndex].t.at<double>(2, 0);

          m_isCalibDone = true;
      }

      m_isCalibDone = true;

      return m_isCalibDone;
  }

  public:
    CalibInit():
    m_t(),m_R()
    {}

    ~CalibInit(){}

    void init(Mat * im){
      int rows = im->rows;
      int cols = im->cols;
      int ncols = cols/2;

      // SIFTs
      Ptr<SIFT> st_left = SIFT::create();
      vector<KeyPoint> kp_left;
      Ptr<SIFT> st_right = SIFT::create();
      vector<KeyPoint> kp_right;

      Mat frame_left(rows,ncols,im->type());
      Mat frame_right(rows,ncols,im->type());

      // Split picture in 2
      Mat mask_image( im->size(), CV_8U, Scalar(0));
      for(int i(0); i<rows;i++){
        for(int j(0); j<ncols;j++){
          frame_left.at<Vec3b>(Point(j,i)) = im->at<Vec3b>(Point(j,i));
          frame_right.at<Vec3b>(Point(j,i)) = im->at<Vec3b>(Point(j+ncols,i));
        }
      }
      //imshow("left",frame_left);
      //imshow("right",frame_right);

      st_left->detect(frame_left,kp_left);
      st_right->detect(frame_right,kp_right);

      Mat descriptors_left;
      st_left->compute(frame_left, kp_left, descriptors_left);
      Mat descriptors_right;
      st_right->compute(frame_right, kp_right, descriptors_right);
      /*Mat outputImage1;
      Scalar keypointColor = Scalar(255, 0, 0);     // Blue keypoints.
      drawKeypoints(frame_left, kp_left, outputImage1, keypointColor, DrawMatchesFlags::DEFAULT);
      keypointColor = Scalar(0, 0, 255);     // Red keypoints.
      Mat outputImage;
      drawKeypoints(frame_right, kp_right, outputImage, keypointColor, DrawMatchesFlags::DEFAULT);
      outputImage+=outputImage1;*/

      Mat image;
      vector<DMatch> matches1to2;
      vector< vector<DMatch> > matches;
      BFMatcher matcher;
      matcher.knnMatch(descriptors_left, descriptors_right, matches, 8);
      //matcher.radiusMatch(descriptors_left, descriptors_right, matches, 300);
      matches1to2.reserve(matches.size());
      double tresholdDist = 0.25 * sqrt(double(frame_left.size().height*frame_left.size().height + frame_left.size().width*frame_left.size().width));


      // Supposed Ransac
      for (size_t i = 0; i < matches.size(); ++i)
      {
         for (unsigned int j = 0; j < matches[i].size(); j++)
         {
             Point2f from = kp_left[matches[i][j].queryIdx].pt;
             Point2f to = kp_right[matches[i][j].trainIdx].pt;

             //calculate local distance for each possible match
             double dist = sqrt((from.x - to.x) * (from.x - to.x) + (from.y - to.y) * (from.y - to.y));

             //save as best match if local distance is in specified area and on same height
             if (dist < tresholdDist && abs(from.y-to.y)<5)
             {
                 matches1to2.push_back(matches[i][j]);
                 j = matches[i].size();
             }
         }
      }
      drawMatches(frame_left, kp_left, frame_right,kp_right,matches1to2, image, Scalar::all(-1), Scalar::all(0));

      resize(image,image,Size(800,400));
      //imshow("Test",image);
      //cvWaitKey();
      imwrite("../Calibration.jpg",image);
      std::vector<Point2f> points1;
      std::vector<Point2f> points2;
      Mat fund_matrice;
      //cout<<"Taille des matches "<<matches1to2.size()<<endl;
      for(unsigned int i(0);i<matches1to2.size();i++){
         //cout<<matches1to2.at(i).queryIdx<<"\t"<<matches1to2.at(i).trainIdx<<endl;
         points1.push_back(kp_left.at(matches1to2.at(i).queryIdx).pt);
         points2.push_back(kp_right.at(matches1to2.at(i).trainIdx).pt);
      }
      essential_matrix = findFundamentalMat(points1,points2, cv::FM_8POINT,5,0.95);
      cout<<"====ESSENTIAL===="<<endl;
      cout<<essential_matrix<<endl;
      cout<<doCalibration(points1, points2)<<endl;
    }

    Mat getR(){return m_CalibR;}
    Mat getT(){return m_CalibT;}
    Mat getMatrix(){return essential_matrix;}

};
