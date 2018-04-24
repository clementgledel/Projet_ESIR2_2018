#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/plot.hpp>

using namespace std;
using namespace cv;
using namespace plot;

class MainWindow{

private:
  Mat m_video;
  Mat m_3DGraph;
  Mat m_XGraph;
  Mat m_YGraph;
  Mat m_mainPicture;



  // Stick pictures together
  void stick(){
    Mat up;
    Mat down;

    resize(m_video,m_video,Size(400,200));
    resize(m_3DGraph,m_3DGraph,Size(400,200));
    resize(m_XGraph,m_XGraph,Size(400,200));
    resize(m_YGraph,m_YGraph,Size(400,200));

    hconcat(m_video,m_3DGraph,up);
    hconcat(m_XGraph,m_YGraph,down);
    vconcat(up,down,m_mainPicture);
  }

public:
  MainWindow():m_video(),m_3DGraph(),m_XGraph(),m_YGraph(),m_mainPicture()
  {
    m_video = imread("../save/1.jpg");
    m_3DGraph = imread("../save/1.jpg");
    m_XGraph = imread("../save/1.jpg");
    m_YGraph = imread("../save/1.jpg");

    //hconcat(m_video,m_3DGraph,m_mainPicture);
    //vconcat(m_video,m_3DGraph,m_mainPicture);

    //imshow("Test",m_mainPicture);
    //cvWaitKey();
    refresh();
  }

  //refresh
  void refresh(){
    stick();
    imshow("Test",m_mainPicture);

  }

  // Setters
  void setVideo(Mat & video){m_video = video;}
  void setXGraph(Ptr<Plot2d> plot){plot->render(m_XGraph);}
  void setYGraph(Ptr<Plot2d> plot){plot->render(m_YGraph);}
};
