#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <time.h>
#include <sstream> // for converting the command line parameter to integer
#include "std_msgs/Int16MultiArray.h"

using namespace std;
using namespace cv;


int main(int argc, char** argv)
{

  ros::init(argc, argv, "Jetson2");
  ros::NodeHandle nh;
  //image_transport::ImageTransport it(nh);
  //image_transport::Publisher pub = it.advertise("videoJ2", 1);
  ros::Publisher array_pub = nh.advertise<std_msgs::Int16MultiArray>("PointsJ2", 1000);
  std_msgs::Int16MultiArray points;


  cout << "\nStart...\n";
  VideoCapture cap(0);
  cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
  // Check if video device can be opened with the given index
  if(!cap.isOpened()){
    cout << "\n***problemo - camera will not open\n\n";
    return 1;
  }

  cout << "\nwe gucci!\n";
  //sensor_msgs::ImagePtr msg;

  Mat src, thresh, mixed_src, canny, dst, cdst, blueOnly, src2, doneFrame;

// Code for calculate fps
  int num_frames = 60; 
  time_t start, end;
  //cout << "Capturing " << num_frames << " frames" << endl ;
  int frames = 0;    

  ros::Rate loop_rate(1);
  while (nh.ok()) {
    //start time on first frame
    if(frames == 0)
      time(&start);
    frames++;

		cap >> src; // get a new frame from camera
		//imshow("Preview", src);


		Mat channel[3]; //color channel matrix

/*************************************IMAGE MANIPULATION**********************************************/
		/*process for removing unwanted objects*/
		inRange(src, Scalar(40, 0, 0), Scalar(255, 200, 150), blueOnly); //finding only blue barrels
		/*converting to BRG. This is done so that matrix subtraction can occur. When the in range 	  
		  function is performed, it becomes a single channel grayscale image. this will allow for 		
	 	 	subtracting images.*/
		cvtColor(blueOnly, blueOnly, CV_GRAY2BGR); 
		src2 = (src - blueOnly);

		/*splitting the masked image into three RGB color planes to increase contrast beween features
	 	  2*Blue - Green worked the best*/
		split(src2, channel);
		mixed_src = (2*channel[0])-channel[1];

		/*performing threshold to keep only the pixels that are part of the line*/
		threshold(mixed_src, thresh, 200, 255, 2);
		/*removing as much noise as possible, Kernel must be (odd,odd)*/
		blur(thresh, canny, Size(7,7)); 

		/*performing edge detection for hough transform and converting back to 8-channel BGR image*/
		Canny(canny, canny, 100, 200, 3);
		cvtColor(canny, cdst, CV_GRAY2BGR);

		/*performing the probableistic line fit*/
		vector<Vec4i> lines; //creating the vector that the fit will use

		/*performing the probablistic line fit. The traditional Hough transform
		  can be used, but this seems to work better as it deals with smaller and multiple
		  connecting segments which work better with non straight segments of the course*/
		HoughLinesP(canny, lines, 1, CV_PI/180, 50, 50, 20 );
               Mat final_frame(src.size(), CV_8UC3, Scalar(0)); //3-channel

                points.data.push_back(lines.size());
 
		/*fitting the line to the original image*/
		for( size_t i = 0; i < lines.size(); i++ ){
			Vec4i l = lines[i];
			line(src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0, 255), 3, CV_AA);
                        points.data.push_back(l[0]);
                        points.data.push_back(l[1]);
                        points.data.push_back(l[2]);
                        points.data.push_back(l[3]);
		}

/*****************************************************************************************************/

/*************Windows for testing purposes. They simply show the result of processing*****************/
			imshow("source", src);
                        
		//	moveWindow("source",0,600);
		//	imshow("barrel", blueOnly);
		//	imshow("Detected_Lines", cdst);
		//	moveWindow("Detected_Lines",600,0);
		//	imshow("mask", src2);
		//	imshow("Test Image", src);	
		//	imshow("Red", channel[2]);
		//	imshow("Green", channel[1]);
		//	imshow("Blue", channel[0]);
		//	imshow("MIXED-TEST_1", mixed_src);
		//	imshow("THRESH-TEST_2", thresh);
		//	imshow("CANNY_3", canny);
/*****************************************************************************************************/

    // Check if grabbed frame is actually full with some content
    if(!src.empty()) {
      //msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final_frame).toImageMsg();
      //pub.publish(msg);
      array_pub.publish(points);
      points.data.clear();
      if(frames == num_frames){
        // End Time
        time(&end);
        // Time elapsed
        double seconds = difftime (end, start);
        cout << "Time taken : " << seconds << " seconds" << endl;
        // Calculate frames per second
        int fps  = num_frames / seconds;
        cout << "Estimated frames per second : " << fps << endl;
      }
      cv::waitKey(1); 
    }

//    ros::spinOnce();
  }
}
