#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/legacy/legacy.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cmath>


using namespace std;
cv::Mat gris;
cv::Mat_<float> H(3, 3);
std::vector<cv::Point> contours;
std::vector<cv::Point> contours_pro;
std::vector<cv::Point> line;
cv_bridge::CvImagePtr cv_ptr;
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
         cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
         cv::cvtColor(cv_ptr->image, gris, CV_BGR2GRAY);
         cv::Mat res(gris.rows,gris.cols,CV_8UC1,cv::Scalar::all(0));
         int nl = gris.rows;
         int nc = gris.cols * gris.channels();
         
         
        for (int j=0; j<nl;j++)
    {
     uchar* data = gris.ptr<uchar>(j);
     uchar* datagr = res.ptr<uchar>(j);
          for (int i=0; i<nc;i++)
          {
            if (data[i] > 100 && data[i] < 170)
               {
                datagr[i] = 255;
                contours.push_back(cv::Point((i),j));
               }
              
          }
          
    
    }
    

               

		cv::Size image_size = gris.size();
		double w = (double)image_size.width, h = (double)image_size.height;


		// Projecion matrix 2D -> 3D
		cv::Mat A1 = (cv::Mat_<double>(4, 3)<< 
			1, 0, -3.1872821392192793e+02,
			0, 1, -2.4699050318932325e+02,
			0, 0, 0,
			0, 0, 1 );
			
                   double alpha = -1.5, beta = 0, gamma = 0;  
	int  dist = 256;
			


		// Rotation matrices Rx, Ry, Rz

		cv::Mat RX = (cv::Mat_<double>(4, 4) << 
			1, 0, 0, 0,
			0, cos(double(alpha)), -sin(double(alpha)), 0,
			0, sin(double(alpha)), cos(double(alpha)), 0,
			0, 0, 0, 1 );

		cv::Mat RY = (cv::Mat_<double>(4, 4) << 
			cos(double(beta)), 0, -sin(double(beta)), 0,
			0, 1, 0, 0,
			sin(double(beta)), 0, cos(double(beta)), 0,
			0, 0, 0, 1	);

		cv::Mat RZ = (cv::Mat_<double>(4, 4) << 
			cos(double(gamma)), -sin(double(gamma)), 0, 0,
			sin(double(gamma)), cos(gamma), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1	);


		cv::Mat R = RX * RY * RZ;
		

		cv::Mat T = (cv::Mat_<double>(4, 4) << 
			1, 0, 0, 0,  
			0, 1, 0, 20,  
			0, 0, 1, dist,  
			0, 0, 0, 1); 
			
		
		

		cv::Mat K = (cv::Mat_<double>(3, 4) << 
			5.5555980089858122e+02, 0, 3.1872821392192793e+02, 0,
			0, 5.5875606667787520e+02, 2.4699050318932325e+02, 0,
			0, 0, 1, 0
			); 


		H = (K)* (T * (R * A1));
		H=H.inv();
		
		


		for (int z = 0;z<contours.size();z++)
                  {                                      
                     double x_pro =  (H(0,0)*contours[z].x+H(0,1)*contours[z].y+H(0,2))/(H(2,0)*contours[z].x+H(2,1)*contours[z].y+H(2,2));
                     
                     double y_pro =  (H(1,0)*contours[z].x+H(1,1)*contours[z].y+H(1,2))/(H(2,0)*contours[z].x+H(2,1)*contours[z].y+H(2,2));
                     
                     contours_pro.push_back(cv::Point2d(x_pro,y_pro));
                  }
                  
                  cv::Mat dst = cv::Mat(cv::Size(640, 480), CV_8U,cv::Scalar::all(0));
                  cv::Mat lin = cv::Mat(cv::Size(640, 480), CV_8U,cv::Scalar::all(0));

  
    

for (int z = 0;z<contours_pro.size();z++)
                    {   
                     cv::circle( dst, contours_pro[z], 2, cv::Scalar(255), 2);
                     if ((contours_pro[z].x > 330 & contours_pro[z].x < 390) & contours_pro[z].y > 250 & contours_pro[z].x < 370 )
                         {
                           line.push_back(cv::Point2d(contours_pro[z].x,contours_pro[z].y));
                         
                         }
                     }
                     
                    /* for (int z = 0;z<line.size();z++)
                    {   
                     cv::circle( lin, line[z], 2, cv::Scalar(255), 2);
                    
                     }*/
                     
                     
                   
                     
                     
                     cv::Vec4f line_para; 
	cv::fitLine(contours_pro, line_para, CV_DIST_L2, 0, 0.01, 0.01);
 
	//std::cout << "line_para = " << line_para << std::endl;
         
        cv::Point point0;
	point0.x = line_para[2];
	point0.y = line_para[3];
 
	double k = line_para[1] / line_para[0];

	cv::Point point1, point2;
	point1.x = 0;
	point1.y = k * (0 - point0.x) + point0.y;
	point2.x = 640;
	point2.y = k * (640 - point0.x) + point0.y;
 
	cv::line(lin, point1, point2, 255, 2, 8, 0);
	
                     

                          contours.clear();
                          line.clear();
     contours_pro.clear();
         cv::imshow("gris",gris);
         cv::imshow("segmentado",res);
         cv::imshow("vista de aguila",dst);
         cv::imshow("control",lin);
         
         
         cv::waitKey(30);
     }
     catch (cv_bridge::Exception& e) {
         ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
     }
}

int main(int argc, char **argv) {
          std::cout<<"inicio"<<std::endl;
     ros::init(argc, argv, "image_listener");
     ros::NodeHandle nh;
     cv::namedWindow("view");
     cv::startWindowThread();
     image_transport::ImageTransport it(nh);
     image_transport::Subscriber sub = it.subscribe("/app/camera/rgb/image_raw", 1, imageCallback);
     ros::Publisher chatter_pub = nh.advertise<std_msgs::Int16>("/AutoNOMOS_mini/manual_controlpeed", -100);
     ros::Publisher pub = nh.advertise<std_msgs::Int16>("/AutoNOMOS_mini/manual_control/speed", 500);
     std::cout<<"publicado"<<std::endl;
     ros::Rate rate(10.0);
     while(nh.ok()) {
       ros::spinOnce();
       rate.sleep();
     }
     ros::spin();
     ros::shutdown();
     cv::destroyWindow("view");
}
