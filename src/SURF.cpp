#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include "opencv2/xfeatures2d.hpp"

using namespace cv::xfeatures2d;
cv::Mat object;  
cv::UMat frame, frame_gray,object_gray, object_scaled;

int minHessian = 120; //100 100 120
cv::Ptr<SURF> detector = SURF::create( minHessian );
std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;
cv::UMat descriptors_object, descriptors_scene;
cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
std::vector< std::vector<cv::DMatch> > knn_matches;
const float ratio_thresh = 0.75f; //0.65 0.75 0.75
std::vector<cv::DMatch> good_matches;



int main(int argc, char **argv){
	ros::init(argc, argv, "SURF");
	ros::NodeHandle n;
	ros::Rate loop_rate(15);

	cv::namedWindow("video");         		
	cv::moveWindow("video", 100,100);		
	cv::namedWindow("balon");
	cv::moveWindow("balon", 500,400);
	
	std::string path_image = "/home/marco/catkin_ws/src/vision_tools/img/cuad2.png";
	object = cv::imread(path_image);  
	cv::cvtColor(object, object_gray, cv::COLOR_BGR2GRAY);
	detector->detectAndCompute( object_gray, cv::noArray(), keypoints_object, descriptors_object );
	
	cv::UMat balon;
	
	cv::drawKeypoints(object_gray, keypoints_object, balon);
	cv::imshow("balon", balon);
	cv::waitKey(15);
	
	cv::VideoCapture cap(0);	
	cap.set(CV_CAP_PROP_FPS,15);
	//cap.set(CV_CAP_PROP_FRAME_WIDTH,320);
	//cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);
	detector->setHessianThreshold(120); //100 40 120
	while(ros::ok()){
		cap >> frame;
		if(frame.empty())  exit(1);
		cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

		//-- Detect the keypoints using SURF Detector, compute the descriptors
		
		detector->detectAndCompute( frame_gray, cv::noArray(), keypoints_scene, descriptors_scene );
		//-- Matching descriptor vectors with a FLANN based matcher

		matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2 );
		//-- Filter matches using the Lowe's ratio test

		for (size_t i = 0; i < knn_matches.size(); i++){
		   if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance){
			  good_matches.push_back(knn_matches[i][0]);}
		}
		
		//-- Draw matches
		cv::UMat img_matches;
		cv::drawMatches( object, keypoints_object, frame, keypoints_scene, good_matches, img_matches, cv::Scalar::all(-1),
				  cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
		//-- Localize the object
		
		std::vector<cv::Point2f> obj;
		std::vector<cv::Point2f> scene;
		for( size_t i = 0; i < good_matches.size(); i++ ){
		   //-- Get the keypoints from the good matches
		   obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
		   scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
		}
		
		
		if (good_matches.size()<14 ) { //7 12 14
			std::cout << good_matches.size() << "  No matches found at all....." << std::endl;

		}else{
			std::cout << good_matches.size() << std::endl;
			cv::Mat H = findHomography( obj, scene, cv::RANSAC );
			//-- Get the corners from the image_1 ( the object to be "detected" )
			std::vector<cv::Point2f> obj_corners(4);
			obj_corners[0] = cv::Point2f(0, 0);
			obj_corners[1] = cv::Point2f( (float)object_gray.cols, 0 );
			obj_corners[2] = cv::Point2f( (float)object_gray.cols, (float)object_gray.rows );
			obj_corners[3] = cv::Point2f( 0, (float)object_gray.rows );
			std::vector<cv::Point2f> scene_corners(4);
			cv::perspectiveTransform( obj_corners, scene_corners, H);
			//-- Draw lines between the corners (the mapped object in the scene - image_2 )
			cv::line( img_matches, scene_corners[0] + cv::Point2f((float)object_gray.cols, 0),
				scene_corners[1] + cv::Point2f((float)object_gray.cols, 0), cv::Scalar(0, 255, 0), 4 );
			cv::line( img_matches, scene_corners[1] + cv::Point2f((float)object_gray.cols, 0),
				scene_corners[2] + cv::Point2f((float)object_gray.cols, 0), cv::Scalar( 0, 255, 0), 4 );
			cv::line( img_matches, scene_corners[2] + cv::Point2f((float)object_gray.cols, 0),
				scene_corners[3] + cv::Point2f((float)object_gray.cols, 0), cv::Scalar( 0, 255, 0), 4 );
			cv::line( img_matches, scene_corners[3] + cv::Point2f((float)object_gray.cols, 0),
				scene_corners[0] + cv::Point2f((float)object_gray.cols, 0), cv::Scalar( 0, 255, 0), 4 );
		}
		//	drawMatches( image1_gray, keypoints1, image2_gray, keypoints2, matches, img_matches );
		cv::imshow("video",img_matches);
		cv::waitKey(1);
		good_matches.clear();
		knn_matches.clear();
		obj.clear();
		scene.clear();
		//ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
