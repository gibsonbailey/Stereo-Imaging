#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
// #include "opencv2/aruco.hpp"

#include <iostream>
#include <fstream>

const cv::Size chessBoardSize(6, 9);  
const float calibrationSquareDimension = 0.01905f;

cv::Mat buildBlackImg();
void displayImage(const cv::Mat& img, const float scale_factor = 0.1);
void getChessboardCorners(std::vector<cv::Mat> images, std::vector<std::vector<cv::Point2f> >& allFoundCorners, bool showResults = false);
void showChessboardCornersLive();

int main(int argc, char** argv)
{
	//cv::Mat img = buildBlackImg(); 
	//cv::Mat nore_img = cv::imread("nore_image.jpg", CV_LOAD_IMAGE_COLOR); 

	// displayImage(nore_img, 0.15); 	
	




	cv::Mat frame;
	cv::Mat drawToFrame;

	cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_64F);

	cv::Mat distortionCoefficients;

	std::vector<cv::Mat> savedImages;

	std::vector<std::vector<cv::Point2f> > markerCorners, rejectedCandidates;
	
//	std::cout << "Check 1" << std::endl;

	cv::Mat chess_img = cv::imread("chess_perfect.png");
//	std::cout << "Check 2" << std::endl;
//	savedImages.push_back(chess_img);
//	getChessboardCorners(savedImages, markerCorners, true);
//	std::cout << "Check 3" << std::endl;




	showChessboardCornersLive();


	return 0;
}


void showChessboardCornersLive()
{
	cv::Mat frame;
	cv::Mat drawToFrame;

	cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_64F);

	cv::Mat distortionCoefficients;

	std::vector<cv::Mat> savedImages;

	std::vector<std::vector<cv::Point2f> > markerCorners, rejectedCandidates;
	
//	std::cout << "Check 1" << std::endl;

	cv::Mat chess_img = cv::imread("chess_perfect.png");
//	std::cout << "Check 2" << std::endl;
	savedImages.push_back(chess_img);
//	getChessboardCorners(savedImages, markerCorners, true);
//	std::cout << "Check 3" << std::endl;

	const bool live_video = true;

	cv::VideoCapture vid(0);

	if(!vid.isOpened())
	{
		return;
	}
//	std::cout << "Check 4" << std::endl;

	const int FPS = 20;

	cv::namedWindow("Forward-Facing Webcam", CV_WINDOW_AUTOSIZE);
//	std::cout << "Check 5" << std::endl;

	while(true)
	{
//		std::cout << "Check 6" << std::endl;
	
		if(!vid.read(frame))
		{
			break;
		}

		std::vector<cv::Vec2f> foundPoints;
		bool found = false;
		
		cv::Size sz(int(frame.cols / 1.5),int(frame.rows / 1.5));
		cv::resize(frame, frame, sz);
		
		found = cv::findChessboardCorners(frame, chessBoardSize, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
		frame.copyTo(drawToFrame);
		cv::drawChessboardCorners(drawToFrame, chessBoardSize, foundPoints, found);
		std::cout << "found: " << found << std::endl;
		if(found)
		{
			cv::imshow("Webcam", drawToFrame);
		}
		else {
			cv::imshow("Webcam", frame);
		}
		char c = cv::waitKey(1000 / FPS);

		if(c == 'q')
		{
			break;
		}
		else if (c == 32) // Space Bar
		{
			// add to saved images
			savedImages.push_back(frame);
		}
		else if (c == 27) // Escape Key
		{
			break;
		}
		else if (c == 13) // Carriage Return
		{
			// perform calibration
		}

		std::cout << "Size of image bank: " << savedImages.size() << std::endl;
}
	
}

void calibrate(const std::vector<cv::Mat>& images, const std::vector<std::vector<cv::Point2f> > objectImageSpaceCoords, cv::Mat& cameraMatrix, cv::Mat& distortionCoefficients, cv::Mat&
rotationVecs, cv::Mat& translationVecs)
{
	std::vector<cv::Point3f> objectWorldSpaceCoords;
	createBoardPoints(chessBoardSize, calibrationSquareDimension, objectWorldSpaceCoords);

	

	cv::calibrateCamera(objectWorldSpaceCoords, objectImageSpaceCoords, chessBoardSize, cameraMatrix, distortionCoefficients, rotationVecs, translationVecs);	

}


void createBoardPoints(cv::Size boardSize, float squareEdgeLength, std::vector<cv::Point3f>& corners)
{
	for(int i = 0; i < boardSize.height; i++)
	{
		for(int j = 0; j < boardSize.width; j++)
		{
			corners.push_back(cv::Point3f(j * squareEdgeLength, i * squareEdgeLength, 0.0f));
		}
	} 
}

void getChessboardCorners(std::vector<cv::Mat> images, std::vector<std::vector<cv::Point2f> >& allFoundCorners, bool showResults)
{
	for (std::vector<cv::Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
	{
		std::vector<cv::Point2f> pointBuf;
		bool found = cv::findChessboardCorners(*iter, chessBoardSize, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

		if(found)
		{
			allFoundCorners.push_back(pointBuf);
		}

		if(showResults)
		{
			std::cout << "found: " << found << std::endl;
			drawChessboardCorners(*iter, chessBoardSize, pointBuf, found);
			cv::imshow("Looking for corners", *iter);
			cv::waitKey();
		}
	}
}



















void displayImage(const cv::Mat& img, const float scale_factor)
{
	//	const float scale_factor = 0.1;
	
	cv::namedWindow("Test Image", CV_WINDOW_AUTOSIZE); 
	
	cv::Size s(int(img.cols *	scale_factor),int(img.rows * scale_factor));

	cv::Mat dst;
	cv::resize(img, dst, s); 	
	cv::imshow("Test Image", dst);
	cv::waitKey(); 
}


cv::Mat buildBlackImg()
{
	cv::Mat black_img;
	

	return black_img;
}


