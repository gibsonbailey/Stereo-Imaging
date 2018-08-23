/*
 * 
 *		Bailey Lind-Trefts
 *
 *    Trefts Technologies
 * 
 * 
 *
 *    This program is used for single-camera calibration. Given a set of images of the standard OpenCV 6 X 9 vertex chessboard, 
 *    it can solve for and export the camera matrix and distortion coefficients associated with a lense. This is a required
 *    operation before calibrating a stereo pair of cameras.
 *  
 *		Unique cameras should have unique camera names, so that their properties are not overwritten. Images used for calibration
 *		should be contained within the calibration_images directory and should exhibit the .jpg file extension.
 *		  
 */


#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"


#include <iostream>
#include <fstream>


const cv::Size chessBoardSize(6, 9); 
const float calibrationSquareDimension = 0.01905f; // meters


void getChessboardCorners(std::vector<cv::Mat> images, std::vector<std::vector<cv::Point2f> >& allFoundCorners, bool showResults = false);
void createBoardPoints(const cv::Size boardSize, const float squareEdgeLength, std::vector<cv::Point3f>& corners);
void calibrate(const std::vector<cv::Mat>& images, cv::Mat& cameraMatrix, cv::Mat& distortionCoefficients, cv::Mat& rotationVecs, cv::Mat& translationVecs);
void saveCalibration(const std::string& filename, const cv::Mat& cameraMatrix, const cv::Mat& distortionCoefficients);
void loadImages(const std::string path, std::vector<cv::Mat>& images); void showUndistortedVideo(const cv::Mat cameraMat, const cv::Mat distMat); 


int main(int argc, char** argv)
{
	cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_64F);
	cv::Mat distortionCoefficients, rotVecs, transVecs;
	std::vector<cv::Mat> savedImages;
	std::vector<std::vector<cv::Point2f> > markerCorners, rejectedCandidates;
	
	std::string ImagePath = "./images/calibration_images/*.jpg"; // .jpg only
	std::string cameraName = "MacbookPro2015";

	// load calibration image set into memory
	loadImages(ImagePath, savedImages);

	// solve for camera matrix and distortion coefficients using calibration image set
	calibrate(savedImages, cameraMatrix, distortionCoefficients, rotVecs, transVecs);

	// save camera matrix and distortion coefficients to disc
	saveCalibration("./calibration_results/" + cameraName, cameraMatrix, distortionCoefficients);

	// demonstrate calibration using live video through webcam
	showUndistortedVideo(cameraMatrix, distortionCoefficients);

	return 0;
}


void showUndistortedVideo(const cv::Mat cameraMat, const cv::Mat distMat)
{
	cv::VideoCapture cap(0);

	cv::Mat frame;
	cv::Mat undistortedFrame;
	
	if(!cap.isOpened())
	{
		return;
	}

	const int FPS = 20;

	cv::namedWindow("Camera-View", CV_WINDOW_AUTOSIZE);

	bool undistorted = true;
	std::cout << "Press 'u' to toggle post-calibration/prior-calibration." << std::endl;
	std::cout << "Press 'q' to quit." << std::endl;
	

	while(true)
	{
	
		if(!cap.read(frame))
		{
			break;
		}

		frame.copyTo(undistortedFrame);	
		cv::undistort(undistortedFrame, frame, cameraMat, distMat);


		cv::Size sz(int(frame.cols / 1.5),int(frame.rows / 1.5));
		cv::resize(frame, frame, sz);
		cv::resize(undistortedFrame, undistortedFrame, sz);
		
		if(undistorted)
		{
			cv::putText(undistortedFrame, "Post-Calibration", cv::Point(25,25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255,255,255), 1, CV_AA);
			cv::imshow("Camera-View", undistortedFrame);
		}
		else
		{
			cv::putText(frame, "Pre-Calibration", cv::Point(25,25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255,255,255), 1, CV_AA);
			cv::imshow("Camera-View", frame);
		}

		char c = cv::waitKey(1000.0 / FPS);	
		if(c == 'u'){
			if(undistorted)
			{
				undistorted = false;
			}
			else
			{
				undistorted = true;
			}
		}
		else if(c == 'q')
		{
			break;
		}
	}
}


void calibrate(const std::vector<cv::Mat>& images, cv::Mat& cameraMatrix, cv::Mat& distortionCoefficients, cv::Mat& rotationVecs, cv::Mat& translationVecs)
{
	std::vector<std::vector<cv::Point2f> > objectImageSpaceCoords;
	getChessboardCorners(images, objectImageSpaceCoords, false);
	// std::cout << "images Coords Size: " << objectImageSpaceCoords.size() << std::endl;
	
	std::vector<std::vector<cv::Point3f> > objectWorldSpaceCoords(1);
	createBoardPoints(chessBoardSize, calibrationSquareDimension, objectWorldSpaceCoords[0]);
	objectWorldSpaceCoords.resize(objectImageSpaceCoords.size(), objectWorldSpaceCoords[0]);
	// std::cout << "world Coords Size: " << objectWorldSpaceCoords.size() << std::endl;

	cv::calibrateCamera(objectWorldSpaceCoords, objectImageSpaceCoords, chessBoardSize, cameraMatrix, distortionCoefficients, rotationVecs, translationVecs);	
}


void saveCalibration(const std::string& filename, const cv::Mat& cameraMatrix, const cv::Mat& distortionCoefficients)
{
	std::ofstream fOut(filename);
	if(fOut)
	{		

		// Camera Matrix
		for(int r = 0; r < cameraMatrix.rows; r++)
		{
			for(int c = 0; c < cameraMatrix.cols; c++)
			{
				fOut << cameraMatrix.at<double>(r,c) << std::endl;
			}
		}
		
		
		// Distortion Coefficients 
		for(int r = 0; r < distortionCoefficients.rows; r++)
		{
			for(int c = 0; c < distortionCoefficients.cols; c++)
			{
				fOut << distortionCoefficients.at<double>(r,c) << std::endl;
			}
		}
	}

	fOut.close();
}


void createBoardPoints(const cv::Size boardSize, const float squareEdgeLength, std::vector<cv::Point3f>& corners)
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


void loadImages(const std::string path, std::vector<cv::Mat>& images)
{
	std::vector<cv::String> filenames;
	cv::glob(path, filenames, false);

	if(filenames.size() <= 0)
	{
		std::cerr << "Invalid path to calibration images." << std::endl;
		return;
	}
	else
	{
		std::cout << "Number of images available for calibration: " << filenames.size() << std::endl;
	}

	for(int i = 0; i < filenames.size(); i++)
	{
		cv::Mat im = cv::imread(filenames[i]);

		if(im.empty())
		{
			std::cerr << "Failed to load: " + filenames[i] << std::endl;
		}
		else
		{
			std::vector<cv::Point2f> pointBuf;
			bool found = cv::findChessboardCorners(im, chessBoardSize, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
			
			if(found)
			{
				images.push_back(im);
			}
		}
	}
	std::cout << "Number of images selected for calibration: " << images.size() << std::endl;
}
