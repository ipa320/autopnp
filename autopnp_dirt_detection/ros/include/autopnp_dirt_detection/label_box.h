#include "opencv2/core/core.hpp"
#include "cv.h"
#include "highgui.h"

#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <stack>
#include <cstdio>
#include <dirent.h>
#include <time.h>

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/fstream.hpp"

#include <time.h>


void onMouse(int event, int x, int y, int flags, void* param);

class labelImage
{
protected:
	int input_text(cv::Mat img, std::string winName, std::string * text, cv::Rect rect);

	// ASCII KEY CODES
	const static int LEFT = 1113937;
	const static int LEFT2 = 65361;
	const static int UP = 65362;
	const static int UP2 = 1113938;
	const static int RIGHT = 65363;
	const static int RIGHT2 = 1113939;
	const static int DOWN = 65364;
	const static int DOWN2 = 1113940;
	const static int RETURN = 10;
	const static int RETURN2 = 1048586;
	const static int C = 99;
	const static int C2 = 1048675;
	const static int H = 104;
	const static int H2 = 1048680;
	const static int S = 115;
	const static int S2 = 1048691;
	const static int R = 114;
	const static int R2 = 1048690;
	const static int Z = 122;
	const static int Z2 = 1048698;
	const static int ESC = 27;
	const static int ESC2 = 1048603;
	const static int LSHIFT = 65505;
	const static int LSHIFT2 = 1114081;
	const static int RSHIFT = 65506;
	const static int RSHIFT2 = 1114082;
	const static int BACKSPACE = 65288;
	const static int BACKSPACE2 = 1113864;
	const static int SPACE = 32;
	const static int SPACE2 = 1048608;
	const static int A = 97;
	const static int A2 = 666665;
	const static int D = 100;
	const static int D2 = 666666;

public:
	labelImage(std::string winName, cv::Mat imageWithRects, cv::Mat originalImage, cv::Scalar clr) :
		name(winName), img(imageWithRects), originalImage(originalImage), actualClr(clr),
		actualRect(cv::Rect_<float>(1.0, 1.0, 1.0, 1.0)), middlePoint(0.0, 0.0), textMode(false), rotationMode(false)
	{

	}
	~labelImage()
	{

	}

	void labelingLoop();

	static void writeTxt(std::vector<labelImage> all, std::string path, std::string actTime);

	static void writeTxt3d(std::vector<labelImage> all, std::string path, std::string actTime);

	static bool checkPointInsideRect(cv::RotatedRect rect, cv::Point2f p);

	static void showInfo(std::string winName, cv::Mat img, cv::RotatedRect r, std::string text, cv::Scalar clr);

	static void drawRect(cv::Mat & img, cv::RotatedRect rrect, cv::Scalar clr);

	std::string name;

	cv::Mat img;
	cv::Mat originalImage;
	//cv::Mat temporaryImage;

	std::vector<cv::RotatedRect> allRects;
	std::vector<std::string> allTexts;
	std::vector<cv::Scalar> allClrs;

	// 3d coordinate representation with 3 points, i.e. one center point and two border points in width (p1) and height (p2) direction
	struct RegionPointTriple
	{
		cv::Point3f center;
		cv::Point3f p1;	// width direction
		cv::Point3f p2;	// height direction
	};
	std::vector<RegionPointTriple> allRects3d;

	cv::Scalar actualClr;
	cv::Rect_<float> actualRect;
	cv::RotatedRect actualRotatedRect;

	cv::Point2f middlePoint;

	bool textMode;

	bool rotationMode;
};
