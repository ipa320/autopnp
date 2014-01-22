/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2013 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: care-o-bot
* \note
* ROS stack name: autopnp
* \note
* ROS package name: autopnp_dirt_detection
*
* \author
* Author: Richard Bormann
* \author
* Supervised by:
*
* \date Date of creation: January 2014
*
* \brief
* Module for checking the appearance of ground areas against a stored desired appearance.
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************/

// standard includes
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// false alarm reduction by image comparison:
// 1. during exploration, always save most central image snippet of each dirt location, and perspective
// 2. before cleaning, compare image snippets against false alarm database
//   a) align the (slightly inflated) image patch against db patch in rotation (roughly by perspective, thoroughly by gradient histogram, weighted with distance to center)
//   b) align translation of db patch with SSD on gradient image
//   c) check for dirt with SSD on gradient image and dirt image response


class AppearanceCheck
{
private:

public:

	void databaseTest()
	{
		std::string filename = ros::package::getPath("autopnp_dirt_detection") + "/common/files/ac_database/images.txt";
		std::ifstream f(filename.c_str(), std::fstream::in);
		if(!f.is_open())
		{
			std::cout << "databaseTest: Could not load '" << filename << "'" << std::endl;
			return;
		}

		while (f.eof() == false)
		{
			std::string patchFile, referenceFile;
			f >> patchFile;
			f >> referenceFile;

			std::stringstream patchFileS, referenceFileS;
			patchFileS << ros::package::getPath("autopnp_dirt_detection") << "/common/files/ac_database/" << patchFile;
			referenceFileS << ros::package::getPath("autopnp_dirt_detection") << "/common/files/ac_database/" << referenceFile;
			cv::Mat patch = cv::imread(patchFileS.str());
			cv::Mat referencePatch = cv::imread(referenceFileS.str());

			compareAppearance(patch, referencePatch);
		}

		f.close();
	}

	// compares a roughly localized image patch (+/- 10 deg rotation, +/- 30 px translation) against a reference image patch
	void compareAppearance(const cv::Mat& patch, const cv::Mat& referencePatch)
	{
		// compute rotational offset between image patch and reference patch
		cv::Mat patchAngleHistogram, referenceAngleHistogram;
		cv::Mat patchDx, patchDy, referenceDx, referenceDy;
		computeAngleHistogram(patch, patchDx, patchDy, patchAngleHistogram, true, true);
		computeAngleHistogram(referencePatch, referenceDx, referenceDy, referenceAngleHistogram, true, true);
		int rotationalOffset=0;
		double matchScore = 0.0;
		matchAngleHistogram(referenceAngleHistogram, patchAngleHistogram, 0, rotationalOffset, matchScore);
		std::cout << "Best match at rotational offset " << rotationalOffset << "deg.\n";

		// turn image patch in correct rotational alignment to reference patch
		cv::Mat patchRotated;
		cv::Mat rotationMatrix = cv::getRotationMatrix2D(cv::Point2f(patch.cols/2, patch.rows/2), (double)rotationalOffset, 1.0);
		cv::warpAffine(patch, patchRotated, rotationMatrix, patch.size());

		// determine translational offset
		cv::Mat referenceMagnitude;
		cv::magnitude(referenceDx, referenceDy, referenceMagnitude);
		cv::Mat patchRotatedDx, patchRotatedDy, patchRotatedMagnitude;
		cv::Sobel(patchRotated, patchRotatedDx, CV_32F, 1, 0, 3);
		cv::Sobel(patchRotated, patchRotatedDy, CV_32F, 0, 1, 3);
		cv::magnitude(patchRotatedDx, patchRotatedDy, patchRotatedMagnitude);
		int minV = std::max(0, patch.rows/2-referencePatch.rows);
		int maxV = std::min(patch.rows-1, patch.rows/2+referencePatch.rows)-referencePatch.rows;
		int minU = std::max(0, patch.cols/2-referencePatch.cols);
		int maxU = std::min(patch.cols-1, patch.cols/2+referencePatch.cols)-referencePatch.cols;
		double minGradientSSD = 1e10;
		int offsetU=0, offsetV=0;
		for (int v=minV; v<maxV; ++v)
		{
			for (int u=minU; u<maxU; ++u)
			{
				double ssd = computeSSD<float>(referenceMagnitude, patchRotatedMagnitude(cv::Rect(u,v,referencePatch.cols, referencePatch.rows)));
				if (ssd < minGradientSSD)
				{
					minGradientSSD = ssd;
					offsetU = u;
					offsetV = v;
				}
			}
		}

		// judge similarity measure value
		std::cout << "Gradient SSD is " << minGradientSSD << " at offsets (u,v,rot) = (" << offsetU << ", " << offsetV << ", " << rotationalOffset << ")\n";
		double distance = minGradientSSD / ((double)referencePatch.cols*referencePatch.rows);
	}

	// computes a gradient angle histogram over the given image
	bool computeAngleHistogram(const cv::Mat& image, cv::Mat& dx, cv::Mat& dy, cv::Mat& histogram, bool smoothHistogram=true, bool display=false)
	{
		if (image.rows != image.cols)
		{
			std::cout << "computeAngleHistogram: Error: image width and height differ.";
			return false;
		}

		// compute weighted angle histogram
		if (dx.empty() == true)
			cv::Sobel(image, dx, CV_32F, 1, 0, 3);
		if (dy.empty() == true)
			cv::Sobel(image, dy, CV_32F, 0, 1, 3);
		histogram = cv::Mat::zeros(1, 360, CV_32FC1);
		cv::Mat weightKernel = cv::getGaussianKernel(2*(image.rows/2)+1,-1,CV_32F);

		for (int v=0; v<image.rows; ++v)
		{
			for (int u=0; u<image.cols; ++u)
			{
				double alpha = 180.0 + 180.0/M_PI*cv::fastAtan2(dy.at<float>(v,u),dx.at<float>(v,u));
				histogram.at<float>((int)alpha) += weightKernel.at<float>(v,u);
			}
		}

		// smooth histogram
		if (smoothHistogram)
		{
			cv::Mat histogramSmoothed(1, histogram.cols, CV_32FC1);
			histogramSmoothed.at<float>(0) = 0.25*histogram.at<float>(histogram.cols-1) + 0.5*histogram.at<float>(0) + 0.25*histogram.at<float>(1);
			histogramSmoothed.at<float>(histogram.cols-1) = 0.25*histogram.at<float>(histogram.cols-2) + 0.5*histogram.at<float>(histogram.cols-1) + 0.25*histogram.at<float>(0);
			for (int i=1; i<histogramSmoothed.cols-1; i++)
				histogramSmoothed.at<float>(i) = 0.25*histogram.at<float>(i-1) + 0.5*histogram.at<float>(i) + 0.25*histogram.at<float>(i+1);
			histogram = histogramSmoothed;
		}

		if (display)
		{
			cv::Mat dispHist(300, 360, CV_8UC1);
			dispHist.setTo(255);
			for (int i=0; i<histogram.cols; i++)
				cv::line(dispHist, cv::Point(i, 0), cv::Point(i, (int)histogram.at<float>(i)), CV_RGB(0,0,0));
			cv::imshow("histogram", dispHist);
			cv::waitKey(10);
		}

		return true;
	}

	bool matchAngleHistogram(const cv::Mat& referenceHistogram, const cv::Mat& matchHistogram, int approximateOffset, int& offset, double& matchScore)
	{
		matchScore = 0.0;
		for (int o=-10; o<=10; ++o)
		{
			double score = histogramIntersectionKernel(referenceHistogram, matchHistogram, approximateOffset+o);
			if (score > matchScore)
			{
				matchScore = score;
				offset = approximateOffset+o;
			}
		}

		return true;
	}

	double histogramIntersectionKernel(const cv::Mat& referenceHistogram, const cv::Mat& matchHistogram, int offset)
	{
		double score = 0;

		if ((int)referenceHistogram.cols != matchHistogram.cols)
			std::cout << "histogramIntersectionKernel: Error: Array sizes do not match." << std::endl;

		int size = matchHistogram.cols;

		for (int i=0; i<size; i++)
			score += std::min<float>(referenceHistogram.at<float>(i), matchHistogram.at<float>((i+offset)%size));
		return score;
	}

	template <typename t>
	double computeSSD(const cv::Mat& img1, const cv::Mat& img2)
	{
		double ssd = 0.;
		for (int v=0; v<img1.rows; ++v)
		{
			for (int u=0; u<img1.cols; ++u)
			{
				double diff = img1.at<t>(v,u)-img2.at<t>(v,u);
				ssd += diff*diff;
			}
		}
		return ssd;
	}
};



int main(int argc, char **argv)
{

	ros::init(argc, argv, "appearance check");

	ros::NodeHandle n;

	AppearanceCheck ac;
	ac.databaseTest();

	ros::spin();

	return 0;
}
