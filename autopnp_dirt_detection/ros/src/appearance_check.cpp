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

			if (patchFile.length()==0 || referenceFile.length()==0)
				break;

			std::stringstream patchFileS, referenceFileS;
			patchFileS << ros::package::getPath("autopnp_dirt_detection") << "/common/files/ac_database/" << patchFile;
			referenceFileS << ros::package::getPath("autopnp_dirt_detection") << "/common/files/ac_database/" << referenceFile;
			cv::Mat patch = cv::imread(patchFileS.str());
			cv::Mat referencePatch = cv::imread(referenceFileS.str());

			std::cout << "-------------------------------------------------------------\nImage pair: " << patchFile << " - " << referenceFile << std::endl;
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
		int ksize = 2*(referencePatch.rows/2)+1;
		double sigma = /*0.3*/0.4*((ksize-1)*0.5 - 1) + 0.8;
		computeAngleHistogram(patch, sigma, patchDx, patchDy, patchAngleHistogram, true, false);
		computeAngleHistogram(referencePatch, sigma, referenceDx, referenceDy, referenceAngleHistogram, true, false);
		int rotationalOffset=0;
		double matchScore = 0.0;
		matchAngleHistogram(referenceAngleHistogram, patchAngleHistogram, 0, rotationalOffset, matchScore);
		//std::cout << "Best match at rotational offset " << rotationalOffset << " deg.\n";

		// turn image patch in correct rotational alignment to reference patch
		cv::Mat patchRotated;
		cv::Mat rotationMatrix = cv::getRotationMatrix2D(cv::Point2f(patch.cols/2, patch.rows/2), (double)rotationalOffset, 1.0);
		cv::warpAffine(patch, patchRotated, rotationMatrix, patch.size());

		// determine translational offset
		cv::Mat referenceMagnitude;
		cv::magnitude(referenceDx, referenceDy, referenceMagnitude);
		cv::Mat patchRotatedDx, patchRotatedDy, patchRotatedMagnitude;
		cv::Mat grayImage;
		cv::cvtColor(patchRotated, grayImage, CV_BGR2GRAY);
		cv::Sobel(grayImage, patchRotatedDx, CV_32F, 1, 0, 7);
		cv::Sobel(grayImage, patchRotatedDy, CV_32F, 0, 1, 7);
		cv::magnitude(patchRotatedDx, patchRotatedDy, patchRotatedMagnitude);
		int minV = std::max(0, patch.rows/2-referencePatch.rows);
		int maxV = std::min(patch.rows, patch.rows/2+referencePatch.rows)-referencePatch.rows;
		int minU = std::max(0, patch.cols/2-referencePatch.cols);
		int maxU = std::min(patch.cols, patch.cols/2+referencePatch.cols)-referencePatch.cols;
		double minGradientSSD = 1e100;
		int offsetU=0, offsetV=0;
		for (int v=minV; v<=maxV; ++v)
		{
			for (int u=minU; u<=maxU; ++u)
			{
				double ssd = sqrt(computeSSD<float>(referenceMagnitude, patchRotatedMagnitude(cv::Rect(u,v,referencePatch.cols, referencePatch.rows))));
				if (ssd < minGradientSSD)
				{
					minGradientSSD = ssd;
					offsetU = u;
					offsetV = v;
				}
			}
		}

		cv::Rect roi(offsetU,offsetV,referencePatch.cols, referencePatch.rows);

		// dirt image
		cv::Mat patchRotatedSaliency, referenceSaliency;
		cv::Mat patchRotatedMask = cv::Mat::ones(referencePatch.rows, referencePatch.cols, CV_8UC1);
		cv::Mat referenceMask = cv::Mat::ones(referencePatch.rows, referencePatch.cols, CV_8UC1);
		cv::Mat C1_saliency_image;
		SaliencyDetection_C3(referencePatch, C1_saliency_image, &referenceMask, 3);
		// post processing, dirt/stain selection
		cv::Mat C1_BlackWhite_image;
		cv::Mat new_plane_color_image = referencePatch.clone();
		std::vector<cv::RotatedRect> dirtDetections;
		Image_Postprocessing_C1_rmb(C1_saliency_image, referenceSaliency, C1_BlackWhite_image, new_plane_color_image, dirtDetections, referenceMask);
		// dirt image
		SaliencyDetection_C3(patchRotated(roi), C1_saliency_image, &patchRotatedMask, 3);
		// post processing, dirt/stain selection
		new_plane_color_image = patchRotated(roi).clone();
		Image_Postprocessing_C1_rmb(C1_saliency_image, patchRotatedSaliency, C1_BlackWhite_image, new_plane_color_image, dirtDetections, patchRotatedMask);
		double dirtSsd = sqrt(computeSSD<float>(referenceSaliency, patchRotatedSaliency));


		// judge similarity measure value
		double gradientDistance = minGradientSSD / ((double)referencePatch.cols*referencePatch.rows);
		double dirtDistance = dirtSsd / ((double)referencePatch.cols*referencePatch.rows);
		std::cout << "Gradient SSD is " << minGradientSSD << " (" << gradientDistance << "), saliency SSD is " << dirtSsd << " (" << dirtDistance << ") at offsets (u,v,rot) = (" << offsetU << ", " << offsetV << ", " << rotationalOffset << ")\n";
		cv::imshow("referencePatch", referencePatch);
		cvMoveWindow("referencePatch", 650, 0);
		cv::imshow("best matching patch", patchRotated(roi));
		cvMoveWindow("best matching patch", 760, 0);
		cv::normalize(referenceMagnitude, referenceMagnitude, 0., 1., cv::NORM_MINMAX);
		cv::Mat dispPatchRotatedMagnitude;
		cv::normalize(patchRotatedMagnitude(roi), dispPatchRotatedMagnitude, 0., 1., cv::NORM_MINMAX);
		cv::imshow("reference magnitude", referenceMagnitude);
		cvMoveWindow("reference magnitude", 650, 150);
		cv::imshow("patch rotated magnitude", dispPatchRotatedMagnitude);
		cvMoveWindow("patch rotated magnitude", 760, 150);
		cv::waitKey();
	}

	// computes a gradient angle histogram over the given image
	bool computeAngleHistogram(const cv::Mat& image, double sigma, cv::Mat& dx, cv::Mat& dy, cv::Mat& histogram, bool smoothHistogram=true, bool display=false)
	{
		if (image.rows != image.cols)
		{
			std::cout << "computeAngleHistogram: Error: image width and height differ.";
			return false;
		}

		// compute weighted angle histogram
		cv::Mat grayImage;
		if (dx.empty() || dy.empty())
			cv::cvtColor(image, grayImage, CV_BGR2GRAY);
		if (dx.empty() == true)
			cv::Sobel(grayImage, dx, CV_32F, 1, 0, 7);
		if (dy.empty() == true)
			cv::Sobel(grayImage, dy, CV_32F, 0, 1, 7);
		histogram = cv::Mat::zeros(1, 360, CV_32FC1);
		cv::Mat weightKernel1D = cv::getGaussianKernel(2*(image.rows/2)+1, sigma, CV_32F);
		cv::Mat weightKernel = weightKernel1D*weightKernel1D.t();
		cv::Mat mag;
		cv::magnitude(dx, dy, mag);
		for (int v=0; v<image.rows; ++v)
		{
			for (int u=0; u<image.cols; ++u)
			{
				//int alpha = (int)(180.0 + 180.0/M_PI*atan2(dy.at<float>(v,u),dx.at<float>(v,u)));
				int alpha = (int)cv::fastAtan2(dy.at<float>(v,u),dx.at<float>(v,u));
				if (alpha == 360)
					alpha = 0;
				histogram.at<float>(alpha) += mag.at<float>(v,u)*weightKernel.at<float>(v,u);
			}
		}

		// smooth histogram
		if (smoothHistogram)
		{
			cv::Mat histogramSmoothed(1, histogram.cols, CV_32FC1);
			histogramSmoothed.at<float>(0) = 0.25*histogram.at<float>(histogram.cols-1) + 0.5*histogram.at<float>(0) + 0.25*histogram.at<float>(1);
			histogramSmoothed.at<float>(histogram.cols-1) = 0.25*histogram.at<float>(histogram.cols-2) + 0.5*histogram.at<float>(histogram.cols-1) + 0.25*histogram.at<float>(0);
			for (int i=1; i<histogramSmoothed.cols-1; ++i)
				histogramSmoothed.at<float>(i) = 0.25*histogram.at<float>(i-1) + 0.5*histogram.at<float>(i) + 0.25*histogram.at<float>(i+1);
			histogram = histogramSmoothed;
		}

		// normalize histogram
		double sum = 0.;
		for (int i=0; i<histogram.cols; ++i)
			sum += histogram.at<float>(i);
		for (int i=0; i<histogram.cols; ++i)
			histogram.at<float>(i) /= sum;

		if (display)
		{
			cv::Mat conv(mag.rows, mag.cols, CV_32FC1);
			cv::multiply(weightKernel(cv::Rect(0,0,mag.cols, mag.rows)), mag, conv);
			cv::normalize(mag, mag, 0., 1., cv::NORM_MINMAX);
			cv::imshow("magnitude", mag);
			cvMoveWindow("magnitude", 870, 0);
//			cv::normalize(weightKernel, weightKernel, 0., 1., cv::NORM_MINMAX);
//			cv::imshow("kernel", weightKernel);
//			cvMoveWindow("kernel", 870, 500);
			cv::normalize(conv, conv, 0., 1., cv::NORM_MINMAX);
			cv::imshow("conv", conv);
			cvMoveWindow("conv", 870, 250);

			cv::Mat dispHist(400, 360, CV_8UC1);
			dispHist.setTo(255);
			for (int i=0; i<histogram.cols; ++i)
				cv::line(dispHist, cv::Point(i, 0), cv::Point(i, (int)(1000.f*histogram.at<float>(i))), CV_RGB(0,0,0));
			cv::imshow("histogram", dispHist);
			cvMoveWindow("histogram", 760, 750);
			cv::waitKey();
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


	// just a copy from dirt_detection.cpp
	void SaliencyDetection_C1(const cv::Mat& C1_image, cv::Mat& C1_saliency_image)
	{
		//given a one channel image
		//int scale = 6;
		//unsigned int size = (int)floor((float)pow(2.0,scale)); //the size to do the saliency at
		double spectralResidualImageSizeRatio_ = 0.25;
		unsigned int size_cols = (int)(C1_image.cols * spectralResidualImageSizeRatio_);
		unsigned int size_rows = (int)(C1_image.rows * spectralResidualImageSizeRatio_);

		//create different images
		cv::Mat bw_im;
		cv::resize(C1_image,bw_im,cv::Size(size_cols,size_rows));

		cv::Mat realInput(size_rows,size_cols, CV_32FC1);	//calculate number of test samples
		//int NumTestSamples = 10; //ceil(NumSamples*percentage_testdata);
		//printf("Anzahl zu ziehender test samples: %d \n", NumTestSamples);
		cv::Mat imaginaryInput(size_rows,size_cols, CV_32FC1);
		cv::Mat complexInput(size_rows,size_cols, CV_32FC2);

		bw_im.convertTo(realInput,CV_32F,1.0/255,0);
		imaginaryInput = cv::Mat::zeros(size_rows,size_cols,CV_32F);

		std::vector<cv::Mat> vec;
		vec.push_back(realInput);
		vec.push_back(imaginaryInput);
		cv::merge(vec, complexInput);

		cv::Mat dft_A(size_rows,size_cols,CV_32FC2);

		cv::dft(complexInput, dft_A, cv::DFT_COMPLEX_OUTPUT,size_rows);
		vec.clear();
		cv::split(dft_A,vec);
		realInput = vec[0];
		imaginaryInput = vec[1];

		// Compute the phase angle
		cv::Mat image_Mag(size_rows,size_cols, CV_32FC1);
		cv::Mat image_Phase(size_rows,size_cols, CV_32FC1);

		//compute the phase of the spectrum
		cv::cartToPolar(realInput, imaginaryInput, image_Mag, image_Phase,0);

		std::string name;
		std::vector<std::string> name_vec;

		name = "carpet-gray.tepp";
		name_vec.push_back(name);
		cv::Mat log_mag(size_rows,size_cols, CV_32FC1);
		cv::log(image_Mag, log_mag);

	//	cv::Mat log_mag_;
	//	cv::normalize(log_mag, log_mag_, 0, 1, NORM_MINMAX);
	//	cv::imshow("log_mag", log_mag_);

		//Box filter the magnitude, then take the difference
		cv::Mat log_mag_Filt(size_rows,size_cols, CV_32FC1);

		cv::Mat filt = cv::Mat::ones(3, 3, CV_32FC1) * 1./9.;
		//filt.convertTo(filt,-1,1.0/9.0,0);

		cv::filter2D(log_mag, log_mag_Filt, -1, filt);
		//cv::GaussianBlur(log_mag, log_mag_Filt, cv::Size2i(25,25), 0);
		//log_mag = log_mag_Filt.clone();
		//cv::medianBlur(log_mag, log_mag_Filt, 5);

	//	cv::Mat log_mag_filt_;
	//	cv::normalize(log_mag_Filt, log_mag_filt_, 0, 1, NORM_MINMAX);
	//	cv::imshow("log_mag_filt", log_mag_filt_);

		//cv::subtract(log_mag, log_mag_Filt, log_mag);
		log_mag -= log_mag_Filt;

	//	cv::Mat log_mag_sub_a_, log_mag_sub_;
	//	cv::normalize(log_mag, log_mag_sub_a_, 0, 1, NORM_MINMAX);
	//	cv::GaussianBlur(log_mag_sub_a_, log_mag_sub_, cv::Size2i(21,21), 0);
	//	cv::imshow("log_mag_sub", log_mag_sub_);
	//	log_mag_Filt = log_mag.clone();
	//	cv::GaussianBlur(log_mag_Filt, log_mag, cv::Size2i(21,21), 0);

		cv::exp(log_mag, image_Mag);

		cv::polarToCart(image_Mag, image_Phase, realInput, imaginaryInput,0);

		vec.clear();
		vec.push_back(realInput);
		vec.push_back(imaginaryInput);
		cv::merge(vec, dft_A);

		cv::dft(dft_A, dft_A, cv::DFT_INVERSE,size_rows);

		dft_A = abs(dft_A);
		dft_A.mul(dft_A);

		cv::split(dft_A, vec);

		C1_saliency_image = vec[0];
	}


	void SaliencyDetection_C3(const cv::Mat& C3_color_image, cv::Mat& C1_saliency_image, const cv::Mat* mask, int gaussianBlurCycles)
	{
		cv::Mat fci; // "fci"<-> first channel image
		cv::Mat sci; // "sci"<-> second channel image
		cv::Mat tci; //"tci"<-> second channel image

		std::vector<cv::Mat> vec;
		vec.push_back(fci);
		vec.push_back(sci);
		vec.push_back(tci);

		cv::split(C3_color_image,vec);

		fci = vec[0];
		sci = vec[1];
		tci = vec[2];

		cv::Mat res_fci; // "fci"<-> first channel image
		cv::Mat res_sci; // "sci"<-> second channel image
		cv::Mat res_tci; //"tci"<-> second channel image

		SaliencyDetection_C1(fci, res_fci);
		SaliencyDetection_C1(sci, res_sci);
		SaliencyDetection_C1(tci, res_tci);

		cv::Mat realInput;

		realInput = (res_fci + res_sci + res_tci)/3;

		cv::Size2i ksize;
		ksize.width = 3;
		ksize.height = 3;
		for (int i=0; i<gaussianBlurCycles; i++)
			cv::GaussianBlur(realInput, realInput, ksize, 0); //necessary!? --> less noise


		cv::resize(realInput,C1_saliency_image,C3_color_image.size());

		// remove borders of the ground plane because of artifacts at the border like lines
		if (mask != 0)
		{
			// maske erodiere
			cv::Mat mask_eroded = mask->clone();
			cv::dilate(*mask, mask_eroded, cv::Mat(), cv::Point(-1, -1), 2);
			cv::erode(mask_eroded, mask_eroded, cv::Mat(), cv::Point(-1, -1), 25.0/640.0*C3_color_image.cols);
			cv::Mat neighborhood = cv::Mat::ones(3, 1, CV_8UC1);
			cv::erode(mask_eroded, mask_eroded, neighborhood, cv::Point(-1, -1), 15.0/640.0*C3_color_image.cols);

			//cv::erode(mask_eroded, mask_eroded, cv::Mat(), cv::Point(-1, -1), 35.0/640.0*C3_color_image.cols);
			// todo: hack for autonomik
			//cv::erode(mask_eroded, mask_eroded, cv::Mat(), cv::Point(-1, -1), 45.0/640.0*C3_color_image.cols);

			cv::Mat temp;
			C1_saliency_image.copyTo(temp, mask_eroded);
			C1_saliency_image = temp;
		}

		// remove saliency at the image border (because of artifacts in the corners)
		int borderX = C1_saliency_image.cols/20;
		int borderY = C1_saliency_image.rows/20;
		if (borderX > 0 && borderY > 0)
		{
			cv::Mat smallImage_ = C1_saliency_image.colRange(borderX, C1_saliency_image.cols-borderX);
			cv::Mat smallImage = smallImage_.rowRange(borderY, smallImage_.rows-borderY);

			copyMakeBorder(smallImage, C1_saliency_image, borderY, borderY, borderX, borderX, cv::BORDER_CONSTANT, cv::Scalar(0));
		}


		// display the individual channels
	//	for (int i=0; i<gaussianBlurCycles; i++)
	//		cv::GaussianBlur(res_fci, res_fci, ksize, 0); //necessary!? --> less noise
	//	for (int i=0; i<gaussianBlurCycles; i++)
	//		cv::GaussianBlur(res_sci, res_sci, ksize, 0); //necessary!? --> less noise	//calculate number of test samples
	//	int NumTestSamples = 10; //ceil(NumSamples*percentage_testdata);
	//	printf("Anzahl zu ziehender test samples: %d \n", NumTestSamples);
	//	for (int i=0; i<gaussianBlurCycles; i++)
	//		cv::GaussianBlur(res_tci, res_tci, ksize, 0); //necessary!? --> less noise
	//
	//	cv::resize(res_fci,res_fci,C3_color_image.size());
	//	cv::resize(res_sci,res_sci,C3_color_image.size());
	//	cv::resize(res_tci,res_tci,C3_color_image.size());
	//
	//	// scale input_image
	//	double minv, maxv;
	//	cv::Point2i minl, maxl;
	//	cv::minMaxLoc(res_fci,&minv,&maxv,&minl,&maxl);
	//	res_fci.convertTo(res_fci, -1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));
	//	cv::minMaxLoc(res_sci,&minv,&maxv,&minl,&maxl);
	//	res_sci.convertTo(res_sci, -1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));
	//	cv::minMaxLoc(res_tci,&minv,&maxv,&minl,&maxl);
	//	res_tci.convertTo(res_tci, -1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));
	//
	//	cv::imshow("b", res_fci);
	//	cv::imshow("g", res_sci);
	//	cv::imshow("r", res_tci);
	}

	void Image_Postprocessing_C1_rmb(const cv::Mat& C1_saliency_image, cv::Mat& scaled_C1_saliency_image, cv::Mat& C1_BlackWhite_image, cv::Mat& C3_color_image, std::vector<cv::RotatedRect>& dirtDetections, const cv::Mat& mask)
	{
		double dirtThreshold_ = 0.2;
		double dirtCheckStdDevFactor_ = 3.0;
		int spectralResidualGaussianBlurIterations_ = 3;
		double spectralResidualNormalizationHighestMaxValue_ = 1500.;

		// dirt detection on image with artificial dirt
		cv::Mat color_image_with_artifical_dirt = C3_color_image.clone();
		cv::Mat mask_with_artificial_dirt = mask.clone();
		// add dirt
		int dirtSize = std::max(2, cvRound(3.0/640.0 * C3_color_image.cols));
		cv::Point2f ul(0.4375*C3_color_image.cols, 0.416666667*C3_color_image.rows);
		cv::Point2f ur(0.5625*C3_color_image.cols, 0.416666667*C3_color_image.rows);
		cv::Point2f ll(0.4375*C3_color_image.cols, 0.583333333*C3_color_image.rows);
		cv::Point2f lr(0.5625*C3_color_image.cols, 0.583333333*C3_color_image.rows);
		cv::ellipse(color_image_with_artifical_dirt, cv::RotatedRect(ul, cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
		cv::ellipse(mask_with_artificial_dirt, cv::RotatedRect(ul, cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
		cv::ellipse(color_image_with_artifical_dirt, cv::RotatedRect(lr, cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
		cv::ellipse(mask_with_artificial_dirt, cv::RotatedRect(lr, cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
		cv::ellipse(color_image_with_artifical_dirt, cv::RotatedRect(ll, cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(0, 0, 0), dirtSize);
		cv::ellipse(mask_with_artificial_dirt, cv::RotatedRect(ll, cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
		cv::ellipse(color_image_with_artifical_dirt, cv::RotatedRect(ur, cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(0, 0, 0), dirtSize);
		cv::ellipse(mask_with_artificial_dirt, cv::RotatedRect(ur, cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
		cv::Mat C1_saliency_image_with_artifical_dirt;
		SaliencyDetection_C3(color_image_with_artifical_dirt, C1_saliency_image_with_artifical_dirt, &mask_with_artificial_dirt, spectralResidualGaussianBlurIterations_);
		//cv::imshow("ai_dirt", color_image_with_artifical_dirt);

		// display of images with artificial dirt
		if (false)
			cv::imshow("color with artificial dirt", color_image_with_artifical_dirt);
		cv::Mat C1_saliency_image_with_artifical_dirt_scaled;
		double salminv, salmaxv;
		cv::Point2i salminl, salmaxl;
		cv::minMaxLoc(C1_saliency_image_with_artifical_dirt,&salminv,&salmaxv,&salminl,&salmaxl, mask_with_artificial_dirt);
		C1_saliency_image_with_artifical_dirt.convertTo(C1_saliency_image_with_artifical_dirt_scaled, -1, 1.0/(salmaxv-salminv), -1.0*(salminv)/(salmaxv-salminv));
		if (false)
			cv::imshow("saliency with artificial dirt", C1_saliency_image_with_artifical_dirt_scaled);

		// scale C1_saliency_image to value obtained from C1_saliency_image with artificially added dirt
		double minv, maxv;
		cv::Point2i minl, maxl;
		cv::minMaxLoc(C1_saliency_image_with_artifical_dirt,&minv,&maxv,&minl,&maxl, mask_with_artificial_dirt);
		cv::Scalar mean, stdDev;
		cv::meanStdDev(C1_saliency_image_with_artifical_dirt, mean, stdDev, mask);
		double newMaxVal = std::min(1.0, maxv/spectralResidualNormalizationHighestMaxValue_);///mean.val[0] / spectralResidualNormalizationHighestMaxMeanRatio_);
	//	std::cout << "dirtThreshold=" << dirtThreshold_ << "\tmin=" << minv << "\tmax=" << maxv << "\tmean=" << mean.val[0] << "\tstddev=" << stdDev.val[0] << "\tnewMaxVal (r)=" << newMaxVal << std::endl;


		////C1_saliency_image.convertTo(scaled_input_image, -1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));
		scaled_C1_saliency_image = C1_saliency_image.clone();	// square C1_saliency_image_with_artifical_dirt to emphasize the dirt and increase the gap to background response
		scaled_C1_saliency_image.convertTo(scaled_C1_saliency_image, -1, newMaxVal/(maxv-minv), -newMaxVal*(minv)/(maxv-minv));

		double newMean = mean.val[0] * newMaxVal/(maxv-minv) - newMaxVal*(minv)/(maxv-minv);
		double newStdDev = stdDev.val[0] * newMaxVal/(maxv-minv);
	//	std::cout << "newMean=" << newMean << "   newStdDev=" << newStdDev << std::endl;

	//	// scale C1_saliency_image
	//	cv::Mat scaled_C1_saliency_image;
	//	double minv, maxv;
	//	cv::Point2i minl, maxl;
	//	cv::minMaxLoc(C1_saliency_image,&minv,&maxv,&minl,&maxl, mask);
		cv::Mat badscale;
		double badminv, badmaxv;
		cv::Point2i badminl, badmaxl;
		cv::minMaxLoc(C1_saliency_image,&badminv,&badmaxv,&badminl,&badmaxl, mask);
		C1_saliency_image.convertTo(badscale, -1, 1.0/(badmaxv-badminv), -1.0*(badminv)/(badmaxv-badminv));
	//	std::cout << "bad scale:   " << "\tmin=" << badminv << "\tmax=" << badmaxv << std::endl;
		if (false)
		{
			cv::imshow("bad scale", badscale);
			cvMoveWindow("bad scale", 650, 0);
		}
		//cvMoveWindow("bad scale", 650, 520);
	//	cv::Scalar mean, stdDev;
	//	cv::meanStdDev(C1_saliency_image, mean, stdDev, mask);
	//	std::cout << "min=" << minv << "\tmax=" << maxv << "\tmean=" << mean.val[0] << "\tstddev=" << stdDev.val[0] << std::endl;
	//
	//	double newMaxVal = min(1.0, maxv/mean.val[0] /5.);
	//	//C1_saliency_image.convertTo(scaled_C1_saliency_image, -1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));
	//	C1_saliency_image.convertTo(scaled_C1_saliency_image, -1, newMaxVal/(maxv-minv), -newMaxVal*(minv)/(maxv-minv));

		// remove responses that lie on lines
		if (false)
		{
			cv::Mat src, dst, color_dst;

			cv::cvtColor(C3_color_image, src, CV_BGR2GRAY);

			// hack: autonomik
			//cv::Canny(src, dst, 150, 200, 3);
			cv::Canny(src, dst, 90, 170, 3);
			cv::cvtColor(dst, color_dst, CV_GRAY2BGR);

			std::vector<cv::Vec4i> lines;
			cv::HoughLinesP(dst, lines, 1, CV_PI/180, 80, 30, 10);
			for( size_t i = 0; i < lines.size(); i++ )
			{
				line(color_dst, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,255), 3, 8);
				line(scaled_C1_saliency_image, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,0), 13, 8);
				// todo: hack for autonomik
				//line(scaled_C1_saliency_image, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,0), 35, 8);
			}

			if (false)
			{
				cv::namedWindow("Detected Lines", 1);
				cv::imshow("Detected Lines", color_dst);
			}
		}

		if (true)
		{
			cv::imshow("saliency detection", scaled_C1_saliency_image);
			cvMoveWindow("saliency detection", 0, 530);
		}

		//set dirt pixel to white
		C1_BlackWhite_image = cv::Mat::zeros(C1_saliency_image.size(), CV_8UC1);
		cv::threshold(scaled_C1_saliency_image, C1_BlackWhite_image, dirtThreshold_, 1, cv::THRESH_BINARY);
	//	cv::threshold(scaled_C1_saliency_image, C1_BlackWhite_image, mean.val[0] + stdDev.val[0] * dirtCheckStdDevFactor_, 1, cv::THRESH_BINARY);

	//	std::cout << "(C1_saliency_image channels) = (" << C1_saliency_image.channels() << ")" << std::endl;

		cv::Mat CV_8UC_image;
		C1_BlackWhite_image.convertTo(CV_8UC_image, CV_8UC1);


	//	Mat dst = Mat::zeros(img.rows, img.cols, CV_8UC3);
	//	dst = C3_color_image;

		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;

		cv::findContours(CV_8UC_image, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

		cv::Scalar green(0, 255, 0);
		cv::Scalar red(0, 0, 255);
		for (int i = 0; i < (int)contours.size(); i++)
		{
			cv::RotatedRect rec = minAreaRect(contours[i]);
			double meanIntensity = 0;
			for (int t=0; t<(int)contours[i].size(); t++)
				meanIntensity += scaled_C1_saliency_image.at<float>(contours[i][t].y, contours[i][t].x);
			meanIntensity /= (double)contours[i].size();
			if (meanIntensity > newMean + dirtCheckStdDevFactor_ * newStdDev)
			{
				// todo: hack: for autonomik only detect green ellipses
				//dirtDetections.push_back(rec);
				cv::ellipse(C3_color_image, rec, green, 2);
			}
			else
				cv::ellipse(C3_color_image, rec, green, 2);	// todo: use red
			dirtDetections.push_back(rec);
		}
	}
};



int main(int argc, char **argv)
{

	ros::init(argc, argv, "appearance_check");

	ros::NodeHandle n;

	AppearanceCheck ac;
	ac.databaseTest();

	//ros::spin();

	return 0;
}
