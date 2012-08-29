/*
 * dirt_detection.h
 *
 *  Created on: 06.10.2011
 *      Author: rmb-hs
 */

#ifndef DIRT_DETECTION_H_
#define DIRT_DETECTION_H_

//##################
//#### includes ####

// standard includes
#include <iostream>
#include <fstream>
#include <string>
#include <deque>
#include <time.h>
#include <math.h>

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

// topics
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/ml/ml.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// PCL
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/io/pcd_io.h>

//bridge
#include <cv_bridge/cv_bridge.h>
//#include <cv_bridge/CvBridge.h>

namespace ipa_DirtDetection {

using namespace std;



/**
 *  Detects dirt from color image.
 */
class DirtDetection
{
protected:

	/**
	 * Used to subscribe and publish images.
	 */
	image_transport::ImageTransport* it_;

	/**
	 * Used to receive color image topic from camera.
	 */
	image_transport::Subscriber color_camera_image_sub_;
	/**
	 * Used to receive point cloud topic from camera.
	 */
	ros::Subscriber camera_depth_points_sub_;

	/**
	 * ROS node handle.
	 */
	ros::NodeHandle node_handle_;

	//parameters
	int spectralResidualGaussianBlurIterations_;
	double dirtThreshold_;
	double spectralResidualNormalizationHighestMaxValue_;
	double spectralResidualImageSizeRatio_;
	double dirtCheckStdDevFactor_;

	std::map<std::string, bool> debug_;


public:

	/**
	 * Needed to set pixel color.
	 */
	struct bgr
	{
		uchar b; /**< Blue channel value. */
		uchar g; /**< Green channel value. */
		uchar r; /**< Red channel value. */
	};


	/**
	 * Used to describe a carpet.
	 */
	struct CarpetFeatures
	{
		float min; 	/**< Minimum value in the "C1_saliency_image_with_artifical_dirt" image. */
		float max; 	/**< Maximum value in the "C1_saliency_image_with_artifical_dirt" image. */
		float mean; 	/**< Mean value in the "C1_saliency_image_with_artifical_dirt" image. */
		float stdDev; 	/**< Standard deviation in the "C1_saliency_image_with_artifical_dirt" image. */

	};

	/**
	 * Determines the class of a carpet.
	 */
	struct CarpetClass
	{
		float dirtThreshold;	/**< Carpet-label. */
	};

	struct NumStruc
	{
		int correctnum; /**< Number of correct classified samples. */
		int totalnum;	/**< Total number of samples of this class. */
	};


	/**
	 * Constructor.
	 */
	DirtDetection(ros::NodeHandle node_handle);

	/**
	 * Destructor.
	 */
	~DirtDetection();

	/**
	 * Create subscribers.
	 */
	void init();


	/**
	 * Function is called if color image topic is received.
	 *
	 * @param [in] color_image_msg	Color image message from camera.
	 *
	 */
	void imageDisplayCallback(const sensor_msgs::ImageConstPtr& color_image_msg);

	/**
	 * Function is called if point cloud topic is received.
	 *
	 * @param [in] point_cloud2_rgb_msg	Point cloude message from camera.
	 *
	 */
	void planeDetectionCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud2_rgb_msg);


	/**
	 * Converts: "sensor_msgs::Image::ConstPtr" \f$ \rightarrow \f$ "cv::Mat".
	 *	@param [in] 	color_image_msg 		Color image message from camera.
	 *	@param [in] 	color_image_ptr			See cv_bridge message to cv::Mat converter manual.
	 *	@param [out] 	color_image 			Color image from the message, in OpenCV representation.
	 */
	unsigned long convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& color_image_msg, cv_bridge::CvImageConstPtr& color_image_ptr, cv::Mat& color_image);


	/**
	 * Converts: "sensor_msgs::PointCloud2" \f$ \rightarrow \f$ "pcl::PointCloud<pcl::PointXYZRGB>::Ptr".
	 *	@param [in] 	point_cloud2_rgb_msg 		Point cloud message from camera.
	 *	@param [out] 	point_cloud_XYZRG 			Point cloud representation in PCL.
	 */
	void convertPointCloudMessageToPointCloudPcl(const sensor_msgs::PointCloud2ConstPtr& point_cloud2_rgb_msg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_XYZRGB);


	/**
	 * Converts: "sensor_msgs::PointCloud2" \f$ \rightarrow \f$ "pcl::PointCloud<pcl::PointXYZRGB>::Ptr".
	 *
	 * The function detects a plane in the point cloud, creates a mask for the plane pixels and sets all pixels in
	 * "plane_color_image" to their color if they lie in the plane, else to black.
	 *
	 *	@param [in] 	input_cloud 				Point cloud for plane detection.
	 *	@param [out] 	plane_color_image 			Shows the true color of all pixel within the plane. The size of the image is determined with the help of the point cloud!
	 *	@param [out]	plane_mask					Mask to separate plane pixels. Plane pixels are white (255), all other pixels are black (0).
	 *	@return 		True if any plane could be found in the image.
	 */
	bool planeSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, cv::Mat& plane_color_image, cv::Mat& plane_mask, pcl::ModelCoefficients& plane_model);


	/**
	 * This function performs the saliency detection to spot dirt stains.
	 *
	 * @param [in] 	C1_image				!!!ONE CHANNEL!!!('C1') image used to perfom the salciency detection.
	 * @param [out]	C1_saliency_image		One channel image('C1') which results from the saliency detection.
	 */
	void SaliencyDetection_C1(const cv::Mat& C1_image, cv::Mat& C1_saliency_image);


	/**
	 * This function performs a saliency detection for a  3 channel color image.
	 *
	 * The function proceeds as follows:
	 * 						1.) The 3 channel image is split into their 3 channels.
	 * 						2.) The saliency detection is performed for each channel.
	 * 						3.) The resulting images are add up.
	 *
	 * @param [in] 	C3_color_image			!!!THREE CHANNEL!!!('C3') image used to perform the saliency detection.
	 * @param [out]	C1_saliency_image		One channel image('C1') which results from the saliency detection.
	 * @param [in] 	mask					Determines the area of interest. Pixel of interests are white (255), all other pixels are black (0).
	 * @param [in]	gaussianBlurCycles		Determines the number of repetitions of the gaussian filter used to reduce the noise.
	 */
	void SaliencyDetection_C3(const cv::Mat& C3_color_image, cv::Mat& C1_saliency_image, const cv::Mat* mask = 0, int gaussianBlurCycles = 2);

	/**
	 * This function uses the "C1_saliency_image" to mark the dirt in the "C3_color_image".
	 * Furthermore, it returns an image ("C1_BlackWhite_image") in which all dirt pixels are white (255) and all other pixels are black (0).
	 *
	 *
	 * @param [in]		C1_saliency_image		One channel('C1') saliency image used for postprocessing.
	 * @param [out] 	C1_BlackWhite_image		One channel('C1') image in which all dirt pixels are white (255) and all other pixels are black (0).
	 * @param [in,out]	C3_color_image			Three channel('C3') color which corresponds to the "C1_saliency_image".
	 */
	void Image_Postprocessing_C1(const cv::Mat& C1_saliency_image, cv::Mat& C1_BlackWhite_image, cv::Mat& C3_color_image);


	/**
	 *
	 * This function avoids the false detections if no dirt is on the floor.
	 * Furthermore, it returns an image ("C1_BlackWhite_image") in which all dirt pixels are white (255) and all other pixels are black (0).
	 * It also the "C1_saliency_image" to mark the dirt in the "C3_color_image".
	 *
	 * @param [in] 		C1_saliency_image		One channel('C1') saliency image used for postprocessing.
	 * @param [out]		C1_BlackWhite_image		One channel('C1') image in which all dirt pixels are white (255) and all other pixels are black (0).
	 * @param [in,out] 	C3_color_image			Three channel('C3') color which corresponds to the "C1_saliency_image".
	 * @param [in]		mask					Determines the area of interest. Pixel of interests are white (255), all other pixels are black (0).
	 *
	 */
	void Image_Postprocessing_C1_rmb(const cv::Mat& C1_saliency_image, cv::Mat& C1_BlackWhite_image, cv::Mat& C3_color_image, const cv::Mat& mask = cv::Mat());


	/**
	 * This function is out of date and must not be used! The function is kept as backup copy.
	 *
	 * @param [in] color_image_msg		Color image message from camera.
	 */
	void SaliencyDetection_C1_old_cv_code(const sensor_msgs::ImageConstPtr& color_image_msg);

	/**
	 * This function creates/calculates a carpet-classifier for the given carpets. In this case an opencv support vector machine (SVM)
	 * is used as carpet-classifier.
	 *
	 * @param [in]	carp_feat_vec	Vector which contains the features of the different carpets.
	 * @param [in]  carp_class_vec	Vector containing the class specifier of the carpets.
	 * @param [out]	carpet_SVM		Returns an opencv support vector machine.
	 *
	 */
	void CreateCarpetClassiefierSVM(const std::vector<CarpetFeatures>& carp_feat_vec, const std::vector<CarpetClass>& carp_class_vec, CvSVM &carpet_SVM);


	/**
	 * This function creates/calculates a carpet-classifier for the given carpets. In this case an opencv tree model
	 * is used as carpet-classifier. PLEASE NOTE THAT THE OPENCV TREE LEARGNING ALGORITHMS DO NOT WORK PROPERLY!
	 *
	 * @param [in]	carp_feat_vec	Vector which contains the features of the different carpets.
	 * @param [in]  carp_class_vec	Vector containing the class specifier of the carpets.
	 * @param [out]	carpet_Tree		Returns an opencv tree model.
	 *
	 */
	void CreateCarpetClassiefierRTree(const std::vector<CarpetFeatures>& carp_feat_vec, const std::vector<CarpetClass>& carp_class_vec, CvRTrees &carpet_Tree);

	/**
	 * This function creates/calculates a carpet-classifier for the given carpets. In this case an opencv gradient boosted tree model
	 * is used as carpet-classifier. PLEASE NOTE THAT THE OPENCV TREE LEARGNING ALGORITHMS DO NOT WORK PROPERLY!
	 *
	 * @param [in]	carp_feat_vec	Vector which contains the features of the different carpets.
	 * @param [in]  carp_class_vec	Vector containing the class specifier of the carpets.
	 * @param [out]	carpet_GBTree	Returns an opencv gradient boosted tree model.
	 *
	 */
	void CreateCarpetClassiefierGBTree(const std::vector<CarpetFeatures>& carp_feat_vec, const std::vector<CarpetClass>& carp_class_vec,
			CvGBTrees &carpet_GBTree);


	/**
	 * This function illustrates how to use/implement openCV-SVM.
	 * The function has no other purpose than to illustrate how to use/implement openCV-SVM.
	 */
	void SVMExampleCode();


	/**
	 * This function illustrates how to use/implement openCV-SVM.
	 * The function has no other purpose than to illustrate how to use/implement openCV-SVM.
	 */
	void SVMTestFunction();

	/**
	 * Reads the carpet features and the class of the carpet from a file and saves them to the corresponding vectors.
	 * Please note that the data are added to the given vectors, in other words, the vectors are not reset!
	 *
	 * @param [in,out]	carp_feat_vec	Vector which contains the features of the different carpets.
	 * @param [in,out]	carp_class_vec 	Vector which saves the class of each carpet.
	 * @param [in]		filepath		The path to the given file.
	 * @param [in] 		filename		Name of the file which contains the different carpet data.
	 *
	 */
	void ReadDataFromCarpetFile(std::vector<CarpetFeatures>& carp_feat_vec, std::vector<CarpetClass>& carp_class_vec, std::string filepath, std::string filename);

	/**
	 * Splits the carpets, given by the corresponding feature and class vector, into: \n
	 * 1.) carpets which are used to train the different machine learning algorithms and \n
	 * 2.) carpets which are used to test the different algorithms.
	 *
	 * @param [in]		NumTestSamples	Number of carpets which are used to test the algorithms.
	 * @param [in]		input_feat_vec	Vector containing the features of the carpets which have to be split.
	 * @param [in]		input_class_vec Vector containing the class specifier of the carpets which have to be split..
	 * @param [in,out]	train_feat_vec	Vector containing the features of the carpets which are used to train the different algorithms. Please note that the data are added to the already existing data in the vector!
	 * @param [in,out] 	train_class_vec	Vector containing the class specifier of the carpets which are used to train the different algorithms. Please note that the data are added to the already existing data in the vector!
	 * @param [in,out]	test_feat_vec	Vector containing the features of the carpets which are used to test the different algorithms. Please note that the data are added to the already existing data in the vector!
	 * @param [in,out] 	test_class_vec	Vector containing the class specifier of the carpets which are used to test the different algorithms. Please note that the data are added to the already existing data in the vector!
	 *
	 */
	void SplitIntoTrainAndTestSamples(	int NumTestSamples,
										std::vector<CarpetFeatures>& input_feat_vec, std::vector<CarpetClass>& input_class_vec,
										std::vector<CarpetFeatures>& train_feat_vec, std::vector<CarpetClass>& train_class_vec,
										std::vector<CarpetFeatures>& test_feat_vec, std::vector<CarpetClass>& test_class_vec);

	/**
	 * This function can be used to test a specific opencv support vector machine. The function, however, is only usable if only one or two features are used to classify
	 * a carpet. For more features it is necessary to adapt the function!
	 *
	 * @param [in]	train_feat_vec	Vector containing the features of the carpets which are used to train the different algorithms.
	 * @param [in] 	train_class_vec	Vector containing the class specifier of the carpets which are used to train the different algorithms.
	 * @param [in]	test_feat_vec	Vector containing the features of the carpets which are used to test the different algorithms.
	 * @param [in] 	test_class_vec	Vector containing the class specifier of the carpets which are used to test the different algorithms.
	 * @param [in]	carpet_SVM		Opencv support vector machine.
	 *
	 */
	void SVMEvaluation(	std::vector<CarpetFeatures>& train_feat_vec, std::vector<CarpetClass>& train_class_vec,
						std::vector<CarpetFeatures>& test_feat_vec, std::vector<CarpetClass>& test_class_vec,
						CvSVM &carpet_SVM, double ScaleMean, double ScaleStd);


	/**
	 * This function can be used to test a specific opencv random tree model. The function, however, is only usable if only one or two features are used to classify
	 * a carpet. For more features it is necessary to adapt the function!
	 *
	 * @param [in]	train_feat_vec	Vector containing the features of the carpets which are used to train the different algorithms.
	 * @param [in] 	train_class_vec	Vector containing the class specifier of the carpets which are used to train the different algorithms.
	 * @param [in]	test_feat_vec	Vector containing the features of the carpets which are used to test the different algorithms.
	 * @param [in] 	test_class_vec	Vector containing the class specifier of the carpets which are used to test the different algorithms.
	 * @param [in]	carpet_Tree		Opencv random tree model.
	 *
	 */
	void RTreeEvaluation(	std::vector<CarpetFeatures>& train_feat_vec, std::vector<CarpetClass>& train_class_vec,
						std::vector<CarpetFeatures>& test_feat_vec, std::vector<CarpetClass>& test_class_vec,
						CvRTrees &carpet_Tree, double ScaleMean, double ScaleStd);

	/**
	 * This function can be used to test a specific opencv gradient boosted tree model. The function, however, is only usable if only one or two features are used to classify
	 * a carpet. For more features it is necessary to adapt the function!
	 *
	 * @param [in]	train_feat_vec	Vector containing the features of the carpets which are used to train the different algorithms.
	 * @param [in] 	train_class_vec	Vector containing the class specifier of the carpets which are used to train the different algorithms.
	 * @param [in]	test_feat_vec	Vector containing the features of the carpets which are used to test the different algorithms.
	 * @param [in] 	test_class_vec	Vector containing the class specifier of the carpets which are used to test the different algorithms.
	 * @param [in]	carpet_GBTree	Opencv gradient boosted tree model.
	 *
	 */
	void GBTreeEvaluation(	std::vector<CarpetFeatures>& train_feat_vec, std::vector<CarpetClass>& train_class_vec,
						std::vector<CarpetFeatures>& test_feat_vec, std::vector<CarpetClass>& test_class_vec,
						CvGBTrees &carpet_GBTree, double ScaleMean, double ScaleStd);

	/**
	 * This function scales the features to the interval [0,1]. It returns the scaled features and the
	 * scaling parameters.
	 *
	 * @param [in, out]	feat_vec 	Features which have to be scaled.
	 * @param [out]	maxMean			Scaling parameter for the mean values.
	 * @param [out]	maxStd			Scaling parameter for the standard deviation values.
	 *
	 */
	void ScaleSamples(std::vector<CarpetFeatures>& feat_vec,double & maxMean, double & maxStd);


};	//end-class

}; //end-namespace


#endif /* DIRT_DETECTION_H_ */
