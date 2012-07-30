#include <autopnp_dirt_detection/dirt_detection.h>

using namespace ipa_DirtDetection;
using namespace std;
using namespace cv;

/////////////////////////////////////////////////
// Constructor
/////////////////////////////////////////////////


DirtDetection::DirtDetection(ros::NodeHandle node_handle)
: node_handle_(node_handle)
{
	it_ = 0;

}

/////////////////////////////////////////////////
//  Destructor
/////////////////////////////////////////////////

DirtDetection::~DirtDetection()
{
	if (it_ != 0) delete it_;
}

/////////////////////////////////////////////////
// Create subscribers
/////////////////////////////////////////////////

void DirtDetection::init()
{
	it_ = new image_transport::ImageTransport(node_handle_);
//	color_camera_image_sub_ = it_->subscribe("image_color", 1, boost::bind(&DirtDetection::imageDisplayCallback, this, _1));
	camera_depth_points_sub_ =  node_handle_.subscribe<sensor_msgs::PointCloud2>("colored_point_cloud", 1, &DirtDetection::planeDetectionCallback, this);

	// Parameters
	std::cout << "\n--------------------------\nDirt Detection Parameters:\n--------------------------\n";
	node_handle_.param("spectralResidualGaussianBlurIterations", spectralResidualGaussianBlurIterations_, 2);
	std::cout << "spectralResidualGaussianBlurIterations = " << spectralResidualGaussianBlurIterations_ << std::endl;
	node_handle_.param("dirtThreshold", dirtThreshold_, 0.5);
	std::cout << "dirtThreshold = " << dirtThreshold_ << std::endl;
	node_handle_.param("spectralResidualNormalizationHighestMaxValue", spectralResidualNormalizationHighestMaxValue_, 0.5);
	std::cout << "spectralResidualNormalizationHighestMaxValue = " << spectralResidualNormalizationHighestMaxValue_ << std::endl;
	node_handle_.param("spectralResidualImageSizeRatio", spectralResidualImageSizeRatio_, 0.25);
	std::cout << "spectralResidualImageSizeRatio = " << spectralResidualImageSizeRatio_ << std::endl;
	node_handle_.param("dirtCheckStdDevFactor", dirtCheckStdDevFactor_, 2.5);
	std::cout << "dirtCheckStdDevFactor = " << dirtCheckStdDevFactor_ << std::endl;

}

/////////////////////////////////////////////////
// Main functions
/////////////////////////////////////////////////

int main(int argc, char **argv)
{

	ros::init(argc, argv, "dirt_detection");

	ros::NodeHandle n;

	DirtDetection id(n);
	id.init();

	printf("Read samples and split them into train-samples and test-samples.\n");
	std::vector<DirtDetection::CarpetFeatures> carp_feat_vec;
	std::vector<DirtDetection::CarpetClass> carp_class_vec;

	//data used to train the different learning algorithms
	std::vector<DirtDetection::CarpetFeatures> train_feat_vec;
	std::vector<DirtDetection::CarpetClass> train_class_vec;

	//used to test the different trained algorithms
	std::vector<DirtDetection::CarpetFeatures> test_feat_vec;
	std::vector<DirtDetection::CarpetClass> test_class_vec;


	//file path to carpet files
	std::string filepath = ros::package::getPath("autopnp_dirt_detection") + "/common/files/TeppichFiles/";

	std::string name;
	//vector of carpet file names
	std::vector<std::string> name_vec;

	name = "carpet-gray.tepp"; //threshold=0.18
	name_vec.push_back(name);

	name = "carpet2.tepp"; //threshold=0.25
	name_vec.push_back(name);

	name = "kitchen1a.tepp"; //threshold=0.2
	name_vec.push_back(name);

	name = "kitchen1b.tepp"; //threshold=0.2
	name_vec.push_back(name);

	name = "wood1.tepp"; //threshold=0.25
	name_vec.push_back(name);

	name = "wood2.tepp"; //threshold=0.25
	name_vec.push_back(name);

	name = "carpet6.tepp"; //threshold=0.25
	name_vec.push_back(name);

	name = "carpet4.tepp"; //threshold=0.3
	name_vec.push_back(name);

	name = "redblackpattern.tepp"; //threshold=0.35
	name_vec.push_back(name);


	for (unsigned int i = 0; i<name_vec.size(); i++)
	{
		carp_feat_vec.clear();
		carp_class_vec.clear();

		//read carpet data from current file
		id.ReadDataFromCarpetFile(carp_feat_vec, carp_class_vec, filepath, name_vec[i]);

		//split data into train and test samples
		id.SplitIntoTrainAndTestSamples(5, carp_feat_vec, carp_class_vec,
											train_feat_vec, train_class_vec,
											test_feat_vec, test_class_vec);
	}


	//determine maximum value of the mean value feature and the standard deviation feature
	//-> needed to draw the evaluation picture
	double maxMean;
	double maxStd;
	id.ScaleSamples(train_feat_vec, maxMean, maxStd);

	std::cout << "Data read and splitted.\n"  << std::endl;
	std::cout << "Total samples=" << train_feat_vec.size() + test_feat_vec.size()  << std::endl;
	std::cout << "Number of train-samples=" << train_feat_vec.size()  << std::endl;
	std::cout << "Number of test-samples=" << test_feat_vec.size()  << std::endl;

	// switchflag = 1; <-> SVM
	// switchflag = 2; <-> normal tree
	// switchflag = 3; <-> gradient boosted tree
	int switchflag = 3;

	switch (switchflag)
	{
		case 1: //<-> SVM
		{
			printf("Create SVM...\n");
			CvSVM carp_classi;
			id.CreateCarpetClassiefierSVM(train_feat_vec, train_class_vec, carp_classi);
			printf("SVM created.\n");

			printf("Check SVM...\n");
			id.SVMEvaluation(train_feat_vec, train_class_vec, test_feat_vec, test_class_vec, carp_classi, maxMean, maxStd);
			printf("Check SVM done.\n");

			break;
		}
		case 2: //<-> normal tree
		{
			printf("Create tree model...\n");
			CvRTrees rtree;
			id.CreateCarpetClassiefierRTree(train_feat_vec, train_class_vec, rtree);
			printf("Tree model created.\n");

			printf("Check Forest...\n");
			id.RTreeEvaluation(train_feat_vec, train_class_vec, test_feat_vec, test_class_vec, rtree, maxMean, maxStd);
			printf("Check tree model done.\n");

			break;
		}
		case 3: //<-> gradient boosted tree
		{
			printf("Create gradient boosted tree model...\n");
			CvGBTrees GBtree;
			id.CreateCarpetClassiefierGBTree(train_feat_vec, train_class_vec, GBtree);
			printf("Gradient boosted tree model created.\n");

			printf("Check gradient boosted tree model...\n");
			id.GBTreeEvaluation(train_feat_vec, train_class_vec, test_feat_vec, test_class_vec, GBtree, maxMean, maxStd);
			printf("Check gradient boosted tree model done.\n");

			break;
		}
	}

	//start to look for messages (loop)
	ros::spin();

	return 0;
}


/////////////////////////////////////////////////
// Callback functions
/////////////////////////////////////////////////

void DirtDetection::imageDisplayCallback(const sensor_msgs::ImageConstPtr& color_image_msg)
{
	cv_bridge::CvImageConstPtr color_image_ptr;
	cv::Mat color_image;
	//convert message to cv::Mat
	convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);

	// saliency detection
	cv::Mat result_image;
	SaliencyDetection_C3(color_image, result_image, 0, spectralResidualGaussianBlurIterations_);

	// post processing
	cv::Mat image_postproc;
	cv::Mat new_color_image = color_image;
	Image_Postprocessing_C1(result_image, image_postproc, new_color_image);

	cv::imshow("image postprocessing", new_color_image);
	cvMoveWindow("image postprocessing", 650, 0);

	cv::waitKey(10);
}

void DirtDetection::planeDetectionCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud2_rgb_msg)
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	convertPointCloudMessageToPointCloudPcl(point_cloud2_rgb_msg, input_cloud);

	// find ground plane
	cv::Mat plane_color_image = cv::Mat();
	cv::Mat plane_mask = cv::Mat();
	bool found_plane = planeSegmentation(input_cloud, plane_color_image, plane_mask);

	// check if a ground plane could be found
	if (found_plane == true)
	{
		//cv::cvtColor(plane_color_image, plane_color_image, CV_BGR2Lab);

//		cv::Mat laplace;
//		cv::Laplacian(plane_color_image, laplace, CV_32F, 5);
//		laplace = laplace.mul(laplace);
//		cv::normalize(laplace, laplace, 0, 1, NORM_MINMAX);
//		cv:imshow("laplace", laplace);


		// detect dirt on the floor
		cv::Mat C1_saliency_image;
		SaliencyDetection_C3(plane_color_image, C1_saliency_image, &plane_mask, spectralResidualGaussianBlurIterations_);

		// post processing, dirt/stain selection
		cv::Mat C1_BlackWhite_image;
		cv::Mat new_plane_color_image = plane_color_image.clone();
		Image_Postprocessing_C1_rmb(C1_saliency_image, C1_BlackWhite_image, new_plane_color_image, plane_mask);

		cv::imshow("image postprocessing", new_plane_color_image);
		cvMoveWindow("image postprocessing", 650, 0);
	}

	cv::waitKey(10);
}

/////////////////////////////////////////////////
// Convert functions
/////////////////////////////////////////////////

void DirtDetection::convertPointCloudMessageToPointCloudPcl(const sensor_msgs::PointCloud2ConstPtr& point_cloud2_rgb_msg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_XYZRGB)
{
	//conversion Ros message->Pcl point cloud
	pcl::fromROSMsg(*point_cloud2_rgb_msg, *point_cloud_XYZRGB);
}


unsigned long DirtDetection::convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& color_image_msg, cv_bridge::CvImageConstPtr& color_image_ptr, cv::Mat& color_image)
{
	try
	{
		color_image_ptr = cv_bridge::toCvShare(color_image_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("DirtDetection: cv_bridge exception: %s", e.what());
		return 1;
	}
	color_image = color_image_ptr->image;

	return 0;
}


bool DirtDetection::planeSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, cv::Mat& plane_color_image, cv::Mat& plane_mask)
{

	//recreate original color image from point cloud
	cv::Mat color_image = cv::Mat::zeros(input_cloud->height, input_cloud->width, CV_8UC3);
	int index = 0;
	for (int v=0; v<(int)input_cloud->height; v++)
	{
		for (int u=0; u<(int)input_cloud->width; u++, index++)
		{
			pcl::PointXYZRGB point = (*input_cloud)[index];
			bgr bgr_ = {point.b, point.g, point.r};
			color_image.at<bgr>(v, u) = bgr_;
		}
	}

	//display original image
	cv::imshow("color image", color_image);
	cvMoveWindow("color image", 0, 520);


	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.02);

	seg.setInputCloud(input_cloud);
	seg.segment (*inliers, *coefficients);

	bool found_plane = false;
	if (inliers->indices.size () != 0) //check if ground/floor detected
	{

		plane_color_image = cv::Mat::zeros(input_cloud->height, input_cloud->width, CV_8UC3);
		plane_mask = cv::Mat::zeros(input_cloud->height, input_cloud->width, CV_8UC1);
		for (size_t i=0; i<inliers->indices.size(); i++)
		{
			int v = inliers->indices[i]/input_cloud->width;	// check ob das immer abrundet ->noch offen!!!
			int u = inliers->indices[i] - v*input_cloud->width;

			pcl::PointXYZRGB point = (*input_cloud)[(inliers->indices[i])];
			bgr bgr_ = {point.b, point.g, point.r};
			plane_color_image.at<bgr>(v, u) = bgr_;
			plane_mask.at<uchar>(v, u) = 255;
		}

		//display detected floor
		//cv::imshow("floor cropped", ground_image);

		found_plane = true;
	}

	return found_plane;
}


void DirtDetection::SaliencyDetection_C1(const cv::Mat& C1_image, cv::Mat& C1_saliency_image)
{
	//given a one channel image
	//int scale = 6;
	//unsigned int size = (int)floor((float)pow(2.0,scale)); //the size to do the saliency at
	unsigned int size_cols = (int)(C1_image.cols * spectralResidualImageSizeRatio_);
	unsigned int size_rows = (int)(C1_image.rows * spectralResidualImageSizeRatio_);

	//create different images
	cv::Mat bw_im;
	cv::resize(C1_image,bw_im,cv::Size(size_cols,size_rows));

	cv::Mat realInput(size_rows,size_cols, CV_32FC1);	//calculate number of test samples
	int NumTestSamples = 10; //ceil(NumSamples*percentage_testdata);
	printf("Anzahl zu ziehender test samples: %d \n", NumTestSamples);
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
	std::vector<DirtDetection::CarpetFeatures> test_feat_vec;
	std::vector<DirtDetection::CarpetClass> test_class_vec;


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
	void GBTreeEvaluation(	std::vector<CarpetFeatures>& train_feat_vec, std::vector<CarpetClass>& train_class_vec,
						std::vector<CarpetFeatures>& test_feat_vec, std::vector<CarpetClass>& test_class_vec,
						CvGBTrees &carpet_GBTree);
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
}	std::vector<DirtDetection::CarpetFeatures> test_feat_vec;


void DirtDetection::SaliencyDetection_C3(const cv::Mat& C3_color_image, cv::Mat& C1_saliency_image, const cv::Mat* mask, int gaussianBlurCycles)
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
		cv::erode(mask_eroded, mask_eroded, cv::Mat(), cv::Point(-1, -1), 25);
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
	int NumTestSamples = 10; //ceil(NumSamples*percentage_testdata);
	printf("Anzahl zu ziehender test samples: %d \n", NumTestSamples);
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


void DirtDetection::Image_Postprocessing_C1(const cv::Mat& C1_saliency_image, cv::Mat& C1_BlackWhite_image, cv::Mat& C3_color_image)
{
	// scale input_image
	cv::Mat scaled_input_image;
	double minv, maxv;
	cv::Point2i minl, maxl;
	cv::minMaxLoc(C1_saliency_image,&minv,&maxv,&minl,&maxl);
	C1_saliency_image.convertTo(scaled_input_image, -1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));

	cv::imshow("SaliencyDetection", scaled_input_image);

	//set dirt pixel to white
	C1_BlackWhite_image = cv::Mat::zeros(C1_saliency_image.size(), CV_8UC1);
	cv::threshold(scaled_input_image, C1_BlackWhite_image, dirtThreshold_, 1, cv::THRESH_BINARY);

//	std::cout << "(C1_saliency_image channels) = (" << C1_saliency_image.channels() << ")" << std::endl;

	cv::Mat CV_8UC_image;
	C1_BlackWhite_image.convertTo(CV_8UC_image, CV_8UC1);


//	Mat dst = Mat::zeros(img.rows, img.cols, CV_8UC3);
//	dst = C3_color_image;

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    cv::findContours(CV_8UC_image, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    Scalar color(0, 255, 0);
    cv::RotatedRect rec;

    for (int i = 0; i < (int)contours.size(); i++)
    {
		rec = minAreaRect(contours[i]);
    	cv::ellipse(C3_color_image, rec, color, 2);
    }	//calculate number of test samples
	int NumTestSamples = 10; //ceil(NumSamples*percentage_testdata);
	printf("Anzahl zu ziehender test samples: %d \n", NumTestSamples);

}

void  DirtDetection::ReadDataFromCarpetFile(std::vector<CarpetFeatures>& carp_feat_vec, std::vector<CarpetClass>& carp_class_vec, std::string filepath, std::string filename)
{
	CarpetFeatures features;
	CarpetClass cc;

	std::ifstream indata;
	float data; // variable for input value

	std::string svmpath = filepath + filename;
	indata.open(svmpath.c_str()); // opens the file
	if(!indata) { // file couldn't be opened
	  cerr << "Error: file could not be opened" << endl;
	}

	int hnum = 0;
	int count = 1;
	indata >>data;
	while ( !indata.eof() ) { // keep reading until end-of-file
		switch ( count )
		{
			case 1:
			{
				cc.dirtThreshold = data;
				count++;
			  break;
			}
			case 2:
			{
				features.min = data;
				count++;
			  break;
			}
			case 3:
			{
				features.max = data;
				count++;
			  break;
			}
			case 4:
			{
				features.mean = data;
				count++;
			  break;
			}
			case 5:
			{
				features.stdDev = data;
				count = 1;

				carp_feat_vec.push_back(features);
				carp_class_vec.push_back(cc);

				hnum++;

//				std::cout << "Original Class=" << cc.dirtThreshold << "\tmean= " << features.mean
//						  << "\tstd= " << features.stdDev << std::endl;

			  break;
			}
		} //switch
	  indata >> data; // sets EOF flag if no value found
	} //while

	indata.close();

}


void DirtDetection::ScaleSamples(std::vector<CarpetFeatures>& feat_vec, double & maxMean, double & maxStd)
{
	//determine number of samples
	int NumSamples = feat_vec.size();

	maxMean = feat_vec[0].mean;
	maxStd = feat_vec[0].stdDev;

	for (int i = 1; i<NumSamples; i++)
	{
		if (feat_vec[i].mean > maxMean)
		{
			maxMean = feat_vec[i].mean;
		}//if

		if (feat_vec[i].stdDev > maxStd)
		{
			maxStd = feat_vec[i].stdDev;
		}//if

	} //for-i

	maxMean = maxMean + 30;
	maxStd = maxStd + 30;

	for (int i = 1; i<NumSamples; i++)
	{
		feat_vec[i].mean = feat_vec[i].mean/maxMean;
		feat_vec[i].stdDev = feat_vec[i].stdDev/maxStd;

//		std::cout << "New mean value=" << feat_vec[i].mean << std::endl;
//		std::cout << "New std value=" << feat_vec[i].stdDev << std::endl;

	} //for-i

}


void DirtDetection::SplitIntoTrainAndTestSamples(	int NumTestSamples,
													std::vector<CarpetFeatures>& input_feat_vec, std::vector<CarpetClass>& input_class_vec,
													std::vector<CarpetFeatures>& train_feat_vec, std::vector<CarpetClass>& train_class_vec,
													std::vector<CarpetFeatures>& test_feat_vec,  std::vector<CarpetClass>& test_class_vec)
{

	//determine number of samples
	int NumSamples = input_feat_vec.size();

	//initialize random seed
	srand ( time(NULL) );

	//helps to save numbers of test samples
	int SampledNumbers [NumTestSamples];
	for (int i = 0; i<NumTestSamples; i++)
	{
		SampledNumbers[i] = -1;
	}

	//loop variables
	int j = 0;
	bool foundflag = 0;

	for (int i = 0; i<NumTestSamples; i++)
	{

		//get valid sample number
		do
		{
			//generate random number in the range 0 to NumSamples-1
			SampledNumbers[i] =   rand() % NumSamples;

			//check if sample already used as test sample
			foundflag = 0;
			j = 0;
			while ( (foundflag == 0) && (j < i) )
			{

				if (SampledNumbers[i] == SampledNumbers[j])
				{
					foundflag = 1;
				} //if-end

				j++;

			} //while-do-end

		} while(foundflag == 1);

		//add test sample
		test_feat_vec.push_back(input_feat_vec[SampledNumbers[i]]);
		test_class_vec.push_back(input_class_vec[SampledNumbers[i]]);
	}

	for (int i = 0; i<NumSamples; i++ )
	{
		//check if sample already used as test sample
		foundflag = 0;
		j = 0;
		while ( (foundflag==0) && (j<NumTestSamples) )
		{
			if (i == SampledNumbers[j])
			{
				foundflag = 1;
			} //if-end

			j++;
		}

		//if sample not used as test sample, then use it as train sample
		if (foundflag == 0)
		{
			//add train sample
			train_feat_vec.push_back(input_feat_vec[i]);
			train_class_vec.push_back(input_class_vec[i]);
		}

	}

}

void DirtDetection::Image_Postprocessing_C1_rmb(const cv::Mat& C1_saliency_image, cv::Mat& C1_BlackWhite_image, cv::Mat& C3_color_image, const cv::Mat& mask)
{
	// dirt detection on image with artificial dirt
	cv::Mat color_image_with_artifical_dirt = C3_color_image.clone();
	cv::Mat mask_with_artificial_dirt = mask.clone();
	// add dirt
	int dirtSize = 3;
	cv::ellipse(color_image_with_artifical_dirt, cv::RotatedRect(cv::Point2f(280,200), cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
	cv::ellipse(mask_with_artificial_dirt, cv::RotatedRect(cv::Point2f(280,200), cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
	cv::ellipse(color_image_with_artifical_dirt, cv::RotatedRect(cv::Point2f(360,280), cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
	cv::ellipse(mask_with_artificial_dirt, cv::RotatedRect(cv::Point2f(360,280), cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
	cv::ellipse(color_image_with_artifical_dirt, cv::RotatedRect(cv::Point2f(280,280), cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(0, 0, 0), dirtSize);
	cv::ellipse(mask_with_artificial_dirt, cv::RotatedRect(cv::Point2f(280,280), cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
	cv::ellipse(color_image_with_artifical_dirt, cv::RotatedRect(cv::Point2f(360,200), cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(0, 0, 0), dirtSize);
	cv::ellipse(mask_with_artificial_dirt, cv::RotatedRect(cv::Point2f(360,200), cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
	cv::Mat C1_saliency_image_with_artifical_dirt;
	SaliencyDetection_C3(color_image_with_artifical_dirt, C1_saliency_image_with_artifical_dirt, &mask_with_artificial_dirt, spectralResidualGaussianBlurIterations_);
	//cv::imshow("ai_dirt", color_image_with_artifical_dirt);

	// scale C1_saliency_image to value obtained from C1_saliency_image with artificially added dirt
	std::cout << "res_img: " << C1_saliency_image_with_artifical_dirt.at<float>(300,200);
//	C1_saliency_image_with_artifical_dirt = C1_saliency_image_with_artifical_dirt.mul(C1_saliency_image_with_artifical_dirt);	// square C1_saliency_image_with_artifical_dirt to emphasize the dirt and increase the gap to background response
	std::cout << " -> " << C1_saliency_image_with_artifical_dirt.at<float>(300,200) << std::endl;
	double minv, maxv;
	cv::Point2i minl, maxl;
	cv::minMaxLoc(C1_saliency_image_with_artifical_dirt,&minv,&maxv,&minl,&maxl, mask_with_artificial_dirt);
	cv::Scalar mean, stdDev;
	cv::meanStdDev(C1_saliency_image_with_artifical_dirt, mean, stdDev, mask);
	double newMaxVal = min(1.0, maxv/spectralResidualNormalizationHighestMaxValue_);///mean.val[0] / spectralResidualNormalizationHighestMaxMeanRatio_);
	std::cout << "dirtThreshold=" << dirtThreshold_ << "\tmin=" << minv << "\tmax=" << maxv << "\tmean=" << mean.val[0] << "\tstddev=" << stdDev.val[0] << "\tnewMaxVal=" << newMaxVal << std::endl;


	//determine ros package path
	std::string svmpath = ros::package::getPath("autopnp_dirt_detection") + "/common/files/svm/Teppich1.tepp";
	ofstream teppichfile;
	teppichfile.open (svmpath.c_str(), ios::out| ios::app);
	teppichfile << dirtThreshold_ << "\t\t" << minv << "\t\t" << maxv << "\t\t" << mean.val[0] << "\t\t" << stdDev.val[0] << "\n";
	teppichfile.close();


	////C1_saliency_image.convertTo(scaled_input_image, -1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));
	cv::Mat scaled_C1_saliency_image = C1_saliency_image.clone();	// square C1_saliency_image_with_artifical_dirt to emphasize the dirt and increase the gap to background response
	//scaled_C1_saliency_image = scaled_C1_saliency_image.mul(scaled_C1_saliency_image);
	scaled_C1_saliency_image.convertTo(scaled_C1_saliency_image, -1, newMaxVal/(maxv-minv), -newMaxVal*(minv)/(maxv-minv));

	double newMean = mean.val[0] * newMaxVal/(maxv-minv) - newMaxVal*(minv)/(maxv-minv);
	double newStdDev = stdDev.val[0] * newMaxVal/(maxv-minv);
	std::cout << "newMean=" << newMean << "   newStdDev=" << newStdDev << std::endl;

//	// scale C1_saliency_image
//	cv::Mat scaled_C1_saliency_image;
//	double minv, maxv;
//	cv::Point2i minl, maxl;
//	cv::minMaxLoc(C1_saliency_image,&minv,&maxv,&minl,&maxl, mask);
	cv::Mat badscale;
	C1_saliency_image.convertTo(badscale, -1, 1.0/(maxv-minv), -1.0*(minv)/(maxv-minv));
	cv::imshow("bad scale", badscale);
	cvMoveWindow("bad scale", 650, 520);
//	cv::Scalar mean, stdDev;
//	cv::meanStdDev(C1_saliency_image, mean, stdDev, mask);
//	std::cout << "min=" << minv << "\tmax=" << maxv << "\tmean=" << mean.val[0] << "\tstddev=" << stdDev.val[0] << std::endl;
//
//	double newMaxVal = min(1.0, maxv/mean.val[0] /5.);
//	//C1_saliency_image.convertTo(scaled_C1_saliency_image, -1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));
//	C1_saliency_image.convertTo(scaled_C1_saliency_image, -1, newMaxVal/(maxv-minv), -newMaxVal*(minv)/(maxv-minv));

	cv::imshow("SaliencyDetection", scaled_C1_saliency_image);
	cvMoveWindow("SaliencyDetection", 0, 0);

	//set dirt pixel to white
	C1_BlackWhite_image = cv::Mat::zeros(C1_saliency_image.size(), CV_8UC1);
	cv::threshold(scaled_C1_saliency_image, C1_BlackWhite_image, dirtThreshold_, 1, cv::THRESH_BINARY);
//	cv::threshold(scaled_C1_saliency_image, C1_BlackWhite_image, mean.val[0] + stdDev.val[0] * dirtCheckStdDevFactor_, 1, cv::THRESH_BINARY);

//	std::cout << "(C1_saliency_image channels) = (" << C1_saliency_image.channels() << ")" << std::endl;

	cv::Mat CV_8UC_image;
	C1_BlackWhite_image.convertTo(CV_8UC_image, CV_8UC1);


//	Mat dst = Mat::zeros(img.rows, img.cols, CV_8UC3);
//	dst = C3_color_image;

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    cv::findContours(CV_8UC_image, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    cv::Scalar green(0, 255, 0);
    cv::Scalar red(0, 0, 255);
    cv::RotatedRect rec;

    for (int i = 0; i < (int)contours.size(); i++)
    {
    	double meanIntensity = 0;
    	for (int t=0; t<(int)contours[i].size(); t++)
    		meanIntensity += scaled_C1_saliency_image.at<float>(contours[i][t].y, contours[i][t].x);
    	meanIntensity /= (double)contours[i].size();
    	if (meanIntensity > newMean + dirtCheckStdDevFactor_ * newStdDev)
    	{
			rec = minAreaRect(contours[i]);
			cv::ellipse(C3_color_image, rec, green, 2);
    	}
    	else
    	{
    		rec = minAreaRect(contours[i]);
    		cv::ellipse(C3_color_image, rec, red, 2);
    	}
    }

}


void DirtDetection::SaliencyDetection_C1_old_cv_code(const sensor_msgs::ImageConstPtr& color_image_msg)
{
	cv_bridge::CvImageConstPtr color_image_ptr;
	cv::Mat color_image;
	convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);

	// color_image is now available with the current image from your camera

	cv::imshow("image", color_image);
	//cv::waitKey(10);

	//int thresh = 350;
	int scale = 6;

	//convert color image to one channel gray-scale image
	//color_image -> gray_image
    IplImage hsrc = color_image;
    IplImage *src = &hsrc;

    /* get image properties */
    int width  = src->width;
    int height = src->height;

    /* create new image for the grayscale version */
    IplImage * gray_image = cvCreateImage( cvSize( width, height ), IPL_DEPTH_8U, 1 );

    /* CV_RGB2GRAY: convert RGB image to grayscale */
    cvCvtColor( src, gray_image, CV_RGB2GRAY );


    cv::Mat bild(gray_image);

	cv::imshow("image3", bild);
	//cv::waitKey(10);


	//IplImage imageIpl = (IplImage) gray_image;
	IplImage* image = gray_image; //& imageIpl;

	//given a one channel image
	unsigned int size = (int)floor((float)pow(2.0,scale)); //the size to do the saliency at

	IplImage* bw_im = cvCreateImage(cvSize(size,size), IPL_DEPTH_8U,1);
	cvResize(image, bw_im);
	IplImage* realInput = cvCreateImage( cvGetSize(bw_im), IPL_DEPTH_32F, 1);
	IplImage* imaginaryInput = cvCreateImage( cvGetSize(bw_im), IPL_DEPTH_32F, 1);
	IplImage* complexInput = cvCreateImage( cvGetSize(bw_im), IPL_DEPTH_32F, 2);

	cvScale(bw_im, realInput, 1.0/255.0);
	cvZero(imaginaryInput);
	cvMerge(realInput, imaginaryInput, NULL, NULL, complexInput);
	CvMat* dft_A = cvCreateMat( size, size, CV_32FC2 );

	// copy A to dft_A and pad dft_A with zeros
	CvMat tmp;
	cvGetSubRect( dft_A, &tmp, cvRect(0,0, size,size));
	cvCopy( complexInput, &tmp );
	// cvZero(&tmp);

	cvDFT( dft_A, dft_A, CV_DXT_FORWARD, size );
	cvSplit( dft_A, realInput, imaginaryInput, NULL, NULL );
	// Compute the phase angle
	IplImage* image_Mag = cvCreateImage(cvSize(size, size), IPL_DEPTH_32F, 1);
	IplImage* image_Phase = cvCreateImage(cvSize(size, size), IPL_DEPTH_32F, 1);


	//compute the phase of the spectrum
	cvCartToPolar(realInput, imaginaryInput, image_Mag, image_Phase, 0);

	IplImage* log_mag = cvCreateImage(cvSize(size, size), IPL_DEPTH_32F, 1);
	cvLog(image_Mag, log_mag);

	//Box filter the magnitude, then take the difference
	IplImage* log_mag_Filt = cvCreateImage(cvSize(size, size), IPL_DEPTH_32F, 1);
	CvMat* filt = cvCreateMat(3,3, CV_32FC1);
	cvSet(filt,cvScalarAll(1.0/9.0));
	cvFilter2D(log_mag, log_mag_Filt, filt);
	cvReleaseMat(&filt);

	cvSub(log_mag, log_mag_Filt, log_mag);

	cvExp(log_mag, image_Mag);

	cvPolarToCart(image_Mag, image_Phase, realInput, imaginaryInput,0);
	//cvExp(log_mag, image_Mag);

	cvMerge(realInput, imaginaryInput, NULL, NULL, dft_A);
	cvDFT( dft_A, dft_A, CV_DXT_INV_SCALE, size);

	cvAbs(dft_A, dft_A);
	cvMul(dft_A,dft_A, dft_A);
	cvGetSubRect( dft_A, &tmp, cvRect(0,0, size,size));
	cvCopy( &tmp, complexInput);
	cvSplit(complexInput, realInput, imaginaryInput, NULL,NULL);

	IplImage* result_image = cvCreateImage(cvGetSize(image),IPL_DEPTH_32F, 1);
	double minv, maxv;
	CvPoint minl, maxl;
	cvSmooth(realInput,realInput);
	cvSmooth(realInput,realInput);
	cvMinMaxLoc(realInput,&minv,&maxv,&minl,&maxl);
	//printf("Max value %lf, min %lf\n", maxv,minv);
	maxv= 0.03;
	cvScale(realInput, realInput, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));
	cvResize(realInput, result_image);


	cv::Mat resultImage(result_image);


	// remove saliency at the image border
	int borderX = resultImage.cols/20;
	int borderY = resultImage.rows/20;
	if (borderX > 0 && borderY > 0)
	{
	cv::Mat smallImage_ = resultImage.colRange(borderX, resultImage.cols-1-borderX);
	cv::Mat smallImage = smallImage_.rowRange(borderY, smallImage_.rows-1-borderY);

	copyMakeBorder(smallImage, resultImage, borderY, borderY, borderX, borderX, cv::BORDER_CONSTANT, cv::Scalar(0));
	}

	//CvMat -> cv::Mat
	//cv::meanStdDev();

	//cv::Mat stainImage;
	// check if the image makes sense (or convert to 0...255)
	//resultImage.convertTo(stainImage, -1, 1.0, 0.0);


	//mThresholdHou = thresh/100.0*cvAvg(realInput).val[0];

	cv::imshow("image2", resultImage);
	cv::waitKey(10);


	//stainImage.release();

	resultImage.release();
	cvReleaseImage(&result_image);
	cvReleaseImage(&realInput);
	cvReleaseImage(&imaginaryInput);
	cvReleaseImage(&complexInput);
	cvReleaseMat(&dft_A);
	cvReleaseImage(&bw_im);

	cvReleaseImage(&image_Mag);
	cvReleaseImage(&image_Phase);

	cvReleaseImage(&log_mag);
	cvReleaseImage(&log_mag_Filt);
	cvReleaseImage(&bw_im);

}

void DirtDetection::CreateCarpetClassiefierSVM(const std::vector<CarpetFeatures>& carp_feat_vec,
											const std::vector<CarpetClass>& carp_class_vec,
											CvSVM &carpet_SVM)
{

	/////////////////////////////////////////////////////////////////////////////
	///Old style code-> necessary because svm does not work with c++ style!!!///
	///////////////////////////////////////////////////////////////////////////

	//determine number of samples
	int Nges = carp_feat_vec.size();

	//number of features
	int NCarpetFeatures = 2;

	int i;
	float *data = 0;
	data = new float [Nges*2];

	float *res = 0;
	res = new float [Nges];

	CvMat data_mat, res_mat;

	for (i = 0; i < Nges; i++)
	{
	  data[i * 2] = float(carp_feat_vec[i].mean);
	  data[i * 2 + 1] = float(carp_feat_vec[i].stdDev);
	  res[i] = carp_class_vec[i].dirtThreshold;
//	  std::cout << 	"threshold= " << res[i] << std::endl;
	}

	cvInitMatHeader (&data_mat, Nges, NCarpetFeatures, CV_32FC1, data);
	cvInitMatHeader (&res_mat, Nges, 1, CV_32FC1, res);

//    // Set up SVM's parameters
    CvSVMParams params;
    params.svm_type    = CvSVM::NU_SVR;
    params.kernel_type = CvSVM::RBF;//POLY;

    params.degree = 1.0;
    params.gamma = 1.0;
    params.coef0 = 1.0;
    params.C = 3;
    params.nu = 0.99;
    params.p = 1;
    params.class_weights = NULL;
    params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
    params.term_crit = cvTermCriteria (CV_TERMCRIT_EPS, 100, FLT_EPSILON);

	//define testing area for "train_auto()":
    cv::ParamGrid gamma_grid(0.1, 1, 2);
    cv::ParamGrid C_grid(1, 10, 2 );
    cv::ParamGrid nu_grid(0.01, 0.2, 2 );
    cv::ParamGrid degree_grid(0.1, 1, 2);

	//determine optimal parameters and train SVM
	//Comment: It's important to define all input parameters because otherwise "SVM.train_auto" might not work properly!!!
	carpet_SVM.train_auto(	&data_mat, &res_mat, NULL, NULL, params, 10,
							CvSVM::get_default_grid(CvSVM::C), //C_grid,
							CvSVM::get_default_grid(CvSVM::GAMMA), //gamma_grid,
							CvSVM::get_default_grid(CvSVM::P),
							CvSVM::get_default_grid(CvSVM::NU), //nu_grid,
							CvSVM::get_default_grid(CvSVM::COEF),
							CvSVM::get_default_grid(CvSVM::DEGREE),
							false );

	//get SVM parameter
	params = carpet_SVM.get_params();

	//display SVM parameters on screen
	std::cout << 	"C = " << params.C << "\ncoeff0 = " << params.coef0 << "\ndegree = " <<
					params.degree << "\ngamma = " << params.gamma << "\nnu = " << params.nu <<
					"\np = " << params.p << std::endl;

	//freigabe
	delete[] data;
	delete[] res;

}

void DirtDetection::SVMEvaluation(	std::vector<CarpetFeatures>& train_feat_vec, std::vector<CarpetClass>& train_class_vec,
									std::vector<CarpetFeatures>& test_feat_vec, std::vector<CarpetClass>& test_class_vec,
									CvSVM &carpet_SVM, double ScaleMean, double ScaleStd)
{

	//number of test samples
	int NumTestSamples = test_feat_vec.size();
	int NumTrainSamples = train_feat_vec.size();

	//saves the different class types
	std::vector<CarpetClass> ClassTypes_vec;

	//number of different class types
	int NumClass = 0;

	//max mean value
	double maxmean = -1;
	//max std. dev.
	double maxstd = -1;

	//loop variables
	bool foundflag = 0;
	int j = 0;

	bool imageOnFlag = 1;

	for(int i = 0; i< NumTrainSamples; i++)
	{
		//test if mean or stdDev bigger than saved values
		if (train_feat_vec[i].mean > maxmean ){maxmean = train_feat_vec[i].mean;}
		if (train_feat_vec[i].stdDev > maxstd ){maxstd = train_feat_vec[i].stdDev;}

		//Determine if new class type
		foundflag = 0;
		j = 0;
		while ((foundflag == 0) && (j<NumClass))
		{
			if (ClassTypes_vec[j].dirtThreshold==train_class_vec[i].dirtThreshold)
			{
				foundflag = 1;
			}
			j++;
		}

		//if new class type, then save it in the list
		if (foundflag == 0)
		{
			ClassTypes_vec.push_back(train_class_vec[i]);
			NumClass++;
		}

	}

	//saves the total number and the number of correct classified samples of each class
	std::vector<NumStruc> num_vec;

	//initialise num_vec with void values
	NumStruc num_struc;
	num_struc.totalnum = 0;
	num_struc.correctnum = 0;

	for(int i = 0; i<NumClass; i++)
	{
		num_vec.push_back(num_struc);
	}


	int x,y;

	//save the prediction of the SVM
	//"predicted current class"
	float pcc;

	//Image for visual representation
	int width = 700, height = 700;

	cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
	cv::Mat image2 = cv::Mat::zeros(height, width, CV_8UC3);

	double kx1 = width/(1.2*maxmean);
	double kx2 = height/(1.2*maxstd);

	Vec3b green(0,255,0), blue (255,0,0), red (0,0,255);

//	cv::Mat sampleMat = cv::Mat::zeros(1, 2, CV_32FC1);
	cv::Mat sampleMat(1, 2, CV_32FC1);

	if (imageOnFlag == 1)
	{
		// determine class of each image pixel
		for (int i = 0; i < (image.rows); ++i)
			for (int j = 0; j < (image.cols); ++j)
			{
				x = i/kx1;
				y = j/kx2;


				//predict sample class for image pixel <-> old code style
				CvMat m;
			    float a[] = { float(x/ScaleMean), float(y/ScaleStd) };
				cvInitMatHeader (&m, 1, 2, CV_32FC1, a);
				pcc = 700*carpet_SVM.predict (&m);


				//plot image pixel
				image.at<Vec3b>(j,i) = Vec3b( pcc, pcc, pcc);

				//plot image pixel
				image2.at<Vec3b>(j, i)  = blue;

			}
	}

	//determines the "area" which still belongs to a certain class
	double r = 0.03;

	//"current true class type"
	double ctct;

	for (int i=0; i<NumTestSamples; i++)
	{
		//////////////////
		///image stuff///
		////////////////

			//determine true sample class
			ctct = test_class_vec[i].dirtThreshold;

			if (imageOnFlag == 1)
			{
				x = ceil(kx1*test_feat_vec[i].mean);
				y = ceil(kx2*test_feat_vec[i].stdDev);

				//daw samples if their true class color to the image (as circles)
				circle( image, Point(x,y), 3, Scalar(floor(ctct*700.0), floor(ctct*700.0), floor(ctct*700.0)), -1 , 8);
			}

		///////////////////////////////////
		///percentage calculation stuff///
		/////////////////////////////////


			//predict sample class
			CvMat m;
		    float a[] = { float(test_feat_vec[i].mean/ScaleMean), float(test_feat_vec[i].stdDev/ScaleStd) };
			cvInitMatHeader (&m, 1, 2, CV_32FC1, a);
			float pcc = carpet_SVM.predict (&m);

			//show result
			std::cout << "True threshold=" << ctct << "\tPredicted threshold=" << pcc
					  << "\tmean=" << x << "\tstd=" << y << std::endl;

			//determine class index
			j = 0;
			while (ClassTypes_vec[j].dirtThreshold != ctct)
			{
				j++;
			}

			//increase the total number of test samples for this class
			num_vec[j].totalnum++;

			//check if test sample in class "area"
			if ((pcc > ctct-r) && (pcc < ctct+r))
//			if (((pcc == 1) && (ctct == 0.25)) || ((pcc == 2) && (ctct == 0.35)))
			{
				num_vec[j].correctnum++;
			}

	} //end-for


	///////////////////////////////////////////////
	///plot position and class of train samples///
	/////////////////////////////////////////////

	if (imageOnFlag == 1)
	{
		for (int i=0; i<NumTrainSamples; i++)
		{
			//determine true sample class
			ctct = train_class_vec[i].dirtThreshold;

			x = ceil(kx1*train_feat_vec[i].mean*ScaleMean);
			y = ceil(kx2*train_feat_vec[i].stdDev*ScaleStd);

			//daw samples if their true class color to the image (as circles)
			circle( image2, Point(x,y), 3, Scalar(floor(ctct*700.0), floor(ctct*700.0), floor(ctct*700.0)), -1 , 8);
		}
	}

	////////////////////////////////////////
	///plot percentage calculation stuff///
	//////////////////////////////////////

	double res = 0;

	for (int i=0; i<NumClass; i++)
	{
		res = 100*(double(num_vec[i].correctnum)/double(num_vec[i].totalnum));

		std::cout 	<< "Dirt threshold (class)=" << ClassTypes_vec[i].dirtThreshold
					<< "\tCorrect/total test-samples=" << num_vec[i].correctnum << "/" << num_vec[i].totalnum
					<< "\tPercentage=" << res <<std::endl;

	}


	///////////////////////
	///show image stuff///
	/////////////////////

	if (imageOnFlag == 1)
	{
		//show image on screen
		imshow("SVM evaluation image", image);
		imshow("Sample image", image2);
		waitKey(0);
	}
}

void DirtDetection::CreateCarpetClassiefierRTree(const std::vector<CarpetFeatures>& carp_feat_vec, const std::vector<CarpetClass>& carp_class_vec, CvRTrees &carpet_Tree)
{
	/////////////////////////////////////////////////////////////////////////////
	///Old style code-> necessary because svm does not work with c++ style!!!///
	///////////////////////////////////////////////////////////////////////////

	//determine number of samples
	int Nges = carp_feat_vec.size();

	//number of features
	int NCarpetFeatures = 2;

	int i;
	float *data = 0;
	data = new float [Nges*2];

	float *res = 0;
	res = new float [Nges];

	CvMat data_mat, res_mat;

	for (i = 0; i < Nges; i++)
	{
	  data[i * 2] = float(carp_feat_vec[i].mean);
	  data[i * 2 + 1] = float(carp_feat_vec[i].stdDev);
	  res[i] = carp_class_vec[i].dirtThreshold;
//	  std::cout << 	"threshold= " << res[i] << std::endl;
	}

	cvInitMatHeader (&data_mat, Nges, NCarpetFeatures, CV_32FC1, data);
	cvInitMatHeader (&res_mat, Nges, 1, CV_32FC1, res);

    // define all the attributes as numerical
    // alternatives are CV_VAR_CATEGORICAL or CV_VAR_ORDERED(=CV_VAR_NUMERICAL)
    // that can be assigned on a per attribute basis
	cv::Mat var_type = cv::Mat(NCarpetFeatures+1, 1, CV_8U );
	var_type.setTo(Scalar(CV_VAR_NUMERICAL) ); // all inputs are numerical

	CvMat varT2 = var_type;

	//////////////////
	//Create-Forest//
	////////////////

    CvRTParams params = CvRTParams(10, // max depth
                                   10, // min sample count
                                   0.00000001f, // regression accuracy
                                   false, // compute surrogate split, no missing data
                                   10, // max number of categories (use sub-optimal algorithm for larger numbers)
                                   0, // the array of priors
                                   true,  // calculate variable importance
                                   0,       // number of variables randomly selected at node and used to find the best split(s).
                                   100,	 // max number of trees in the forest
                                   0.1f,				// forrest accuracy
                                   CV_TERMCRIT_ITER | CV_TERMCRIT_EPS// termination cirteria
                                  );

    //train tree
    carpet_Tree.train(&data_mat, CV_ROW_SAMPLE, &res_mat, NULL, NULL, &varT2, NULL, params);

	//freigabe
	delete[] data;
	delete[] res;

}

void DirtDetection::CreateCarpetClassiefierGBTree(const std::vector<CarpetFeatures>& carp_feat_vec,
		const std::vector<CarpetClass>& carp_class_vec, CvGBTrees &carpet_GBTree)
{
	/////////////////////////////////////////////////////////////////////////////
	///Old style code-> necessary because svm does not work with c++ style!!!///
	///////////////////////////////////////////////////////////////////////////

	//determine number of samples
	int Nges = carp_feat_vec.size();

	//number of features
	int NCarpetFeatures = 2;

	int i;
	float *data = 0;
	data = new float [Nges*2];

	float *res = 0;
	res = new float [Nges];

	CvMat data_mat, res_mat;

	for (i = 0; i < Nges; i++)
	{
	  data[i * 2] = float(carp_feat_vec[i].mean);
	  data[i * 2 + 1] = float(carp_feat_vec[i].stdDev);
	  res[i] = carp_class_vec[i].dirtThreshold;
//	  std::cout << 	"threshold= " << res[i] << std::endl;
	}

	cvInitMatHeader (&data_mat, Nges, NCarpetFeatures, CV_32FC1, data);
	cvInitMatHeader (&res_mat, Nges, 1, CV_32FC1, res);

    // define all the attributes as numerical
    // alternatives are CV_VAR_CATEGORICAL or CV_VAR_ORDERED(=CV_VAR_NUMERICAL)
    // that can be assigned on a per attribute basis
	cv::Mat var_type = cv::Mat(NCarpetFeatures+1, 1, CV_8U );
	var_type.setTo(Scalar(CV_VAR_NUMERICAL) ); // all inputs are numerical

	//transform to mat to cvmat
	CvMat varT2 = var_type;

	//////////////////
	//Create-Forest//
	////////////////


    CvGBTreesParams params = CvGBTreesParams(
									CvGBTrees::SQUARED_LOSS, // CvGBTrees::SQUARED_LOSS, CvGBTrees::ABSOLUTE_LOSS, CvGBTrees::HUBER_LOSS  <-> different possible loss functions for regression problems
									10, 	// Count of boosting algorithm iterations. weak_count*K is the total count of trees in the GBT model, where K is the output classes count (equal to one in case of a regression).
									0.3, 	//shrinkage – Regularization parameter: (0,1]
									0.3f, 	//subsample_portion – Portion of the whole training set used for each algorithm iteration. Subset is generated randomly.
									5, 		//max_depth – Maximal depth of each decision tree in the ensemble
									false	//use_surrogates – If true, surrogate splits are built
										    );

    // train random forest classifier (using training data)

    carpet_GBTree.train(&data_mat, CV_ROW_SAMPLE, &res_mat, NULL, NULL, &varT2, NULL, params, false);


	//freigabe
	delete[] data;
	delete[] res;

}


void DirtDetection::RTreeEvaluation(std::vector<CarpetFeatures>& train_feat_vec, std::vector<CarpetClass>& train_class_vec,
									std::vector<CarpetFeatures>& test_feat_vec, std::vector<CarpetClass>& test_class_vec,
									CvRTrees &carpet_Tree, double ScaleMean, double ScaleStd)
{

	//number of test samples
	int NumTestSamples = test_feat_vec.size();
	int NumTrainSamples = train_feat_vec.size();

	//saves the different class types
	std::vector<CarpetClass> ClassTypes_vec;

	//number of different class types
	int NumClass = 0;

	//max mean value
	double maxmean = -1;
	//max std. dev.
	double maxstd = -1;

	//loop variables
	bool foundflag = 0;
	int j = 0;

	bool imageOnFlag = 1;

	for(int i = 0; i< NumTrainSamples; i++)
	{
		//test if mean or stdDev bigger than saved values
		if (train_feat_vec[i].mean > maxmean ){maxmean = train_feat_vec[i].mean;}
		if (train_feat_vec[i].stdDev > maxstd ){maxstd = train_feat_vec[i].stdDev;}

		//Determine if new class type
		foundflag = 0;
		j = 0;
		while ((foundflag == 0) && (j<NumClass))
		{
			if (ClassTypes_vec[j].dirtThreshold==train_class_vec[i].dirtThreshold)
			{
				foundflag = 1;
			}
			j++;
		}

		//if new class type, then save it in the list
		if (foundflag == 0)
		{
			ClassTypes_vec.push_back(train_class_vec[i]);
			NumClass++;
		}

	}

	//saves the total number and the number of correct classified samples of each class
	std::vector<NumStruc> num_vec;

	//initialise num_vec with void values
	NumStruc num_struc;
	num_struc.totalnum = 0;
	num_struc.correctnum = 0;

	for(int i = 0; i<NumClass; i++)
	{
		num_vec.push_back(num_struc);
	}


	int x,y;

	//save the prediction of the SVM
	//"predicted current class"
	float pcc;

	//Image for visual representation
	int width = 700, height = 700;

	cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
	cv::Mat image2 = cv::Mat::zeros(height, width, CV_8UC3);

	double kx1 = width/(1.2*maxmean);
	double kx2 = height/(1.2*maxstd);

	Vec3b green(0,255,0), blue (255,0,0), red (0,0,255);

//	cv::Mat sampleMat = cv::Mat::zeros(1, 2, CV_32FC1);
	cv::Mat sampleMat(1, 2, CV_32FC1);

	if (imageOnFlag == 1)
	{
		// determine class of each image pixel
		for (int i = 0; i < (image.rows); ++i)
			for (int j = 0; j < (image.cols); ++j)
			{
				x = i/kx1;
				y = j/kx2;


				//predict sample class for image pixel <-> old code style
				CvMat m;
			    float a[] = { float(x/ScaleMean), float(y/ScaleStd) };
				cvInitMatHeader (&m, 1, 2, CV_32FC1, a);
				pcc = 700*carpet_Tree.predict(&m);


				//plot image pixel
				image.at<Vec3b>(j,i) = Vec3b( pcc, pcc, pcc);

				//plot image pixel
				image2.at<Vec3b>(j, i)  = blue;

			}
	}

	//determines the "area" which still belongs to a certain class
	double r = 0.03;

	//"current true class type"
	double ctct;

	for (int i=0; i<NumTestSamples; i++)
	{
		//////////////////
		///image stuff///
		////////////////

			//determine true sample class
			ctct = test_class_vec[i].dirtThreshold;

			if (imageOnFlag == 1)
			{
				x = ceil(kx1*test_feat_vec[i].mean);
				y = ceil(kx2*test_feat_vec[i].stdDev);

				//daw samples if their true class color to the image (as circles)
				circle( image, Point(x,y), 3, Scalar(floor(ctct*700.0), floor(ctct*700.0), floor(ctct*700.0)), -1 , 8);
			}

		///////////////////////////////////
		///percentage calculation stuff///
		/////////////////////////////////


			//predict sample class
			CvMat m;
		    float a[] = { float(test_feat_vec[i].mean/ScaleMean), float(test_feat_vec[i].stdDev/ScaleStd) };
			cvInitMatHeader (&m, 1, 2, CV_32FC1, a);
			float pcc = carpet_Tree.predict (&m);

			//show result
			std::cout << "True threshold=" << ctct << "\tPredicted threshold=" << pcc
					  << "\tmean=" << x << "\tstd=" << y << std::endl;

			//determine class index
			j = 0;
			while (ClassTypes_vec[j].dirtThreshold != ctct)
			{
				j++;
			}

			//increase the total number of test samples for this class
			num_vec[j].totalnum++;

			//check if test sample in class "area"
			if ((pcc > ctct-r) && (pcc < ctct+r))
//			if (((pcc == 1) && (ctct == 0.25)) || ((pcc == 2) && (ctct == 0.35)))
			{
				num_vec[j].correctnum++;
			}

	} //end-for


	///////////////////////////////////////////////
	///plot position and class of train samples///
	/////////////////////////////////////////////

	if (imageOnFlag == 1)
	{
		for (int i=0; i<NumTrainSamples; i++)
		{
			//determine true sample class
			ctct = train_class_vec[i].dirtThreshold;

			x = ceil(kx1*train_feat_vec[i].mean*ScaleMean);
			y = ceil(kx2*train_feat_vec[i].stdDev*ScaleStd);

			//daw samples if their true class color to the image (as circles)
			circle( image2, Point(x,y), 3, Scalar(floor(ctct*700.0), floor(ctct*700.0), floor(ctct*700.0)), -1 , 8);
		}
	}

	////////////////////////////////////////
	///plot percentage calculation stuff///
	//////////////////////////////////////

	double res = 0;

	for (int i=0; i<NumClass; i++)
	{
		res = 100*(double(num_vec[i].correctnum)/double(num_vec[i].totalnum));

		std::cout 	<< "Dirt threshold (class)=" << ClassTypes_vec[i].dirtThreshold
					<< "\tCorrect/total test-samples=" << num_vec[i].correctnum << "/" << num_vec[i].totalnum
					<< "\tPercentage=" << res <<std::endl;

	}


	///////////////////////
	///show image stuff///
	/////////////////////

	if (imageOnFlag == 1)
	{
		//show image on screen
		imshow("SVM evaluation image", image);
		imshow("Sample image", image2);
		waitKey(0);
	}
}

void DirtDetection::GBTreeEvaluation(	std::vector<CarpetFeatures>& train_feat_vec, std::vector<CarpetClass>& train_class_vec,
					std::vector<CarpetFeatures>& test_feat_vec, std::vector<CarpetClass>& test_class_vec,
					CvGBTrees &carpet_GBTree, double ScaleMean, double ScaleStd)
{

	//number of test samples
	int NumTestSamples = test_feat_vec.size();
	int NumTrainSamples = train_feat_vec.size();

	//saves the different class types
	std::vector<CarpetClass> ClassTypes_vec;

	//number of different class types
	int NumClass = 0;

	//max mean value
	double maxmean = -1;
	//max std. dev.
	double maxstd = -1;

	//loop variables
	bool foundflag = 0;
	int j = 0;

	bool imageOnFlag = 1;

	for(int i = 0; i< NumTrainSamples; i++)
	{
		//test if mean or stdDev bigger than saved values
		if (train_feat_vec[i].mean > maxmean ){maxmean = train_feat_vec[i].mean;}
		if (train_feat_vec[i].stdDev > maxstd ){maxstd = train_feat_vec[i].stdDev;}

		//Determine if new class type
		foundflag = 0;
		j = 0;
		while ((foundflag == 0) && (j<NumClass))
		{
			if (ClassTypes_vec[j].dirtThreshold==train_class_vec[i].dirtThreshold)
			{
				foundflag = 1;
			}
			j++;
		}

		//if new class type, then save it in the list
		if (foundflag == 0)
		{
			ClassTypes_vec.push_back(train_class_vec[i]);
			NumClass++;
		}

	}

	//saves the total number and the number of correct classified samples of each class
	std::vector<NumStruc> num_vec;

	//initialise num_vec with void values
	NumStruc num_struc;
	num_struc.totalnum = 0;
	num_struc.correctnum = 0;

	for(int i = 0; i<NumClass; i++)
	{
		num_vec.push_back(num_struc);
	}


	int x,y;

	//save the prediction of the SVM
	//"predicted current class"
	float pcc;

	//Image for visual representation
	int width = 700, height = 700;

	cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
	cv::Mat image2 = cv::Mat::zeros(height, width, CV_8UC3);

	double kx1 = width/(1.2*maxmean);
	double kx2 = height/(1.2*maxstd);

	Vec3b green(0,255,0), blue (255,0,0), red (0,0,255);

//	cv::Mat sampleMat = cv::Mat::zeros(1, 2, CV_32FC1);
	cv::Mat sampleMat(1, 2, CV_32FC1);

	if (imageOnFlag == 1)
	{
		// determine class of each image pixel
		for (int i = 0; i < (image.rows); ++i)
			for (int j = 0; j < (image.cols); ++j)
			{
				x = i/kx1;
				y = j/kx2;


				//predict sample class for image pixel <-> old code style
				CvMat m;
			    float a[] = { float(x/ScaleMean), float(y/ScaleStd) };
				cvInitMatHeader (&m, 1, 2, CV_32FC1, a);
				pcc = 700*carpet_GBTree.predict(&m);


				//plot image pixel
				image.at<Vec3b>(j,i) = Vec3b( pcc, pcc, pcc);

				//plot image pixel
				image2.at<Vec3b>(j, i)  = blue;

			}
	}

	//determines the "area" which still belongs to a certain class
	double r = 0.03;

	//"current true class type"
	double ctct;

	for (int i=0; i<NumTestSamples; i++)
	{
		//////////////////
		///image stuff///
		////////////////

			//determine true sample class
			ctct = test_class_vec[i].dirtThreshold;

			if (imageOnFlag == 1)
			{
				x = ceil(kx1*test_feat_vec[i].mean);
				y = ceil(kx2*test_feat_vec[i].stdDev);

				//daw samples if their true class color to the image (as circles)
				circle( image, Point(x,y), 3, Scalar(floor(ctct*700.0), floor(ctct*700.0), floor(ctct*700.0)), -1 , 8);
			}

		///////////////////////////////////
		///percentage calculation stuff///
		/////////////////////////////////


			//predict sample class
			CvMat m;
		    float a[] = { float(test_feat_vec[i].mean/ScaleMean), float(test_feat_vec[i].stdDev/ScaleStd) };
			cvInitMatHeader (&m, 1, 2, CV_32FC1, a);
			float pcc = carpet_GBTree.predict (&m);

			//show result
			std::cout << "True threshold=" << ctct << "\tPredicted threshold=" << pcc
					  << "\tmean=" << x << "\tstd=" << y << std::endl;

			//determine class index
			j = 0;
			while (ClassTypes_vec[j].dirtThreshold != ctct)
			{
				j++;
			}

			//increase the total number of test samples for this class
			num_vec[j].totalnum++;

			//check if test sample in class "area"
			if ((pcc > ctct-r) && (pcc < ctct+r))
//			if (((pcc == 1) && (ctct == 0.25)) || ((pcc == 2) && (ctct == 0.35)))
			{
				num_vec[j].correctnum++;
			}

	} //end-for


	///////////////////////////////////////////////
	///plot position and class of train samples///
	/////////////////////////////////////////////

	if (imageOnFlag == 1)
	{
		for (int i=0; i<NumTrainSamples; i++)
		{
			//determine true sample class
			ctct = train_class_vec[i].dirtThreshold;

			x = ceil(kx1*train_feat_vec[i].mean*ScaleMean);
			y = ceil(kx2*train_feat_vec[i].stdDev*ScaleStd);

			//daw samples if their true class color to the image (as circles)
			circle( image2, Point(x,y), 3, Scalar(floor(ctct*700.0), floor(ctct*700.0), floor(ctct*700.0)), -1 , 8);
		}
	}

	////////////////////////////////////////
	///plot percentage calculation stuff///
	//////////////////////////////////////

	double res = 0;

	for (int i=0; i<NumClass; i++)
	{
		res = 100*(double(num_vec[i].correctnum)/double(num_vec[i].totalnum));

		std::cout 	<< "Dirt threshold (class)=" << ClassTypes_vec[i].dirtThreshold
					<< "\tCorrect/total test-samples=" << num_vec[i].correctnum << "/" << num_vec[i].totalnum
					<< "\tPercentage=" << res <<std::endl;

	}


	///////////////////////
	///show image stuff///
	/////////////////////

	if (imageOnFlag == 1)
	{
		//show image on screen
		imshow("SVM evaluation image", image);
		imshow("Sample image", image2);
		waitKey(0);
	}
}

void DirtDetection::SVMExampleCode()
{

    // Data for visual representation
    int width = 512, height = 512;
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);


    const int NUMBER_OF_TRAINING_SAMPLES = 10;
    const int ATTRIBUTES_PER_SAMPLE = 2;

    // Set up training data
    float labels [NUMBER_OF_TRAINING_SAMPLES] = {-1.0, 3.0, 3.0, 3.0, 2.0, 2.0, 2.0, 2.0, -1.0, -1.0};

    float trainingData[NUMBER_OF_TRAINING_SAMPLES][ATTRIBUTES_PER_SAMPLE] = { {50, 10}, {150, 130}, {120, 190}, {170, 110}, {470, 410}, {450, 420}, {490, 430}, {480, 440}, {40, 30}, {20, 20} };


    // create data and label matrix
    cv::Mat training_data(NUMBER_OF_TRAINING_SAMPLES, ATTRIBUTES_PER_SAMPLE, CV_32FC1, trainingData);
    cv::Mat training_classifications(NUMBER_OF_TRAINING_SAMPLES, 1, CV_32FC1, labels);


    // Set up SVM's parameters
    CvSVMParams params;
    params.svm_type    = CvSVM::NU_SVR;
    params.kernel_type = CvSVM::RBF;

    params.degree = 3.0;
    params.gamma = 1.0;
    params.coef0 = 1.0;
    params.C = 1;
    params.nu = 0.8;
    params.p = 0.1;
    params.class_weights = NULL;


    params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);


    // Train the SVM
    CvSVM SVM;

    //training if parameters are known
//    SVM.train(training_data, training_classifications, Mat(), Mat(), params);


    //define training area:
    cv::ParamGrid gamma_grid( 0.00012207, 10, 2 );
    cv::ParamGrid C_grid( 0.000976562, 5, 2 );
    cv::ParamGrid nu_grid( 0.01546875, 0.99, 1.5 );
    cv::ParamGrid degree_grid(1.5, 5, 1.5);

    //determine optimal parameters and train SVM
    //Comment: It's important to define all input parameters because otherwise "SVM.train_auto" might not work properly!!!
    SVM.train_auto(	training_data, training_classifications, cv::Mat(), cv::Mat(), params, 10,
					C_grid,
					gamma_grid,
					CvSVM::get_default_grid(CvSVM::P),
					nu_grid, CvSVM::get_default_grid(CvSVM::COEF),
					CvSVM::get_default_grid(CvSVM::DEGREE)	);

    //determine ros package path
    std::string svmpath = ros::package::getPath("autopnp_dirt_detection") + "/common/files/svm/surface.svm";
//    std::cout << svmpath << std::endl;
    //save SVM parameters to file
    SVM.save(svmpath.c_str());
    params = SVM.get_params();
    //display SVM parameters on screen
    std::cout << "C=" << params.C << "\ncoeff0=" << params.coef0 << "\ndegree=" << params.degree << "\ngamma=" << params.gamma << "\nnu=" << params.nu << "\np=" << params.p << std::endl;


    Vec3b green(0,255,0), blue (255,0,0), red (0,0,255);

    // Show the decision regions given by the SVM
    for (int i = 0; i < image.rows; ++i)
        for (int j = 0; j < image.cols; ++j)
        {
            cv::Mat sampleMat = (Mat_<float>(1,2) << i,j);
            float response = SVM.predict(sampleMat);

//            if (response <= 1.5)
//            {
//                image.at<Vec3b>(j, i)  = green;
//            }
//            else
//			{
//				if (response >= 2.5)
//				{
//					image.at<Vec3b>(j, i)  = blue;
//				}
//				else
//				{
//					image.at<Vec3b>(j, i)  = red;
//				}
//
//			}

            image.at<Vec3b>(j, i) = Vec3b((response + 2)/6 * 255, (response + 2)/6 * 255, (response + 2)/6 * 255);

        }

    //show training data
    for (int i=0; i<training_data.rows; i++)
	{
    	float label = training_classifications.at<float>(i);
        if (label <= 1.5)
        {
        	circle( image, Point(training_data.at<float>(i, 0),  training_data.at<float>(i, 1)), 5, Scalar(  0,   255,   0), 3, 8);
        }
        else
		{
			if (label >= 2.5)
			{
				circle( image, Point(training_data.at<float>(i, 0),  training_data.at<float>(i, 1)), 5, Scalar(  255,   0,   0), 3, 8);
			}
			else
			{
				circle( image, Point(training_data.at<float>(i, 0),  training_data.at<float>(i, 1)), 5, Scalar(  0,   0,   255), 3, 8);
			}

		}
	}


    //show image on screen
    imshow("SVM Simple Example", image);
    waitKey(0);

}

void DirtDetection::SVMTestFunction()
{
    const int s = 1000;
    int size = 400;
    int i, j, sv_num;
    IplImage *img;
    CvSVM svm = CvSVM ();
    CvSVMParams param;
    CvTermCriteria criteria;
    CvRNG rng = cvRNG (time (NULL));
    CvPoint pts[s];
    float data[s * 2];
    int res[s];
    CvMat data_mat, res_mat;
    CvScalar rcolor;
    const float *support;

    img = cvCreateImage (cvSize (size, size), IPL_DEPTH_8U, 3);
    cvZero (img);

    for (i = 0; i < s; i++) {
      pts[i].x = cvRandInt (&rng) % size;
      pts[i].y = cvRandInt (&rng) % size;
      if (pts[i].y > 50 * cos (pts[i].x * CV_PI / 100) + 200) {
        cvLine (img, cvPoint (pts[i].x - 2, pts[i].y - 2), cvPoint (pts[i].x + 2, pts[i].y + 2), CV_RGB (255, 0, 0));
        cvLine (img, cvPoint (pts[i].x + 2, pts[i].y - 2), cvPoint (pts[i].x - 2, pts[i].y + 2), CV_RGB (255, 0, 0));
        res[i] = 1;
      }
      else {
        if (pts[i].x > 200) {
          cvLine (img, cvPoint (pts[i].x - 2, pts[i].y - 2), cvPoint (pts[i].x + 2, pts[i].y + 2), CV_RGB (0, 255, 0));
          cvLine (img, cvPoint (pts[i].x + 2, pts[i].y - 2), cvPoint (pts[i].x - 2, pts[i].y + 2), CV_RGB (0, 255, 0));
          res[i] = 2;
        }
        else {
          cvLine (img, cvPoint (pts[i].x - 2, pts[i].y - 2), cvPoint (pts[i].x + 2, pts[i].y + 2), CV_RGB (0, 0, 255));
          cvLine (img, cvPoint (pts[i].x + 2, pts[i].y - 2), cvPoint (pts[i].x - 2, pts[i].y + 2), CV_RGB (0, 0, 255));
          res[i] = 3;
        }
      }
    }

    cvNamedWindow ("SVM", CV_WINDOW_AUTOSIZE);
    cvShowImage ("SVM", img);
    cvWaitKey (0);

    for (i = 0; i < s; i++) {
      data[i * 2] = float (pts[i].x) / size;
      data[i * 2 + 1] = float (pts[i].y) / size;
    }
    cvInitMatHeader (&data_mat, s, 2, CV_32FC1, data);
    cvInitMatHeader (&res_mat, s, 1, CV_32SC1, res);
    criteria = cvTermCriteria (CV_TERMCRIT_EPS, 1000, FLT_EPSILON);
    param = CvSVMParams (CvSVM::C_SVC, CvSVM::RBF, 10.0, 8.0, 1.0, 10.0, 0.5, 0.1, NULL, criteria);

    svm.train (&data_mat, &res_mat, NULL, NULL, param);

    for (i = 0; i < size; i++) {
      for (j = 0; j < size; j++) {
        CvMat m;
        float ret = 0.0;
        float a[] = { float (j) / size, float (i) / size };
        cvInitMatHeader (&m, 1, 2, CV_32FC1, a);
        ret = svm.predict (&m);
        switch ((int) ret) {
        case 1:
          rcolor = CV_RGB (100, 0, 0);
          break;
        case 2:
          rcolor = CV_RGB (0, 100, 0);
          break;
        case 3:
          rcolor = CV_RGB (0, 0, 100);
          break;
        }
        cvSet2D (img, i, j, rcolor);
      }
    }

    for (i = 0; i < s; i++) {
      CvScalar rcolor;
      switch (res[i]) {
      case 1:
        rcolor = CV_RGB (255, 0, 0);
        break;
      case 2:
        rcolor = CV_RGB (0, 255, 0);
        break;
      case 3:
        rcolor = CV_RGB (0, 0, 255);
        break;
      }
      cvLine (img, cvPoint (pts[i].x - 2, pts[i].y - 2), cvPoint (pts[i].x + 2, pts[i].y + 2), rcolor);
      cvLine (img, cvPoint (pts[i].x + 2, pts[i].y - 2), cvPoint (pts[i].x - 2, pts[i].y + 2), rcolor);
    }

    sv_num = svm.get_support_vector_count ();
    for (i = 0; i < sv_num; i++) {
      support = svm.get_support_vector (i);
      cvCircle (img, cvPoint ((int) (support[0] * size), (int) (support[1] * size)), 5, CV_RGB (200, 200, 200));
    }

    cvNamedWindow ("SVM", CV_WINDOW_AUTOSIZE);
    cvShowImage ("SVM", img);
    cvWaitKey (0);

    cvDestroyWindow ("SVM");
    cvReleaseImage (&img);
}




