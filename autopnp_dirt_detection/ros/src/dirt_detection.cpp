#include <autopnp_dirt_detection/dirt_detection.h>

using namespace ipa_ImageDisplay;

ImageDisplay::ImageDisplay(ros::NodeHandle node_handle)
: node_handle_(node_handle)
{
	it_ = 0;
}


ImageDisplay::~ImageDisplay()
{
	if (it_ != 0) delete it_;
}


void ImageDisplay::init()
{
	it_ = new image_transport::ImageTransport(node_handle_);
	color_camera_image_sub_ = it_->subscribe("camera/rgb/image_color", 1, boost::bind(&ImageDisplay::imageDisplayCallback, this, _1));
}


unsigned long ImageDisplay::convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& color_image_msg, cv_bridge::CvImageConstPtr& color_image_ptr, cv::Mat& color_image)
{
	try
	{
		color_image_ptr = cv_bridge::toCvShare(color_image_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("PeopleDetection: cv_bridge exception: %s", e.what());
		return 1;
	}
	color_image = color_image_ptr->image;

	return 0;
}

void ImageDisplay::imageDisplayCallback_old_cv_code(const sensor_msgs::ImageConstPtr& color_image_msg)
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

void ImageDisplay::imageDisplayCallback_new_cv_code(const sensor_msgs::ImageConstPtr& color_image_msg)
{
	cv_bridge::CvImageConstPtr color_image_ptr;
	cv::Mat color_image;
	convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);

	// color_image is now available with the current image from your camera

	cv::imshow("original image", color_image);
	//printf("Columns %d, Rows %d\n", color_image.cols,color_image.rows);

	//int thresh = 350;
	int scale = 6;

	//convert color image to one channel gray-scale image
	//color_image -> gray_image
	cv::Mat gray_image;
	cv::cvtColor(color_image, gray_image, CV_BGR2GRAY);

	//given a one channel image
	unsigned int size = (int)floor((float)pow(2.0,scale)); //the size to do the saliency at


	//create different images
	cv::Mat bw_im;
	cv::resize(gray_image,bw_im,cv::Size(size,size));

	cv::Mat realInput(size,size, CV_32FC1);
	cv::Mat imaginaryInput(size,size, CV_32FC1);
	cv::Mat complexInput(size,size, CV_32FC2);


	bw_im.convertTo(realInput,CV_32F,1.0/255,0);
	imaginaryInput = cv::Mat::zeros(size,size,CV_32F);

	std::vector<cv::Mat> vec;
	vec.push_back(realInput);
	vec.push_back(imaginaryInput);
	cv::merge(vec, complexInput);

	cv::Mat dft_A(size,size,CV_32FC2);

	cv::dft(complexInput, dft_A, cv::DFT_COMPLEX_OUTPUT,size);
	vec.clear();
	cv::split(dft_A,vec);
	realInput = vec[0];
	imaginaryInput = vec[1];

	// Compute the phase angle
	cv::Mat image_Mag(size,size, CV_32FC1);
	cv::Mat image_Phase(size,size, CV_32FC1);

	//compute the phase of the spectrum
	cv::cartToPolar(realInput, imaginaryInput, image_Mag, image_Phase,0);

	cv::Mat log_mag(size,size, CV_32FC1);
	cv::log(image_Mag, log_mag);

	//Box filter the magnitude, then take the difference
	cv::Mat log_mag_Filt(size,size, CV_32FC1);

	cv::Mat filt = cv::Mat::ones(3, 3, CV_32FC1) * 1./9.;
	//filt.convertTo(filt,-1,1.0/9.0,0);

	cv::filter2D(log_mag, log_mag_Filt, -1, filt);

	//cv::subtract(log_mag, log_mag_Filt, log_mag);
	log_mag -= log_mag_Filt;
	cv::exp(log_mag, image_Mag);

	cv::polarToCart(image_Mag, image_Phase, realInput, imaginaryInput,0);

	vec.clear();
	vec.push_back(realInput);
	vec.push_back(imaginaryInput);
	cv::merge(vec, dft_A);

	cv::dft(dft_A, dft_A, cv::DFT_INVERSE,size);

	dft_A = abs(dft_A);
	dft_A.mul(dft_A);

	cv::split(dft_A, vec);

	realInput = vec[0];



	double minv, maxv;
	cv::Point2i minl, maxl;

	cv::Size2i ksize;
	ksize.width = 3;
	ksize.height = 3;
	cv::GaussianBlur(realInput,realInput, ksize,0);
	cv::GaussianBlur(realInput,realInput, ksize,0);

	cv::minMaxLoc(realInput,&minv,&maxv,&minl,&maxl);

	realInput.convertTo(realInput,-1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));


	cv::Mat resultImage;
	cv::resize(realInput,resultImage,color_image.size());

	// remove saliency at the image border
	int borderX = resultImage.cols/20;
	int borderY = resultImage.rows/20;
	if (borderX > 0 && borderY > 0)
	{
	cv::Mat smallImage_ = resultImage.colRange(borderX, resultImage.cols-1-borderX);
	cv::Mat smallImage = smallImage_.rowRange(borderY, smallImage_.rows-1-borderY);

	copyMakeBorder(smallImage, resultImage, borderY, borderY, borderX, borderX, cv::BORDER_CONSTANT, cv::Scalar(0));
	}


	cv::imshow("image2", resultImage);
	cv::waitKey(10);
}

void ImageDisplay::imageDisplayCallback_channel_combination(const sensor_msgs::ImageConstPtr& color_image_msg)
{
	//ImageDisplay::imageDisplayCallback_new_cv_code(color_image_msg);

	cv_bridge::CvImageConstPtr color_image_ptr;
	cv::Mat color_image;
	convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);

	// color_image is now available with the current image from your camera
	cv::imshow("original imageB", color_image);

	cv::Mat fci; // "fci"<-> first channel image
	cv::Mat sci; // "sci"<-> second channel image
	cv::Mat tci; //"tci"<-> second channel image

	std::vector<cv::Mat> vec;
	vec.push_back(fci);
	vec.push_back(sci);
	vec.push_back(tci);

	cv::split(color_image,vec);

	fci = vec[0];
	sci = vec[1];
	tci = vec[2];

	cv::Mat res_fci; // "fci"<-> first channel image
	cv::Mat res_sci; // "sci"<-> second channel image
	cv::Mat res_tci; //"tci"<-> second channel image

	ImageDisplay::oneChannelTrafo(fci, res_fci);
	ImageDisplay::oneChannelTrafo(sci, res_sci);
	ImageDisplay::oneChannelTrafo(tci, res_tci);

	cv::Mat realInput;

	realInput = (res_fci + res_sci + res_tci)/3;



	double minv, maxv;
	cv::Point2i minl, maxl;

	cv::Size2i ksize;
	ksize.width = 3;
	ksize.height = 3;
	cv::GaussianBlur(realInput,realInput, ksize,0);
	cv::GaussianBlur(realInput,realInput, ksize,0);

	cv::minMaxLoc(realInput,&minv,&maxv,&minl,&maxl);

	realInput.convertTo(realInput,-1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));


	cv::Mat resultImage;
	cv::resize(realInput,resultImage,color_image.size());

	// remove saliency at the image border
	int borderX = resultImage.cols/20;
	int borderY = resultImage.rows/20;
	if (borderX > 0 && borderY > 0)
	{
	cv::Mat smallImage_ = resultImage.colRange(borderX, resultImage.cols-1-borderX);
	cv::Mat smallImage = smallImage_.rowRange(borderY, smallImage_.rows-1-borderY);

	copyMakeBorder(smallImage, resultImage, borderY, borderY, borderX, borderX, cv::BORDER_CONSTANT, cv::Scalar(0));
	}


	cv::imshow("image2B", resultImage);
	cv::waitKey(10);
}

void ImageDisplay::oneChannelTrafo(cv::Mat& one_channel_image, cv::Mat& result_image)
{
	int scale = 6;

	//given a one channel image
	unsigned int size = (int)floor((float)pow(2.0,scale)); //the size to do the saliency at

	//create different images
	cv::Mat bw_im;
	cv::resize(one_channel_image,bw_im,cv::Size(size,size));

	cv::Mat realInput(size,size, CV_32FC1);
	cv::Mat imaginaryInput(size,size, CV_32FC1);
	cv::Mat complexInput(size,size, CV_32FC2);

	bw_im.convertTo(realInput,CV_32F,1.0/255,0);
	imaginaryInput = cv::Mat::zeros(size,size,CV_32F);

	std::vector<cv::Mat> vec;
	vec.push_back(realInput);
	vec.push_back(imaginaryInput);
	cv::merge(vec, complexInput);

	cv::Mat dft_A(size,size,CV_32FC2);

	cv::dft(complexInput, dft_A, cv::DFT_COMPLEX_OUTPUT,size);
	vec.clear();
	cv::split(dft_A,vec);
	realInput = vec[0];
	imaginaryInput = vec[1];

	// Compute the phase angle
	cv::Mat image_Mag(size,size, CV_32FC1);
	cv::Mat image_Phase(size,size, CV_32FC1);

	//compute the phase of the spectrum
	cv::cartToPolar(realInput, imaginaryInput, image_Mag, image_Phase,0);

	cv::Mat log_mag(size,size, CV_32FC1);
	cv::log(image_Mag, log_mag);

	//Box filter the magnitude, then take the difference
	cv::Mat log_mag_Filt(size,size, CV_32FC1);

	cv::Mat filt = cv::Mat::ones(3, 3, CV_32FC1) * 1./9.;
	//filt.convertTo(filt,-1,1.0/9.0,0);

	cv::filter2D(log_mag, log_mag_Filt, -1, filt);

	//cv::subtract(log_mag, log_mag_Filt, log_mag);
	log_mag -= log_mag_Filt;
	cv::exp(log_mag, image_Mag);

	cv::polarToCart(image_Mag, image_Phase, realInput, imaginaryInput,0);

	vec.clear();
	vec.push_back(realInput);
	vec.push_back(imaginaryInput);
	cv::merge(vec, dft_A);

	cv::dft(dft_A, dft_A, cv::DFT_INVERSE,size);

	dft_A = abs(dft_A);
	dft_A.mul(dft_A);

	cv::split(dft_A, vec);

	result_image = vec[0];
}

void ImageDisplay::imageDisplayCallback(const sensor_msgs::ImageConstPtr& color_image_msg)
{
	//ImageDisplay::imageDisplayCallback_new_cv_code(color_image_msg);

	cv_bridge::CvImageConstPtr color_image_ptr;
	cv::Mat color_image;
	convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);

	// color_image is now available with the current image from your camera
	cv::imshow("original imageC", color_image);

	std::vector<double> mean;
	std::vector<double> stddev;

	color_image.convertTo(color_image, CV_32FC3);

	cv::meanStdDev(color_image, mean, stddev);

//	printf("Mean1 %f, stddev1 %f\n", mean[0], stddev[0]); //Teppich-Kanal 1 ca. mean=130
//	printf("Mean2 %f, stddev2 %f\n", mean[1], stddev[1]);
//	printf("Mean3 %f, stddev3 %f\n", mean[2], stddev[2]);

	cv::Mat fci; // "fci"<-> first channel image
	cv::Mat sci; // "sci"<-> second channel image
	cv::Mat tci; //"tci"<-> second channel image

	std::vector<cv::Mat> vec;
	vec.push_back(fci);
	vec.push_back(sci);
	vec.push_back(tci);

	cv::split(color_image,vec);

	double minv, maxv;
	cv::Point2i minl, maxl;

	fci = vec[0];

	cv::Mat fci2;
	cv::minMaxLoc(fci,&minv,&maxv,&minl,&maxl);
	fci.convertTo(fci2,-1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));
	cv::imshow("fci", fci2);

	sci = vec[1];
	tci = vec[2];

	cv::Mat res_fci; // "fci"<-> first channel image
	cv::Mat res_sci; // "sci"<-> second channel image
	cv::Mat res_tci; //"tci"<-> second channel image


	fci -=  140;//mean[0];
	cv::multiply(fci,fci,fci);
	cv::minMaxLoc(fci,&minv,&maxv,&minl,&maxl);
	fci.convertTo(fci,-1, 1/(maxv-minv), 1*(minv)/(maxv-minv));
	//cv::threshold(fci, fci, 0.7, 1, cv::THRESH_BINARY);

	cv::imshow("fci2", fci);

	sci -= 140;//mean[1];
	cv::multiply(sci,sci,sci);
	cv::minMaxLoc(sci,&minv,&maxv,&minl,&maxl);
	sci.convertTo(sci,-1, 1/(maxv-minv), 1*(minv)/(maxv-minv));
	//cv::threshold(sci, sci, 0.7, 1, cv::THRESH_BINARY);


	tci -=  140;//mean[2];
	cv::multiply(tci,tci,tci);
	cv::minMaxLoc(tci,&minv,&maxv,&minl,&maxl);
	tci.convertTo(tci,-1, 1/(maxv-minv), 1*(minv)/(maxv-minv));
	//cv::threshold(tci, sci, 0.7, 1, cv::THRESH_BINARY);

	cv::Mat realInput = (fci+sci+tci)/3;
	cv::threshold(realInput, realInput, 0.7, 1, cv::THRESH_BINARY);

//	cv::minMaxLoc(realInput,&minv,&maxv,&minl,&maxl);
//
//	realInput.convertTo(realInput,-1, 255/(maxv-minv), 255*(minv)/(maxv-minv));


	cv::Mat resultImage;
	cv::resize(realInput,resultImage,color_image.size());

	// remove saliency at the image border
	int borderX = resultImage.cols/20;
	int borderY = resultImage.rows/20;
	if (borderX > 0 && borderY > 0)
	{
	cv::Mat smallImage_ = resultImage.colRange(borderX, resultImage.cols-1-borderX);
	cv::Mat smallImage = smallImage_.rowRange(borderY, smallImage_.rows-1-borderY);

	copyMakeBorder(smallImage, resultImage, borderY, borderY, borderX, borderX, cv::BORDER_CONSTANT, cv::Scalar(0));
	}


	cv::imshow("image2C", resultImage);
	cv::waitKey(10);





}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_display");

	ros::NodeHandle n;

	ImageDisplay id(n);
	id.init();


	//start to look for messages (loop)
	ros::spin();

	return 0;
}
