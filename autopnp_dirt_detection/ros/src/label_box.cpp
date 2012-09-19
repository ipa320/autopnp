#include "autopnp_dirt_detection/label_box.h"

int labelImage::input_text(cv::Mat img, std::string winName, std::string * text, cv::Rect rect)
{
	std::string completeText;
	int completeTextWidth = 0;
	cv::waitKey(10);
	int space = 0;

	for (;;)
	{
		int c = cv::waitKey(0);
		if (c == ESC || c == ESC2)
			return 0;
		else if (c == RETURN || c == RETURN2)
			break;
		else if (c == BACKSPACE || c == BACKSPACE2)
		{
			if (completeText.length() > 0)
			{
				std::string sub = completeText.substr(completeText.length() - 1, 1);
				cv::Size textSize = cv::getTextSize(sub, cv::FONT_HERSHEY_SIMPLEX, 0.75, 1, 0);
				cv::rectangle(img, cv::Rect(rect.x + completeTextWidth - textSize.width + 1, rect.y + 1, 200 + textSize.width - completeTextWidth - 2, 30 - 2), cv::Scalar(255, 255, 255), -1);
				completeText = completeText.substr(0, completeText.length() - 1);
				cv::imshow(winName, img);
				space -= textSize.width;
				completeTextWidth -= textSize.width;
			}
		}
		else if (c != LSHIFT && c != RSHIFT && c != LSHIFT2 && c != RSHIFT2)
		{
			std::stringstream ss;
			std::string s;
			if (c == SPACE || c == SPACE2)
				s = " ";
			else
			{
				ss << (char)c;
				ss >> s;
			}
			cv::putText(img, s, cv::Point(rect.x + 1 + space, rect.y + 25), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 1, 8, false);
			cv::imshow(winName, img);
			cv::Size textSize = cv::getTextSize(s, cv::FONT_HERSHEY_SIMPLEX, 0.75, 1, 0);
			space += textSize.width;
			completeText += s;
			completeTextWidth += textSize.width;
		}
	}
	std::cout << "Text: " << completeText << std::endl;
	*text = completeText.data();
	return 1;
}

void labelImage::drawRect(cv::Mat & img, cv::RotatedRect rrect, cv::Scalar clr)
{
	cv::Point2f vertices[4];
	rrect.points(vertices);
	for (int i = 0; i < 4; i++)
		cv::line(img, vertices[i], vertices[(i + 1) % 4], clr);
	cv::rectangle(img, rrect.center, rrect.center, cv::Scalar(255, 255, 255), 2, 1, 0);
}

void labelImage::showInfo(std::string winName, cv::Mat img, cv::RotatedRect r, std::string text, cv::Scalar clr)
{
	cv::Mat infoBox = img.clone();

	float offsetX = r.size.width < 50 ? 50 : r.size.width;
	float offsetY = r.size.height < 50 ? 50 : r.size.height;

	if (r.center.x - offsetX < 0)
		offsetX *= -1;

	if (r.center.y - offsetY < 0)
		offsetY *= -1;

	cv::Rect textRect(r.center.x - offsetX, r.center.y - offsetY, 200, 30);

	cv::rectangle(infoBox, textRect, cv::Scalar(255, 255, 255), -1);
	cv::rectangle(infoBox, textRect, clr);

	labelImage::drawRect(infoBox, r, clr);

	cv::putText(infoBox, text, cv::Point(textRect.x + 1, textRect.y + 25), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 1, 8, false);
	cv::imshow(winName, infoBox);
	cv::waitKey(0);
}

bool labelImage::checkPointInsideRect(cv::RotatedRect rect, cv::Point2f p)
{
	float angle = (rect.angle / 180) * 3.1415926535;
	cv::Point2f dp(p.x - rect.center.x, p.y - rect.center.y);

	// distance point-centre
	float h = std::sqrt(dp.x * dp.x + dp.y * dp.y);
	float currentA = std::atan2(dp.y, dp.x);

	// rotate point
	float newA = currentA - angle;

	// new Point position
	cv::Point2f p2(std::cos(newA) * h, std::sin(newA) * h);

	// actual check
	if (p2.x > -0.5 * rect.size.width && p2.x < 0.5 * rect.size.width && p2.y > -0.5 * rect.size.height && p2.y < 0.5 * rect.size.height)
		return true;
	return false;
}


void labelImage::writeTxt(std::vector<labelImage> all, std::string path, std::string actTime)
{
	std::ifstream inn;
	std::string str;
	std::string word;

	std::cout << "path: " << path << std::endl;

	std::string newFileName = path + "/";
	newFileName.append(actTime);
	newFileName.append("_train.xml");

	std::ofstream newFile;
	newFile.open(newFileName.c_str());
	newFile << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
	newFile << std::endl;
	newFile << "<tagset>";
	newFile << std::endl;
	for (unsigned int j = 0; j < all.size(); j++)
	{
		std::vector<std::string> words = all[j].allTexts;
		std::vector<cv::RotatedRect> rects = all[j].allRects;
		std::string imgName = all[j].name;
		//    imgName = imgName.substr(imgName.find_last_of(' ') + 1, imgName.size() - imgName.find_last_of(' '));
		imgName = imgName.substr(imgName.find_last_of('/') + 1, imgName.size() - imgName.find_last_of('/'));
		newFile << "<image>" << std::endl << " <imageName>" << imgName << "</imageName>" << std::endl << "<taggedRectangles>" << std::endl;
		for (unsigned int i = 0; i < rects.size(); i++)
		{
			newFile << "<taggedRectangle center_x=\"" << rects[i].center.x << "\" center_y=\"" << rects[i].center.y
				<< "\" width=\"" << rects[i].size.width << "\" height=\"" << rects[i].size.height << "\" angle=\""
				<< rects[i].angle << "\" text=\"" << words[i] << "\" />" << std::endl;
		}
		newFile << "</taggedRectangles>" << std::endl << "</image>" << std::endl;
	}
	newFile << "</tagset>";
}


void labelImage::writeTxt3d(const std::vector<labelImage>& all, std::string path, std::string actTime)
{
	std::ifstream inn;
	std::string str;
	std::string word;

	std::cout << "path: " << path << std::endl;

	std::string newFileName = path + "/";
	newFileName.append(actTime);
	newFileName.append("_train.xml");

	std::ofstream newFile;
	newFile.open(newFileName.c_str());
	newFile << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
	newFile << std::endl;
	newFile << "<tagset>";
	newFile << std::endl;
	for (unsigned int j = 0; j < all.size(); j++)
	{
		std::vector<std::string> words = all[j].allTexts;
		std::vector<RegionPointTriple> rects = all[j].allRects3d;
		std::string imgName = all[j].name;
		//    imgName = imgName.substr(imgName.find_last_of(' ') + 1, imgName.size() - imgName.find_last_of(' '));
		imgName = imgName.substr(imgName.find_last_of('/') + 1, imgName.size() - imgName.find_last_of('/'));
		newFile << "<image>" << std::endl << " <imageName>" << imgName << "</imageName>" << std::endl << "<taggedRectangles>" << std::endl;
		for (unsigned int i = 0; i < rects.size(); i++)
		{
			newFile << "<taggedRectangle center_x=\"" << rects[i].center.x << "\" center_y=\"" << rects[i].center.y << "\" center_z=\"" << rects[i].center.z
				<< "\" width_x=\"" << rects[i].p1.x << "\" width_y=\"" << rects[i].p1.y << "\" width_z=\"" << rects[i].p1.z
				<< "\" height_x=\"" << rects[i].p2.x << "\" height_y=\"" << rects[i].p2.y << "\" height_z=\"" << rects[i].p2.z << "\" text=\"" << words[i] << "\" />" << std::endl;
		}
		newFile << "</taggedRectangles>" << std::endl << "</image>" << std::endl;
	}
	newFile << "</tagset>";
}


void labelImage::readTxt3d(std::vector<labelImage>& groundTruthData, std::string filename)
{
	std::ifstream imgxml;
	imgxml.open(filename.c_str());
	if (imgxml.is_open())
	{
		while (!imgxml.eof())
		{
			std::string output;
			imgxml >> output;

			// label[0] = "<imageName>"  => new image is described in xmlfile, copy filename
			if (output.find("<imageName>") != std::string::npos)
			{
				// new scene
				labelImage img("groundtruth", cv::Mat(), cv::Mat(), cv::Scalar(0,0,0));
//				img.name = output.substr(output.find(label[0]) + label[0].length(), output.length() - 2 * label[0].length() - 1); label[0]="<imageName>";
				groundTruthData.push_back(img);
			}
			else if (output.find("<taggedRectangles>") != std::string::npos || output.find("</taggedRectangles>") != std::string::npos)
			{
			}
			else if (output.find("taggedRectangle") != std::string::npos)
			{
				// new detection
				labelImage::RegionPointTriple rpt;
				groundTruthData[groundTruthData.size()-1].allRects3d.push_back(rpt);
				std::string s;
				groundTruthData[groundTruthData.size()-1].allTexts.push_back(s);
			}
			else if (groundTruthData.size()>0 && groundTruthData[groundTruthData.size()-1].allRects3d.size()>0)
			{
				RegionPointTriple& rpt = groundTruthData[groundTruthData.size()-1].allRects3d[groundTruthData[groundTruthData.size()-1].allRects3d.size()-1];

				cutout("center_x=", output, rpt.center.x);
				cutout("center_y=", output, rpt.center.y);
				cutout("center_z=", output, rpt.center.z);
				cutout("width_x=", output, rpt.p1.x);
				cutout("width_y=", output, rpt.p1.y);
				cutout("width_z=", output, rpt.p1.z);
				cutout("height_x=", output, rpt.p2.x);
				cutout("height_y=", output, rpt.p2.y);
				cutout("height_z=", output, rpt.p2.z);


				std::string textTag = "text=";	//text= in xmlfile
				if (output.find(textTag) != std::string::npos)
				{
					std::string text;
					if ((output[output.length() - 1] != '"'))
					{
						text.append(output.substr(output.find(textTag) + textTag.length() + 1, output.length() - textTag.length() - 1));
						text.append(" ");
						imgxml >> output;
						while ((output[output.length() - 1] != '"'))
						{
							text.append(output);
							text.append(" ");
							imgxml >> output;
						}
						text.append(output.substr(0, output.length() - 1));
					}
					else
						text.append(output.substr(output.find(textTag) + textTag.length() + 1, output.length() - textTag.length() - 2));

					groundTruthData[groundTruthData.size()-1].allTexts[groundTruthData[groundTruthData.size()-1].allTexts.size()-1] = text;
				}
			}
		}
		imgxml.close();
	}
}


bool labelImage::cutout(std::string label, std::string output, float& value)
{
	//cutout x,y,width,height
	if (output.find(label) != std::string::npos)
	{
		std::stringstream ss;
		ss << output.substr(output.find(label) + label.length() + 1, output.length() - 2 - label.length());
		ss >> value;
		return true;
	}
	else
		return false;
}


void onMouse(int event, int x, int y, int flags, void* param)
{
	static bool onRect = false;
	static int firstX = 0, firstY = 0;
	static int action = 0;

	cv::Mat image = ((labelImage *)param)->img.clone();
	std::string winName = ((labelImage *)param)->name;
	std::vector<cv::RotatedRect> * allRects = &((labelImage *)param)->allRects;

	// if no text is written at the moment
	if (((labelImage *)param)->textMode == false)
	{
		// Left mouse button clicked
		if (event == cv::EVENT_LBUTTONDOWN)
		{
			((labelImage *)param)->actualRotatedRect.angle = 0;
			((labelImage *)param)->rotationMode = false;
			onRect = false;
			int whichRect = -1;

			// check if other rect was clicked
			for (unsigned int i = 0; i < allRects->size(); i++)
			{
				cv::Point2f actualP(x, y);
				if (labelImage::checkPointInsideRect(allRects->at(i), actualP))
				{
					onRect = true;
					whichRect = i;
				}
			}

			// no other rect was clicked -> start new box, remember x,y
			if (!onRect)
			{
				action = 1;
				firstX = x;
				firstY = y;
			}
			// other rect was clicked -> show what text was written in there
			else
			{
				labelImage::showInfo(winName, image, (((labelImage *)param)->allRects)[whichRect], (((labelImage *)param)->allTexts)[whichRect], ((labelImage*)param)->allClrs[whichRect]);
			}
		}
		if (event == cv::EVENT_LBUTTONUP || event == cv::EVENT_MOUSEMOVE)
		{
			float smallerX = firstX > x ? x : firstX;
			float biggerX = firstX < x ? x : firstX;
			float smallerY = firstY > y ? y : firstY;
			float biggerY = firstY < y ? y : firstY;
			cv::Point2f mpoint(smallerX + 0.5 * (biggerX - smallerX), smallerY + 0.5 * (biggerY - smallerY));
			cv::RotatedRect rotatedRect(mpoint, cv::Size2f(biggerX - smallerX, biggerY - smallerY), 0);

			if (event == cv::EVENT_LBUTTONUP)
			{
				if (!onRect)
				{
					action = 0;
					labelImage::drawRect(image, rotatedRect, ((labelImage*)param)->actualClr);
					((labelImage *)param)->actualRotatedRect = rotatedRect;
					cv::imshow(winName, image);
				}
			}
			else
			{
				if (action == 1)
				{
					labelImage::drawRect(image, rotatedRect, ((labelImage*)param)->actualClr);
					((labelImage *)param)->actualRotatedRect = rotatedRect;
					cv::imshow(winName, image);
				}
			}
		}
		if (event == cv::EVENT_RBUTTONDOWN) //DELETE BOX
		{
			cv::Point2f actualP(x, y);
			for (unsigned int i = 0; i < allRects->size(); i++)
			{
				if (labelImage::checkPointInsideRect(allRects->at(i), actualP))
				{
					((labelImage *)param)->img = ((labelImage *)param)->originalImage.clone();
					((labelImage *)param)->allRects.erase(((labelImage *)param)->allRects.begin() + i);
					((labelImage *)param)->allTexts.erase(((labelImage *)param)->allTexts.begin() + i);
					((labelImage *)param)->allClrs.erase(((labelImage *)param)->allClrs.begin() + i);
					for (unsigned int j = 0; j < ((labelImage *)param)->allRects.size(); j++)
					{
						labelImage::drawRect(((labelImage*)param)->img, ((labelImage *)param)->allRects[j], ((labelImage*)param)->allClrs[j]);
					}
				}
				cv::imshow(winName, ((labelImage *)param)->img);
				//break;
			}
		}
	}
}


void labelImage::labelingLoop()
{
	// Mouse CB function
	cv::setMouseCallback(this->name, onMouse, (void*)this);
	//cv::setMouseCallback(this->name, boost::bind(&labelImage::onMouse, this, _1, _2, _3, _4, _5), (void*)this);

	bool active = true;

	// loop till next image
	while (active)
	{
		cv::Mat temporaryImg;
		static bool shift = true;

		int c = cv::waitKey(0);
		//   std::cout << "Key pressed: " << c << std::endl;

		temporaryImg = this->img.clone();
		//this->temporaryImage = this->img.clone();

		// Keyboard Input
		switch (c)
		{
			// Direction Keys for moving or resizing the box
			case LEFT:
			case LEFT2:
			{
				if (shift)
				{
					this->actualRotatedRect.center.x--;
				}
				else
				{
					float dx = std::cos((this->actualRotatedRect.angle / 180.0) * 3.14159265359);
					float dy = std::sin((this->actualRotatedRect.angle / 180.0) * 3.14159265359);

					this->actualRotatedRect.size.width--;
					this->actualRotatedRect.center.x -= 0.5 * dx;
					this->actualRotatedRect.center.y -= 0.5 * dy;
				}

				break;
			}

			case UP:
			case UP2:
			{
				if (shift)
				{
					this->actualRotatedRect.center.y--;
				}
				else
				{
					float dx = std::sin((this->actualRotatedRect.angle / 180.0) * 3.14159265359);
					float dy = std::cos((this->actualRotatedRect.angle / 180.0) * 3.14159265359);

					this->actualRotatedRect.size.height--;
					this->actualRotatedRect.center.x += 0.5 * dx;
					this->actualRotatedRect.center.y -= 0.5 * dy;
				}

				break;
			}

			case RIGHT:
			case RIGHT2:
			{
				if (shift)
				{
					this->actualRotatedRect.center.x++;
				}
				else
				{
					float dx = std::cos((this->actualRotatedRect.angle / 180.0) * 3.14159265359);
					float dy = std::sin((this->actualRotatedRect.angle / 180.0) * 3.14159265359);

					this->actualRotatedRect.size.width++;
					this->actualRotatedRect.center.x += 0.5 * dx;
					this->actualRotatedRect.center.y += 0.5 * dy;
				}

				break;
			}

			case DOWN:
			case DOWN2:
			{
				if (shift)
				{
					this->actualRotatedRect.center.y++;
				}
				else
				{
					float dx = std::sin((this->actualRotatedRect.angle / 180.0) * 3.14159265359);
					float dy = std::cos((this->actualRotatedRect.angle / 180.0) * 3.14159265359);

					this->actualRotatedRect.size.height++;
					this->actualRotatedRect.center.x -= 0.5 * dx;
					this->actualRotatedRect.center.y += 0.5 * dy;
				}

				break;
			}

			// Enter Text Mode
			case RETURN:
			case RETURN2:
			{
				cv::Mat textBox = this->img.clone();

				float offsetX = this->actualRotatedRect.size.width < 50 ? 50 : this->actualRotatedRect.size.width;
				float offsetY = this->actualRotatedRect.size.height < 50 ? 50 : this->actualRotatedRect.size.height;

				if (this->actualRotatedRect.center.x - offsetX < 0)
					offsetX *= -1;

				if (this->actualRotatedRect.center.y - offsetY < 0)
					offsetY *= -1;

				cv::Rect textRect(this->actualRotatedRect.center.x - offsetX, this->actualRotatedRect.center.y - offsetY, 200, 30);

				cv::rectangle(textBox, textRect, cv::Scalar(255, 255, 255), -1);
				cv::rectangle(textBox, textRect, this->actualClr);

				labelImage::drawRect(textBox, this->actualRotatedRect, this->actualClr);
				cv::imshow(this->name, textBox);

				this->textMode = true;
				std::string text;

				if (input_text(textBox, this->name, &text, textRect)) // text was written
				{
					labelImage::drawRect(this->img, this->actualRotatedRect, this->actualClr);
					this->allRects.push_back(this->actualRotatedRect);
					this->allTexts.push_back(text);
					this->allClrs.push_back(this->actualClr);
					shift = true;
				}
				else // user canceled -> just draw rect that was there already
				{
					cv::rectangle(temporaryImg, cv::Rect(this->actualRect.x, this->actualRect.y, this->actualRect.width, this->actualRect.height), this->actualClr);
				}
				this->textMode = false;

				temporaryImg = this->img.clone();
				cv::imshow(this->name, temporaryImg);
				break;
			}

			// [C]olor change
			case C:
			case C2:
			{
				srand(time(NULL));
				int c1, c2, c3;
				c1 = rand() % 255;
				c2 = (c1 * c1) % 255;
				c3 = (c1 * c2) % 255;
				this->actualClr = cv::Scalar(c1, c2, c3);
				break;
			}

			//[S]hift between Moving Box Mode and Resizing Box Mode
			case S:
			case S2:
			{
				shift = !shift;
				break;
			}

			// [R]eset complete image
			case R:
			case R2:
			{
				this->img = this->originalImage.clone();
				cv::imshow(this->name, this->img);
				this->allRects.clear();
				this->allTexts.clear();
				break;
			}

			// [H]elp
			case H:
			case H2:
			{
				std::cout << std::endl;
				std::cout << "labelBox Help" << std::endl;
				std::cout << std::endl;
				std::cout << "* Draw box with mouse." << std::endl;
				std::cout << "* Press direction keys to move/resize drawn box before entering text." << std::endl;
				std::cout << "* Press [d] for rotating clockwise." << std::endl;
				std::cout << "* Press [a] for rotating anticlockwise." << std::endl;
				std::cout << "* Press [s] for switching between moving and resizing with direction keys." << std::endl;
				std::cout << "* Press [c] for color change." << std::endl;
				std::cout << "* Press Return for entering text in drawn and resized box." << std::endl;
				std::cout << "* Press left mouse button on box to show text." << std::endl;
				std::cout << "* Press right mouse button to delete a box (after text was written inside)." << std::endl;
				std::cout << "* Press [z] to show data of all drawn rects." << std::endl;
				std::cout << "* Press [r] to reset complete image." << std::endl;
				std::cout << "* Press [ESC] to quit/move on to next image." << std::endl;
				std::cout << std::endl;
				break;
			}

			// [Esc]ape to next image
			case ESC:
			case ESC2:
			{
				active = false;
				break;
			}

			// [z] Info
			case Z:
			case Z2:
			{
				for (unsigned int i = 0; i < this->allRects.size(); i++)
					std::cout << "Rect #" << i << ": " << this->allRects[i].center.x << "|" << this->allRects[i].center.y
						<< ", " << this->allRects[i].size.width << "|" << this->allRects[i].size.height << ", "
						<< this->allRects[i].angle << ", " << this->allTexts[i] << std::endl;

				break;
			}

			// Rotate clockwise
			case D:
			case D2:
			{
				this->rotationMode = true;
				this->actualRotatedRect.angle++;
				if (this->actualRotatedRect.angle == 180)
				this->actualRotatedRect.angle = 0;
				break;
			}

			// Rotate counter-clockwise
			case A:
			case A2:
			{
				this->rotationMode = true;
				this->actualRotatedRect.angle--;
				if (this->actualRotatedRect.angle == -180)
				this->actualRotatedRect.angle = 0;
				break;
			}

		}
		labelImage::drawRect(temporaryImg, this->actualRotatedRect, this->actualClr);
		cv::imshow(this->name, temporaryImg);
	}
}

/*
int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		std::cout << "error: not enough input parameters!" << std::endl;
		return -1;
	}

	std::vector<std::string> allImageNames;

	std::string arg(argv[1]);
	std::string imgpath = arg.substr(0, arg.find_last_of("/") + 1);

	if (imgpath.empty())
	{
		boost::filesystem::path full_path(boost::filesystem::current_path());
		imgpath = full_path.directory_string();
	}

	// read all image names in the folder
	boost::filesystem::path input_path(argv[1]);
	if (boost::filesystem::is_directory(input_path))
	{
		DIR *pDIR;
		struct dirent *entry;
		if ((pDIR = opendir(argv[1])))
		while ((entry = readdir(pDIR)))
		if (std::strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
		{
			std::string completeName = entry->d_name;
			std::string imgend = completeName.substr(completeName.find_last_of(".") + 1, completeName.length() - completeName.find_last_of("."));
			if (std::strcmp(imgend.c_str(), "png") == 0 || std::strcmp(imgend.c_str(), "PNG") == 0
					|| std::strcmp(imgend.c_str(), "JPG") == 0 || std::strcmp(imgend.c_str(), "jpg") == 0
					|| std::strcmp(imgend.c_str(), "jpeg") == 0 || std::strcmp(imgend.c_str(), "Jpeg") == 0
					|| std::strcmp(imgend.c_str(), "bmp") == 0 || std::strcmp(imgend.c_str(), "BMP") == 0
					|| std::strcmp(imgend.c_str(), "TIFF") == 0 || std::strcmp(imgend.c_str(), "tiff") == 0
					|| std::strcmp(imgend.c_str(), "tif") == 0 || std::strcmp(imgend.c_str(), "TIF") == 0)
			{
				std::string s = argv[1];
				if (s.at(s.length() - 1) != '/')
					s.append("/");
				s.append(entry->d_name);
				allImageNames.push_back(s);
				std::cout << "imagename: " << s << std::endl;
			}
		}
	}
	else
		allImageNames.push_back(argv[1]); //single image to be processed

	std::cout << "Images to be processed: " << allImageNames.size() << std::endl;

	std::vector<labelImage> allImages;

	for (unsigned int i = 0; i < allImageNames.size(); i++)
	{
		cv::Mat originalImage = cv::imread(allImageNames[i]); // originalImage like it used to be, never modify
		cv::Mat imageWithRects = cv::imread(allImageNames[i]); // image with all rects the user selected

		// Show originalImage
		std::string winName = "labelBox - ";
		winName.append(allImageNames[i]);
		cv::imshow(winName.c_str(), originalImage);
		cvMoveWindow(winName.c_str(), 0, 0);

		// labelImage object for everything
		labelImage labelImg(winName, imageWithRects, originalImage, cv::Scalar(0, 0, 0));

		labelImg.labelingLoop();

//			// Mouse CB function
//			cv::setMouseCallback(winName, onMouse, (void*)&labelImg);
//
//			bool active = true;
//
//			// loop till next image
//			while (active)
//			{
//			  cv::Mat temporaryImg;
//			  static bool shift = true;
//
//			  int c = cv::waitKey(0);
//			  //   std::cout << "Key pressed: " << c << std::endl;
//
//			  temporaryImg = labelImg.img.clone();
//			  //labelImg.temporaryImage = labelImg.img.clone();
//
//			  // Keyboard Input
//			  switch (c)
//			  {
//				// Direction Keys for moving or resizing the box
//				case LEFT:
//				case LEFT2:
//				{
//				  if (shift)
//				  {
//					labelImg.actualRotatedRect.center.x--;
//				  }
//				  else
//				  {
//					float dx = std::cos((labelImg.actualRotatedRect.angle / 180.0) * 3.14159265359);
//					float dy = std::sin((labelImg.actualRotatedRect.angle / 180.0) * 3.14159265359);
//
//					labelImg.actualRotatedRect.size.width--;
//					labelImg.actualRotatedRect.center.x -= 0.5 * dx;
//					labelImg.actualRotatedRect.center.y -= 0.5 * dy;
//				  }
//
//				  break;
//				}
//
//				case UP:
//				case UP2:
//				{
//				  if (shift)
//				  {
//					labelImg.actualRotatedRect.center.y--;
//				  }
//				  else
//				  {
//					float dx = std::sin((labelImg.actualRotatedRect.angle / 180.0) * 3.14159265359);
//					float dy = std::cos((labelImg.actualRotatedRect.angle / 180.0) * 3.14159265359);
//
//					labelImg.actualRotatedRect.size.height--;
//					labelImg.actualRotatedRect.center.x += 0.5 * dx;
//					labelImg.actualRotatedRect.center.y -= 0.5 * dy;
//				  }
//
//				  break;
//				}
//
//				case RIGHT:
//				case RIGHT2:
//				{
//				  if (shift)
//				  {
//					labelImg.actualRotatedRect.center.x++;
//				  }
//				  else
//				  {
//					float dx = std::cos((labelImg.actualRotatedRect.angle / 180.0) * 3.14159265359);
//					float dy = std::sin((labelImg.actualRotatedRect.angle / 180.0) * 3.14159265359);
//
//					labelImg.actualRotatedRect.size.width++;
//					labelImg.actualRotatedRect.center.x += 0.5 * dx;
//					labelImg.actualRotatedRect.center.y += 0.5 * dy;
//				  }
//
//				  break;
//				}
//
//				case DOWN:
//				case DOWN2:
//				{
//				  if (shift)
//				  {
//					labelImg.actualRotatedRect.center.y++;
//				  }
//				  else
//				  {
//					float dx = std::sin((labelImg.actualRotatedRect.angle / 180.0) * 3.14159265359);
//					float dy = std::cos((labelImg.actualRotatedRect.angle / 180.0) * 3.14159265359);
//
//					labelImg.actualRotatedRect.size.height++;
//					labelImg.actualRotatedRect.center.x -= 0.5 * dx;
//					labelImg.actualRotatedRect.center.y += 0.5 * dy;
//				  }
//
//				  break;
//				}
//
//				  // Enter Text Mode
//				case RETURN:
//				case RETURN2:
//				{
//				  cv::Mat textBox = labelImg.img.clone();
//
//				  float offsetX = labelImg.actualRotatedRect.size.width < 50 ? 50 : labelImg.actualRotatedRect.size.width;
//				  float offsetY = labelImg.actualRotatedRect.size.height < 50 ? 50 : labelImg.actualRotatedRect.size.height;
//
//				  if (labelImg.actualRotatedRect.center.x - offsetX < 0)
//					offsetX *= -1;
//
//				  if (labelImg.actualRotatedRect.center.y - offsetY < 0)
//					offsetY *= -1;
//
//				  cv::Rect textRect(labelImg.actualRotatedRect.center.x - offsetX, labelImg.actualRotatedRect.center.y
//					  - offsetY, 200, 30);
//
//				  cv::rectangle(textBox, textRect, cv::Scalar(255, 255, 255), -1);
//				  cv::rectangle(textBox, textRect, labelImg.actualClr);
//
//				  drawRect(textBox, labelImg.actualRotatedRect, labelImg.actualClr);
//				  cv::imshow(winName, textBox);
//
//				  labelImg.textMode = true;
//				  std::string text;
//
//				  if (input_text(textBox, winName, &text, textRect)) // text was written
//				  {
//					drawRect(labelImg.img, labelImg.actualRotatedRect, labelImg.actualClr);
//					labelImg.allRects.push_back(labelImg.actualRotatedRect);
//					labelImg.allTexts.push_back(text);
//					labelImg.allClrs.push_back(labelImg.actualClr);
//					shift = true;
//				  }
//				  else // user canceled -> just draw rect that was there already
//				  {
//					cv::rectangle(temporaryImg, cv::Rect(labelImg.actualRect.x, labelImg.actualRect.y,
//														 labelImg.actualRect.width, labelImg.actualRect.height),
//								  labelImg.actualClr);
//				  }
//				  labelImg.textMode = false;
//
//				  temporaryImg = labelImg.img.clone();
//				  cv::imshow(winName, temporaryImg);
//				  break;
//				}
//
//				  // [C]olor change
//				case C:
//				case C2:
//				{
//				  srand(time(NULL));
//				  int c1, c2, c3;
//				  c1 = rand() % 255;
//				  c2 = (c1 * c1) % 255;
//				  c3 = (c1 * c2) % 255;
//				  labelImg.actualClr = cv::Scalar(c1, c2, c3);
//				  break;
//				}
//
//				  //[S]hift between Moving Box Mode and Resizing Box Mode
//				case S:
//				case S2:
//				{
//				  shift = !shift;
//				  break;
//				}
//
//				  // [R]eset complete image
//				case R:
//				case R2:
//				{
//				  labelImg.img = labelImg.originalImage.clone();
//				  cv::imshow(winName, labelImg.img);
//				  labelImg.allRects.clear();
//				  labelImg.allTexts.clear();
//				  break;
//				}
//				  // [H]elp
//				case H:
//				case H2:
//				{
//				  std::cout << std::endl;
//				  std::cout << "labelBox Help" << std::endl;
//				  std::cout << std::endl;
//				  std::cout << "* Draw box with mouse." << std::endl;
//				  std::cout << "* Press direction keys to move/resize drawn box before entering text." << std::endl;
//				  std::cout << "* Press [d] for rotating clockwise." << std::endl;
//				  std::cout << "* Press [a] for rotating anticlockwise." << std::endl;
//				  std::cout << "* Press [s] for switching between moving and resizing with direction keys." << std::endl;
//				  std::cout << "* Press [c] for color change." << std::endl;
//				  std::cout << "* Press Return for entering text in drawn and resized box." << std::endl;
//				  std::cout << "* Press left mouse button on box to show text." << std::endl;
//				  std::cout << "* Press right mouse button to delete a box (after text was written inside)." << std::endl;
//				  std::cout << "* Press [z] to show data of all drawn rects." << std::endl;
//				  std::cout << "* Press [r] to reset complete image." << std::endl;
//				  std::cout << "* Press [ESC] to quit/move on to next image." << std::endl;
//				  std::cout << std::endl;
//				  break;
//				}
//
//				  // [Esc]ape to next image
//				case ESC:
//				case ESC2:
//				{
//				  active = false;
//				  break;
//				}
//
//				  // [z] Info
//				case Z:
//				case Z2:
//				{
//				  for (unsigned int i = 0; i < labelImg.allRects.size(); i++)
//					std::cout << "Rect #" << i << ": " << labelImg.allRects[i].center.x << "|" << labelImg.allRects[i].center.y
//						<< ", " << labelImg.allRects[i].size.width << "|" << labelImg.allRects[i].size.height << ", "
//						<< labelImg.allRects[i].angle << ", " << labelImg.allTexts[i] << std::endl;
//
//				  break;
//				}
//
//				  // Rotate clockwise
//				case D:
//				case D2:
//				{
//				  labelImg.rotationMode = true;
//				  labelImg.actualRotatedRect.angle++;
//				  if (labelImg.actualRotatedRect.angle == 180)
//					labelImg.actualRotatedRect.angle = 0;
//				  break;
//				}
//
//				  // Rotate anti-clockwise
//				case A:
//				case A2:
//				{
//				  labelImg.rotationMode = true;
//				  labelImg.actualRotatedRect.angle--;
//				  if (labelImg.actualRotatedRect.angle == -180)
//					labelImg.actualRotatedRect.angle = 0;
//				  break;
//				}
//
//			  }
//			  drawRect(temporaryImg, labelImg.actualRotatedRect, labelImg.actualClr);
//			  cv::imshow(winName, temporaryImg);
//
//			}
		allImages.push_back(labelImg);
	}
	std::cout << "Saving Data." << std::endl;

	time_t t;
	t = time(NULL);
	std::stringstream ss;
	std::string actTime;
	ss << t;
	ss >> actTime;

	labelImage::writeTxt(allImages, imgpath, actTime);
	return 0;
}
*/
