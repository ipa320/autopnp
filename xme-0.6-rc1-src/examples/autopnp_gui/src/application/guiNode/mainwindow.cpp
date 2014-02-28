#include "mainwindow.h"
#include "wrapperHeader.h"
#include "ui_mainwindow.h"

#include <iostream>

static bool initialized = false;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
	initialized = true;

	connect(this, SIGNAL(writeTextSignal(QString)), this, SLOT(writeText(QString)));
	connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(buttonPushed()));
	connect(ui->pushButtonQuit, SIGNAL(clicked()), this, SLOT(terminateApplication()));
	connect(this, SIGNAL(displayImageSignal(QImage)), this, SLOT(displayImage(QImage)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::emitWriteTextSignal(char* text)
{
	QString textToWrite(text); // = QString::QString(text);
	emit writeTextSignal(textToWrite);
}

void MainWindow::writeText(QString text)
{
	if(initialized)
	{
		ui->textEdit->append(text);
	}
}

void MainWindow::buttonPushed()
{
	int state = 1;
	writeButtonStateWrapper(state);
}

void MainWindow::terminateApplication()
{
	QApplication::quit();
	terminateApplicationWrapper();
}

void MainWindow::displayImage(QImage image)
{
	if(initialized)
	{
		ui->imageDisplay->setPixmap(QPixmap::fromImage(image));
	}
}

void MainWindow::emitDisplayImage(unsigned int width, unsigned int height, unsigned int step, const std::vector<unsigned char>& data)
{
	// deep copy
	QImage image(width, height, QImage::Format_RGB888);
	for (int v=0; v<height; ++v)
	{
		uchar* imgdata = image.scanLine(v);
		for (int u=0; u<3*width; ++u)
		{
			imgdata[u] = data[v*step + u];
//			imgdata[u] = data[v*step + u+2];
//			imgdata[u+1] = data[v*step + u+1];
//			imgdata[u+2] = data[v*step + u];
		}
	}
	emit displayImageSignal(image);
	//ui->textEdit->setText("Callback");
}
