#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <vector>

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void emitWriteTextSignal(char* text);
    void emitDisplayImage(unsigned int width, unsigned int height, unsigned int step, const std::vector<unsigned char>& data);

signals:
	void writeTextSignal(QString text);
	void displayImageSignal(QImage image);

private slots:
	void writeText(QString text);
	void buttonPushed();
    void displayImage(QImage image);

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
