/********************************************************************************
** Form generated from reading UI file 'capabilitiesWindow.ui'
**
** Created: Fri Mar 28 08:49:48 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CAPABILITIESWINDOW_H
#define UI_CAPABILITIESWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QWidget>
#include <QtWebKit/QWebView>

QT_BEGIN_NAMESPACE

class Ui_CapabilitiesWindow
{
public:
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QWebView *webView;

    void setupUi(QMainWindow *CapabilitiesWindow)
    {
        if (CapabilitiesWindow->objectName().isEmpty())
            CapabilitiesWindow->setObjectName(QString::fromUtf8("CapabilitiesWindow"));
        CapabilitiesWindow->resize(800, 600);
        centralwidget = new QWidget(CapabilitiesWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setSpacing(0);
        gridLayout->setContentsMargins(0, 0, 0, 0);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        webView = new QWebView(centralwidget);
        webView->setObjectName(QString::fromUtf8("webView"));
        webView->setUrl(QUrl(QString::fromUtf8("about:blank")));

        gridLayout->addWidget(webView, 0, 0, 1, 1);

        CapabilitiesWindow->setCentralWidget(centralwidget);

        retranslateUi(CapabilitiesWindow);

        QMetaObject::connectSlotsByName(CapabilitiesWindow);
    } // setupUi

    void retranslateUi(QMainWindow *CapabilitiesWindow)
    {
        CapabilitiesWindow->setWindowTitle(QApplication::translate("CapabilitiesWindow", "Capabilities View (Chromosome)", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class CapabilitiesWindow: public Ui_CapabilitiesWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CAPABILITIESWINDOW_H
