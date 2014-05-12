/********************************************************************************
** Form generated from reading UI file 'window.ui'
**
** Created: Mon Mar 24 14:39:01 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WINDOW_H
#define UI_WINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Window
{
public:
    QLabel *label;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QLabel *componentNameLabel;
    QLineEdit *componentName;
    QSpacerItem *verticalSpacer;
    QPushButton *addButton;

    void setupUi(QMainWindow *Window)
    {
        if (Window->objectName().isEmpty())
            Window->setObjectName(QString::fromUtf8("Window"));
        Window->resize(400, 120);
        Window->setMinimumSize(QSize(400, 120));
        Window->setMaximumSize(QSize(400, 120));
        label = new QLabel(Window);
        label->setObjectName(QString::fromUtf8("label"));
        label->setMinimumSize(QSize(400, 120));
        label->setMaximumSize(QSize(400, 120));
        verticalLayoutWidget = new QWidget(label);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(10, 10, 380, 100));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        componentNameLabel = new QLabel(verticalLayoutWidget);
        componentNameLabel->setObjectName(QString::fromUtf8("componentNameLabel"));

        verticalLayout->addWidget(componentNameLabel);

        componentName = new QLineEdit(verticalLayoutWidget);
        componentName->setObjectName(QString::fromUtf8("componentName"));

        verticalLayout->addWidget(componentName);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        addButton = new QPushButton(verticalLayoutWidget);
        addButton->setObjectName(QString::fromUtf8("addButton"));

        verticalLayout->addWidget(addButton);

        Window->setCentralWidget(label);

        retranslateUi(Window);

        QMetaObject::connectSlotsByName(Window);
    } // setupUi

    void retranslateUi(QMainWindow *Window)
    {
        Window->setWindowTitle(QApplication::translate("Window", "AutoPnP Window", 0, QApplication::UnicodeUTF8));
        label->setText(QString());
        componentNameLabel->setText(QApplication::translate("Window", "Component Name:", 0, QApplication::UnicodeUTF8));
        addButton->setText(QApplication::translate("Window", "Add", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class Window: public Ui_Window {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WINDOW_H
