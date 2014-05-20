# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'window.ui'
#
# Created: Tue May 13 13:42:42 2014
#      by: PyQt4 UI code generator 4.9.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(581, 196)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.horizontalLayout = QtGui.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.groupBox = QtGui.QGroupBox(self.centralwidget)
        self.groupBox.setObjectName(_fromUtf8("groupBox"))
        self.verticalLayout_2 = QtGui.QVBoxLayout(self.groupBox)
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.rDWA = QtGui.QRadioButton(self.groupBox)
        self.rDWA.setObjectName(_fromUtf8("rDWA"))
        self.verticalLayout_2.addWidget(self.rDWA)
        self.rLin = QtGui.QRadioButton(self.groupBox)
        self.rLin.setObjectName(_fromUtf8("rLin"))
        self.verticalLayout_2.addWidget(self.rLin)
        self.verticalLayout.addWidget(self.groupBox)
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.verticalLayout_3 = QtGui.QVBoxLayout()
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
        self.gripper = QtGui.QGroupBox(self.centralwidget)
        self.gripper.setObjectName(_fromUtf8("gripper"))
        self.verticalLayout_4 = QtGui.QVBoxLayout(self.gripper)
        self.verticalLayout_4.setObjectName(_fromUtf8("verticalLayout_4"))
        self.btnOpen = QtGui.QPushButton(self.gripper)
        self.btnOpen.setObjectName(_fromUtf8("btnOpen"))
        self.verticalLayout_4.addWidget(self.btnOpen)
        self.btnClose = QtGui.QPushButton(self.gripper)
        self.btnClose.setObjectName(_fromUtf8("btnClose"))
        self.verticalLayout_4.addWidget(self.btnClose)
        self.verticalLayout_3.addWidget(self.gripper)
        spacerItem1 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout_3.addItem(spacerItem1)
        self.horizontalLayout.addLayout(self.verticalLayout_3)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QtGui.QApplication.translate("MainWindow", "Control GUI of Chromosome", None, QtGui.QApplication.UnicodeUTF8))
        self.groupBox.setTitle(QtGui.QApplication.translate("MainWindow", "Navigation", None, QtGui.QApplication.UnicodeUTF8))
        self.rDWA.setText(QtGui.QApplication.translate("MainWindow", "Dynamic Window Apporach", None, QtGui.QApplication.UnicodeUTF8))
        self.rLin.setText(QtGui.QApplication.translate("MainWindow", "Linear Navigation", None, QtGui.QApplication.UnicodeUTF8))
        self.gripper.setTitle(QtGui.QApplication.translate("MainWindow", "Gripper", None, QtGui.QApplication.UnicodeUTF8))
        self.btnOpen.setText(QtGui.QApplication.translate("MainWindow", "Open Gripper", None, QtGui.QApplication.UnicodeUTF8))
        self.btnClose.setText(QtGui.QApplication.translate("MainWindow", "Close Gripper", None, QtGui.QApplication.UnicodeUTF8))

