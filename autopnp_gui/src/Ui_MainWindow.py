# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'window.ui'
#
# Created: Tue May 20 11:04:19 2014
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
        MainWindow.resize(581, 195)
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
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.btnB = QtGui.QPushButton(self.groupBox)
        self.btnB.setObjectName(_fromUtf8("btnB"))
        self.horizontalLayout_2.addWidget(self.btnB)
        self.btnA = QtGui.QPushButton(self.groupBox)
        self.btnA.setObjectName(_fromUtf8("btnA"))
        self.horizontalLayout_2.addWidget(self.btnA)
        self.verticalLayout_2.addLayout(self.horizontalLayout_2)
        self.verticalLayout.addWidget(self.groupBox)
        self.groupBox_2 = QtGui.QGroupBox(self.centralwidget)
        self.groupBox_2.setObjectName(_fromUtf8("groupBox_2"))
        self.horizontalLayout_3 = QtGui.QHBoxLayout(self.groupBox_2)
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.btnToolChange = QtGui.QPushButton(self.groupBox_2)
        self.btnToolChange.setObjectName(_fromUtf8("btnToolChange"))
        self.horizontalLayout_3.addWidget(self.btnToolChange)
        self.verticalLayout.addWidget(self.groupBox_2)
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
        self.verticalLayout_14 = QtGui.QVBoxLayout()
        self.verticalLayout_14.setObjectName(_fromUtf8("verticalLayout_14"))
        self.vacuum = QtGui.QGroupBox(self.centralwidget)
        self.vacuum.setObjectName(_fromUtf8("vacuum"))
        self.verticalLayout_141 = QtGui.QVBoxLayout(self.vacuum)
        self.verticalLayout_141.setObjectName(_fromUtf8("verticalLayout_141"))
        self.btnOn = QtGui.QPushButton(self.vacuum)
        self.btnOn.setObjectName(_fromUtf8("btnOn"))
        self.verticalLayout_141.addWidget(self.btnOn)
        self.btnOff = QtGui.QPushButton(self.vacuum)
        self.btnOff.setObjectName(_fromUtf8("btnOff"))
        self.verticalLayout_141.addWidget(self.btnOff)
        self.verticalLayout_14.addWidget(self.vacuum)
        spacerItem2 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout_14.addItem(spacerItem2)
        self.horizontalLayout.addLayout(self.verticalLayout_14)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QtGui.QApplication.translate("MainWindow", "Control GUI of Chromosome", None, QtGui.QApplication.UnicodeUTF8))
        self.groupBox.setTitle(QtGui.QApplication.translate("MainWindow", "Navigation", None, QtGui.QApplication.UnicodeUTF8))
        self.rDWA.setText(QtGui.QApplication.translate("MainWindow", "Dynamic Window Apporach", None, QtGui.QApplication.UnicodeUTF8))
        self.rLin.setText(QtGui.QApplication.translate("MainWindow", "Linear Navigation", None, QtGui.QApplication.UnicodeUTF8))
        self.btnB.setText(QtGui.QApplication.translate("MainWindow", "A", None, QtGui.QApplication.UnicodeUTF8))
        self.btnA.setText(QtGui.QApplication.translate("MainWindow", "B", None, QtGui.QApplication.UnicodeUTF8))
        self.groupBox_2.setTitle(QtGui.QApplication.translate("MainWindow", "Actions", None, QtGui.QApplication.UnicodeUTF8))
        self.btnToolChange.setText(QtGui.QApplication.translate("MainWindow", "Tool Change", None, QtGui.QApplication.UnicodeUTF8))
        self.gripper.setTitle(QtGui.QApplication.translate("MainWindow", "Gripper", None, QtGui.QApplication.UnicodeUTF8))
        self.btnOpen.setText(QtGui.QApplication.translate("MainWindow", "Open Gripper", None, QtGui.QApplication.UnicodeUTF8))
        self.btnClose.setText(QtGui.QApplication.translate("MainWindow", "Close Gripper", None, QtGui.QApplication.UnicodeUTF8))
        self.vacuum.setTitle(QtGui.QApplication.translate("MainWindow", "Vacuum Cleaner", None, QtGui.QApplication.UnicodeUTF8))
        self.btnOn.setText(QtGui.QApplication.translate("MainWindow", "Power On", None, QtGui.QApplication.UnicodeUTF8))
        self.btnOff.setText(QtGui.QApplication.translate("MainWindow", "Power Off", None, QtGui.QApplication.UnicodeUTF8))

