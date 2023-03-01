# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'startupWin.ui'
#
# Created by: PyQt5 UI code generator 5.13.2
#
# WARNING! All changes made in this file will be lost!
import os
import sys
import time
from datetime import datetime
import calibration
from PyQt5.QtGui import QPixmap, QIcon
from PyQt5.QtWidgets import QApplication, QMainWindow, QDialog
from pywin.Demos.app.basictimerapp import app

from PyQt5 import QtCore, QtGui, QtWidgets
# from Gui.myWin import StartWin

from PyQt5 import QtCore, QtGui, QtWidgets

# from mainWin import MainWin


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        MainWindow.setWindowIcon(QIcon('Icon/rabbit.jpg'))
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QtCore.QRect(270, 370, 271, 51))
        self.pushButton.setObjectName("pushButton")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(270, 110, 271, 221))
        self.label.setText("")
        self.label.setPixmap(QtGui.QPixmap("Icon/rabbit.jpg"))
        self.label.setScaledContents(True)
        self.label.setObjectName("label")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 26))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "3D重建"))
        self.pushButton.setText(_translate("MainWindow", "点击开始3D重建"))


class StartWin(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(StartWin, self).__init__(parent)
        self.setupUi(self)
        self.addWidgets()

    def addWidgets(self):
        self.set_name_tip = QtWidgets.QLabel(self.centralwidget)
        self.set_name_tip.setText("请输入工程文件名")
        self.set_name_tip.setGeometry(QtCore.QRect(350, 350, 150, 40))
        font = QtGui.QFont()
        font.setFamily("幼圆")
        font.setPointSize(10)
        self.set_name_tip.setFont(font)
        self.set_name_tip.setObjectName("setnametip")
        self.set_name_tip.close()

        self.line_edit = QtWidgets.QLineEdit(self.centralwidget)
        self.line_edit.setGeometry(QtCore.QRect(270, 390, 271, 40))
        font = QtGui.QFont()
        font.setFamily("幼圆")
        font.setPointSize(10)
        self.line_edit.setFont(font)
        self.line_edit.setObjectName("LineEdit")
        self.line_edit.close()

        self.name_sub_button = QtWidgets.QPushButton(self.centralwidget)
        self.name_sub_button.setGeometry(QtCore.QRect(270, 438, 120, 40))
        font = QtGui.QFont()
        font.setFamily("幼圆")
        font.setPointSize(10)
        self.name_sub_button.setFont(font)
        self.name_sub_button.setObjectName("namesubButton")
        self.name_sub_button.setText("确认")
        self.name_sub_button.close()

        self.cancle_button = QtWidgets.QPushButton(self.centralwidget)
        self.cancle_button.setGeometry(QtCore.QRect(420, 438, 120, 40))
        font = QtGui.QFont()
        font.setFamily("幼圆")
        font.setPointSize(10)
        self.cancle_button.setFont(font)
        self.cancle_button.setObjectName("cancleButton")
        self.cancle_button.setText("取消")
        self.cancle_button.close()

        # 绑定事件
        self.pushButton.clicked.connect(self.setName)
        self.cancle_button.clicked.connect(self.cancleSetName)
        self.name_sub_button.clicked.connect(self.subName)

    def getProjectName(self):
        return self.project_name

    def setName(self):
        self.pushButton.close()
        self.cancle_button.show()
        self.name_sub_button.show()
        self.set_name_tip.show()
        self.line_edit.show()

    def cancleSetName(self):
        self.pushButton.show()
        self.cancle_button.close()
        self.name_sub_button.close()
        self.set_name_tip.close()
        self.line_edit.close()

    def subName(self):
        Input_text = self.line_edit.text()
        if Input_text != "":
            self.project_name = Input_text
        else:
            self.project_name = "Project-" + str(time.strftime("%Y-%m-%d-%H_%M_%S", time.localtime()))
            if os.path.exists(self.project_name):
                print('工程已存在')
            else:
                os.makedirs('../'+self.project_name)# 默认工程文件名
                print('工程已创建')
        print("project name:", self.project_name)

        # 保存文件名到txt文件中
        with open("../project_name.txt", "w") as f:
            f.write(self.project_name)
            f.close()
        self.close()

        # main_win = MainWin()
        # sys.exit(app.exec_())
app = QApplication(sys.argv)
Start =StartWin()
Start.show()
sys.exit(app.exec_())









