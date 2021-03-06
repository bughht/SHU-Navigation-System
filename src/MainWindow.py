# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'test1.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.

import os
import sys
from PyQt5.QtGui import QIcon
from PyQt5 import QtCore
from PyQt5 import QtWidgets
from MapShower import MapShower
from PyQt5 import QtWebEngineWidgets


class Ui_MainWindow(object):
    def __init__(self):
        self.mapshower = MapShower("../data/map_1.osm")
        print(self.mapshower.OP.maxlat)
        print(self.mapshower.OP.minlat)
        print(self.mapshower.OP.maxlon)
        print(self.mapshower.OP.minlon)
        # print(self.mapshower.building_info)

    # UI初始化
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1434, 790)
        MainWindow.setFixedSize(MainWindow.width(), MainWindow.height())
        self.effect_shadow = QtWidgets.QGraphicsDropShadowEffect()
        self.effect_shadow.setOffset(0, 0)  # 偏移
        self.effect_shadow.setBlurRadius(30)  # 阴影半径
        self.effect_shadow.setColor(QtCore.Qt.gray)  # 阴影颜色

        self.centralwidget = QtWidgets.QWidget(MainWindow)  # 画布
        self.centralwidget.setObjectName("centralwidget")
        # self.centralwidget.setStyleSheet("QWidget{background-color:white;}")
        self.comboBox = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox.setGeometry(QtCore.QRect(310, 430, 171, 31))
        self.comboBox.setObjectName("comboBox")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(60, 20, 1301, 131))
        self.label.setStyleSheet("font: 22pt \"楷体\";\n"
                                 "color: #023047;\n"
                                 "background-color: #83C5BE;\n"
                                 "border-radius: 40px;"
                                 )
        # self.label.setGraphicsEffect(self.effect_shadow)
        self.label.setTextFormat(QtCore.Qt.AutoText)
        self.label.setScaledContents(False)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setWordWrap(False)
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(60, 170, 221, 71))
        self.label_2.setStyleSheet("font: 14pt \"楷体\";")
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(100, 250, 131, 21))
        self.label_3.setStyleSheet("font: 14pt \"楷体\";")
        self.label_3.setObjectName("label_3")
        self.textBrowser_2 = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser_2.setGeometry(QtCore.QRect(310, 250, 171, 31))
        self.textBrowser_2.setObjectName("textBrowser_2")

        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setGeometry(QtCore.QRect(60, 290, 221, 71))
        self.label_4.setStyleSheet("font: 14pt \"楷体\";")
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setGeometry(QtCore.QRect(100, 370, 131, 21))
        self.label_5.setStyleSheet("font: 14pt \"楷体\";")
        self.label_5.setObjectName("label_5")
        self.textBrowser_4 = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser_4.setGeometry(QtCore.QRect(310, 370, 171, 31))
        self.textBrowser_4.setObjectName("textBrowser_4")
        self.label_6 = QtWidgets.QLabel(self.centralwidget)
        self.label_6.setGeometry(QtCore.QRect(80, 410, 221, 71))
        self.label_6.setStyleSheet("font: 14pt \"楷体\";")
        self.label_6.setObjectName("label_6")
        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QtCore.QRect(210, 500, 161, 51))
        self.pushButton.setGraphicsEffect(self.effect_shadow)
        '''self.pushButton.setStyleSheet("font: 16pt \"黑体\";\n"
                                      "color: #E29578;\n"
                                      "background-color: #EDF6F9;"
                                      "border-radius: 20px;"
                                      "border: 2px groove gray;"
                                      "border-style: outset;")
        '''
        self.pushButton.setStyleSheet("QPushButton{\n"
                                      "    background:orange;\n"
                                      "    color:white;\n"
                                      # "    box-shadow: 1px 1px 3px;
                                      "font-size:16pt;border-radius: 24px;font-family: 黑体;\n"
                                      "font-weight:bold;\n"
                                      "}\n"
                                      "QPushButton:pressed{\n"
                                      "    background:black;\n"
                                      "}")
        self.pushButton.setObjectName("pushButton")
        self.label_7 = QtWidgets.QLabel(self.centralwidget)
        self.label_7.setGeometry(QtCore.QRect(140, 600, 131, 21))
        self.label_7.setStyleSheet("font: 14pt \"楷体\";")
        self.label_7.setObjectName("label_7")
        self.label_8 = QtWidgets.QLabel(self.centralwidget)
        self.label_8.setGeometry(QtCore.QRect(140, 660, 131, 21))
        self.label_8.setStyleSheet("font: 14pt \"楷体\";")
        self.label_8.setObjectName("label_8")
        self.textBrowser_5 = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser_5.setGeometry(QtCore.QRect(310, 590, 171, 31))
        self.textBrowser_5.setObjectName("textBrowser_5")
        self.textBrowser_6 = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser_6.setGeometry(QtCore.QRect(310, 660, 171, 31))
        self.textBrowser_6.setObjectName("textBrowser_6")
        self.webEngineView = QtWebEngineWidgets.QWebEngineView(
            self.centralwidget)
        self.webEngineView.setGeometry(QtCore.QRect(550, 160, 811, 601))
        self.path = "file:\\" + os.getcwd() + "\\school_map.html"
        self.path = self.path.replace('\\', '/')
        self.webEngineView.setUrl(QtCore.QUrl(self.path))
        self.webEngineView.setObjectName("webEngineView")
        # self.webEngineView.setGraphicsEffect(self.effect_shadow)
        self.comboBox_2 = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox_2.setGeometry(QtCore.QRect(310, 190, 171, 31))
        self.comboBox_2.setObjectName("comboBox_2")
        self.comboBox_2.currentIndexChanged[str].connect(self.update_combobox2)
        self.comboBox_3 = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox_3.setGeometry(QtCore.QRect(310, 310, 171, 31))
        self.comboBox_3.setObjectName("comboBox_3")
        self.comboBox_3.currentIndexChanged[str].connect(self.update_combobox3)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1434, 26))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        #self.statusbar = QtWidgets.QStatusBar(MainWindow)
        # self.statusbar.setObjectName("statusbar")
        # MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    # UI文本信息初始化
    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "SHU校园导航系统"))
        MainWindow.setWindowIcon(QIcon("../data/LOGO.ico"))
        self.comboBox.setItemText(0, _translate("MainWindow", "步行"))
        self.comboBox.setItemText(1, _translate("MainWindow", "骑行"))
        self.comboBox_2.addItems(self.mapshower.building_name)
        self.comboBox_2.setCurrentIndex(
            self.mapshower.building_name.index("伟长楼"))
        self.comboBox_3.addItems(self.mapshower.building_name)
        self.comboBox_3.setCurrentIndex(
            self.mapshower.building_name.index("吾馨楼"))
        self.label.setText(_translate("MainWindow", "您好，欢迎使用上海大学校园导航系统！"))
        self.label_2.setText(_translate("MainWindow", "请选择起始点位置："))
        self.label_3.setText(_translate("MainWindow", "起始点坐标："))
        self.label_4.setText(_translate("MainWindow", "请选择终止点位置："))
        self.label_5.setText(_translate("MainWindow", "终止点坐标："))
        self.label_6.setText(_translate("MainWindow", "请选择出行方式："))
        self.pushButton.setText(_translate("MainWindow", "出 发"))
        self.label_7.setText(_translate("MainWindow", "距离："))
        self.label_8.setText(_translate("MainWindow", "耗时："))
        self.pushButton.clicked.connect(self.click_success)

    def update_combobox2(self, building_name1):  # 起始点下拉框更新
        pos = self.mapshower.building_name.index(building_name1)
        nd = self.mapshower.building_info[pos][1][1]
        lat, lon = nd.lat, nd.lon
        self.textBrowser_2.setText("%.5f" % lat+','+"%.5f" % lon)  # 显示经纬度信息

    def update_combobox3(self, building_name2):  # 终止点下拉框更新
        pos = self.mapshower.building_name.index(building_name2)
        nd = self.mapshower.building_info[pos][1][1]
        lat, lon = nd.lat, nd.lon
        self.textBrowser_4.setText("%.5f" % lat+','+"%.5f" % lon)  # 显示经纬度信息

    def click_success(self):  # 出发按键回调
        building_name1 = self.comboBox_2.currentText()
        # self.cb.currentIndexChanged[int].connect(self.mapshower.building_name)
        building_name2 = self.comboBox_3.currentText()
        type = self.comboBox.currentText()
        distance, timecost = self.mapshower.update_path_building(
            building_name1, building_name2, type)  # 获取距离与时间
        self.webEngineView.setUrl(QtCore.QUrl(self.path))  # 更新网页信息
        self.textBrowser_5.setText("%.1f米" % distance)  # 显示距离信息
        m, s = divmod(timecost, 60)
        self.textBrowser_6.setText("%d分钟%d秒" % (m, s))  # 显示耗费时间


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    form = QtWidgets.QMainWindow()
    w = Ui_MainWindow()
    w.setupUi(form)
    form.show()
    sys.exit(app.exec_())
