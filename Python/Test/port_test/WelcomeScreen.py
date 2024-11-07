# -*- coding: utf-8 -*-
from PyQt5 import QtCore, QtGui, QtWidgets
class WelcomeScreen(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(1024, 600)
        self.pushButton = QtWidgets.QPushButton(Form)
        self.pushButton.setGeometry(QtCore.QRect(590, 420, 300, 60))
        font = QtGui.QFont()
        font.setFamily("楷体")
        font.setPointSize(30)
        font.setBold(True)
        font.setUnderline(False)
        font.setWeight(75)
        self.pushButton.setFont(font)
        self.pushButton.setStyleSheet("QPushButton:hover {\n"
"    border: 2px solid green;\n"
"}\n"
"QPushButton {\n"
"    border: none;\n"
"}\n"
"\n"
"\n"
"")
        self.pushButton.setObjectName("pushButton")
        self.label = QtWidgets.QLabel(Form)
        self.label.setGeometry(QtCore.QRect(0, 0, 1024, 600))
        self.label.setText("")
        self.label.setPixmap(QtGui.QPixmap("icon/ResizedImage.png"))
        self.label.setObjectName("label")
        self.textEdit = QtWidgets.QTextEdit(Form)
        self.textEdit.setGeometry(QtCore.QRect(520, 190, 421, 191))
        font = QtGui.QFont()
        font.setFamily("楷体")
        self.textEdit.setFont(font)
        self.textEdit.setStyleSheet("background: transparent;\n"
"border: none;\n"
"")
        self.textEdit.setObjectName("textEdit")
        self.label.raise_()
        self.textEdit.raise_()
        self.pushButton.raise_()

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.pushButton.setText(_translate("Form", "点击继续"))
        self.textEdit.setMarkdown(_translate("Form", "**果园作业机械研究团队**\n"
"\n"
""))
        self.textEdit.setHtml(_translate("Form", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'楷体\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:45pt; font-weight:600;\">果园作业机械研究团队</span></p></body></html>"))
