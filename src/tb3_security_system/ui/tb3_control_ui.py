# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'tb3_control.ui'
##
## Created by: Qt User Interface Compiler version 6.9.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QDialog, QLabel, QPushButton,
    QSizePolicy, QTextEdit, QWidget)

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        if not Dialog.objectName():
            Dialog.setObjectName(u"Dialog")
        Dialog.resize(965, 887)
        self.btn_left = QPushButton(Dialog)
        self.btn_left.setObjectName(u"btn_left")
        self.btn_left.setGeometry(QRect(744, 770, 71, 71))
        self.btn_left.setAutoDefault(False)
        self.btn_down = QPushButton(Dialog)
        self.btn_down.setObjectName(u"btn_down")
        self.btn_down.setGeometry(QRect(810, 770, 71, 71))
        self.btn_qr1 = QPushButton(Dialog)
        self.btn_qr1.setObjectName(u"btn_qr1")
        self.btn_qr1.setGeometry(QRect(30, 770, 61, 61))
        self.btn_buzzer = QPushButton(Dialog)
        self.btn_buzzer.setObjectName(u"btn_buzzer")
        self.btn_buzzer.setGeometry(QRect(140, 680, 95, 81))
        self.btn_right = QPushButton(Dialog)
        self.btn_right.setObjectName(u"btn_right")
        self.btn_right.setGeometry(QRect(880, 770, 71, 71))
        self.btn_up = QPushButton(Dialog)
        self.btn_up.setObjectName(u"btn_up")
        self.btn_up.setGeometry(QRect(810, 700, 71, 71))
        self.btn_start = QPushButton(Dialog)
        self.btn_start.setObjectName(u"btn_start")
        self.btn_start.setGeometry(QRect(370, 680, 95, 81))
        self.btn_qrscan = QPushButton(Dialog)
        self.btn_qrscan.setObjectName(u"btn_qrscan")
        self.btn_qrscan.setGeometry(QRect(250, 680, 95, 81))
        self.btn_switch = QPushButton(Dialog)
        self.btn_switch.setObjectName(u"btn_switch")
        self.btn_switch.setGeometry(QRect(30, 680, 95, 81))
        self.btn_estop = QPushButton(Dialog)
        self.btn_estop.setObjectName(u"btn_estop")
        self.btn_estop.setGeometry(QRect(480, 680, 95, 81))
        self.btn_stop = QPushButton(Dialog)
        self.btn_stop.setObjectName(u"btn_stop")
        self.btn_stop.setGeometry(QRect(600, 680, 95, 171))
        self.label_cam1 = QLabel(Dialog)
        self.label_cam1.setObjectName(u"label_cam1")
        self.label_cam1.setGeometry(QRect(36, 36, 601, 271))
        self.label_cam2 = QLabel(Dialog)
        self.label_cam2.setObjectName(u"label_cam2")
        self.label_cam2.setGeometry(QRect(40, 370, 651, 251))
        self.btn_qr2 = QPushButton(Dialog)
        self.btn_qr2.setObjectName(u"btn_qr2")
        self.btn_qr2.setGeometry(QRect(110, 770, 61, 61))
        self.btn_qr3 = QPushButton(Dialog)
        self.btn_qr3.setObjectName(u"btn_qr3")
        self.btn_qr3.setGeometry(QRect(200, 770, 61, 61))
        self.btn_qr4 = QPushButton(Dialog)
        self.btn_qr4.setObjectName(u"btn_qr4")
        self.btn_qr4.setGeometry(QRect(280, 770, 61, 61))
        self.text_log1 = QTextEdit(Dialog)
        self.text_log1.setObjectName(u"text_log1")
        self.text_log1.setGeometry(QRect(740, 40, 181, 271))
        self.text_log2 = QTextEdit(Dialog)
        self.text_log2.setObjectName(u"text_log2")
        self.text_log2.setGeometry(QRect(740, 370, 181, 261))
        self.label_active = QLabel(Dialog)
        self.label_active.setObjectName(u"label_active")
        self.label_active.setGeometry(QRect(743, 8, 181, 21))
        self.btn_patrol_random = QPushButton(Dialog)
        self.btn_patrol_random.setObjectName(u"btn_patrol_random")
        self.btn_patrol_random.setGeometry(QRect(370, 770, 95, 81))
        self.btn_return = QPushButton(Dialog)
        self.btn_return.setObjectName(u"btn_return")
        self.btn_return.setGeometry(QRect(480, 770, 95, 81))

        self.retranslateUi(Dialog)

        QMetaObject.connectSlotsByName(Dialog)
    # setupUi

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", u"Dialog", None))
        self.btn_left.setText(QCoreApplication.translate("Dialog", u"<", None))
        self.btn_down.setText(QCoreApplication.translate("Dialog", u"v", None))
        self.btn_qr1.setText(QCoreApplication.translate("Dialog", u"qr1", None))
        self.btn_buzzer.setText(QCoreApplication.translate("Dialog", u"\ubd80\uc800", None))
        self.btn_right.setText(QCoreApplication.translate("Dialog", u">", None))
        self.btn_up.setText(QCoreApplication.translate("Dialog", u"^", None))
        self.btn_start.setText(QCoreApplication.translate("Dialog", u"\uc815\uc120\uc21c\ucc30", None))
        self.btn_qrscan.setText(QCoreApplication.translate("Dialog", u"qr\uc2a4\uce94", None))
        self.btn_switch.setText(QCoreApplication.translate("Dialog", u"\uad50\ub300", None))
        self.btn_estop.setText(QCoreApplication.translate("Dialog", u"\ube44\uc0c1\uc815\uc9c0", None))
        self.btn_stop.setText(QCoreApplication.translate("Dialog", u"\uc885\ub8cc", None))
        self.label_cam1.setText(QCoreApplication.translate("Dialog", u"cam1", None))
        self.label_cam2.setText(QCoreApplication.translate("Dialog", u"cam2", None))
        self.btn_qr2.setText(QCoreApplication.translate("Dialog", u"qr2", None))
        self.btn_qr3.setText(QCoreApplication.translate("Dialog", u"qr3", None))
        self.btn_qr4.setText(QCoreApplication.translate("Dialog", u"qr4", None))
        self.label_active.setText("")
        self.btn_patrol_random.setText(QCoreApplication.translate("Dialog", u"\ub09c\uc120\uc21c\ucc30", None))
        self.btn_return.setText(QCoreApplication.translate("Dialog", u"\ubcf5\uadc0", None))
    # retranslateUi

