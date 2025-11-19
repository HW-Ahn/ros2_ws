# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'tb3_control_bak2.ui'
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
from PySide6.QtWidgets import (QApplication, QDialog, QFrame, QLabel,
    QPushButton, QSizePolicy, QTextEdit, QWidget)

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        if not Dialog.objectName():
            Dialog.setObjectName(u"Dialog")
        Dialog.resize(1334, 1013)
        self.btn_left = QPushButton(Dialog)
        self.btn_left.setObjectName(u"btn_left")
        self.btn_left.setGeometry(QRect(824, 900, 71, 71))
        self.btn_left.setAutoDefault(False)
        self.btn_down = QPushButton(Dialog)
        self.btn_down.setObjectName(u"btn_down")
        self.btn_down.setGeometry(QRect(890, 900, 71, 71))
        self.btn_qr1 = QPushButton(Dialog)
        self.btn_qr1.setObjectName(u"btn_qr1")
        self.btn_qr1.setGeometry(QRect(80, 920, 61, 61))
        self.btn_buzzer = QPushButton(Dialog)
        self.btn_buzzer.setObjectName(u"btn_buzzer")
        self.btn_buzzer.setGeometry(QRect(190, 830, 95, 81))
        self.btn_right = QPushButton(Dialog)
        self.btn_right.setObjectName(u"btn_right")
        self.btn_right.setGeometry(QRect(960, 900, 71, 71))
        self.btn_up = QPushButton(Dialog)
        self.btn_up.setObjectName(u"btn_up")
        self.btn_up.setGeometry(QRect(890, 830, 71, 71))
        self.btn_start = QPushButton(Dialog)
        self.btn_start.setObjectName(u"btn_start")
        self.btn_start.setGeometry(QRect(420, 830, 95, 81))
        self.btn_qrscan = QPushButton(Dialog)
        self.btn_qrscan.setObjectName(u"btn_qrscan")
        self.btn_qrscan.setGeometry(QRect(300, 830, 95, 81))
        self.btn_switch = QPushButton(Dialog)
        self.btn_switch.setObjectName(u"btn_switch")
        self.btn_switch.setGeometry(QRect(80, 830, 95, 81))
        self.btn_estop = QPushButton(Dialog)
        self.btn_estop.setObjectName(u"btn_estop")
        self.btn_estop.setGeometry(QRect(1080, 830, 81, 71))
        self.btn_stop = QPushButton(Dialog)
        self.btn_stop.setObjectName(u"btn_stop")
        self.btn_stop.setGeometry(QRect(650, 830, 95, 171))
        self.btn_qr2 = QPushButton(Dialog)
        self.btn_qr2.setObjectName(u"btn_qr2")
        self.btn_qr2.setGeometry(QRect(160, 920, 61, 61))
        self.btn_qr3 = QPushButton(Dialog)
        self.btn_qr3.setObjectName(u"btn_qr3")
        self.btn_qr3.setGeometry(QRect(250, 920, 61, 61))
        self.btn_qr4 = QPushButton(Dialog)
        self.btn_qr4.setObjectName(u"btn_qr4")
        self.btn_qr4.setGeometry(QRect(330, 920, 61, 61))
        self.text_log1 = QTextEdit(Dialog)
        self.text_log1.setObjectName(u"text_log1")
        self.text_log1.setGeometry(QRect(20, 620, 611, 191))
        self.text_log2 = QTextEdit(Dialog)
        self.text_log2.setObjectName(u"text_log2")
        self.text_log2.setGeometry(QRect(650, 620, 641, 191))
        self.label_active = QLabel(Dialog)
        self.label_active.setObjectName(u"label_active")
        self.label_active.setGeometry(QRect(40, 600, 181, 21))
        self.btn_patrol_random = QPushButton(Dialog)
        self.btn_patrol_random.setObjectName(u"btn_patrol_random")
        self.btn_patrol_random.setGeometry(QRect(420, 920, 95, 81))
        self.btn_return = QPushButton(Dialog)
        self.btn_return.setObjectName(u"btn_return")
        self.btn_return.setGeometry(QRect(530, 920, 95, 81))
        self.frame = QFrame(Dialog)
        self.frame.setObjectName(u"frame")
        self.frame.setGeometry(QRect(20, 20, 611, 521))
        self.label_cam1 = QLabel(self.frame)
        self.label_cam1.setObjectName(u"label_cam1")
        self.label_cam1.setGeometry(QRect(0, 0, 611, 521))
        self.frame_cam2 = QFrame(Dialog)
        self.frame_cam2.setObjectName(u"frame_cam2")
        self.frame_cam2.setGeometry(QRect(650, 20, 641, 521))
        self.label_cam2 = QLabel(self.frame_cam2)
        self.label_cam2.setObjectName(u"label_cam2")
        self.label_cam2.setGeometry(QRect(0, 0, 641, 521))

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
        self.btn_qr2.setText(QCoreApplication.translate("Dialog", u"qr2", None))
        self.btn_qr3.setText(QCoreApplication.translate("Dialog", u"qr3", None))
        self.btn_qr4.setText(QCoreApplication.translate("Dialog", u"qr4", None))
        self.label_active.setText("")
        self.btn_patrol_random.setText(QCoreApplication.translate("Dialog", u"\ub09c\uc120\uc21c\ucc30", None))
        self.btn_return.setText(QCoreApplication.translate("Dialog", u"\ubcf5\uadc0", None))
        self.label_cam1.setText(QCoreApplication.translate("Dialog", u"cam1", None))
        self.label_cam2.setText(QCoreApplication.translate("Dialog", u"cam2", None))
    # retranslateUi

