/********************************************************************************
** Form generated from reading UI file 'pclviewer.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PCLVIEWER_H
#define UI_PCLVIEWER_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLCDNumber>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_PCLViewer
{
public:
    QWidget *centralwidget;
    QVTKWidget *qvtkWidget;
    QSlider *horizontalSlider_X_Angle;
    QSlider *horizontalSlider_Y_Angle;
    QSlider *horizontalSlider_Z_Angle;
    QLCDNumber *lcdNumber_x_angle;
    QLCDNumber *lcdNumber_y_angle;
    QLCDNumber *lcdNumber_z_angle;
    QSlider *horizontalSlider_p;
    QLCDNumber *lcdNumber_p;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QPushButton *saveButton;
    QPushButton *randomcolorbutton;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *label_7;
    QDoubleSpinBox *x_threshold_left;
    QDoubleSpinBox *x_threshold_right;
    QDoubleSpinBox *y_threshold_bottom;
    QDoubleSpinBox *y_threshold_top;
    QDoubleSpinBox *z_threshold_near;
    QDoubleSpinBox *z_threshold_far;

    void setupUi(QMainWindow *PCLViewer)
    {
        if (PCLViewer->objectName().isEmpty())
            PCLViewer->setObjectName(QString::fromUtf8("PCLViewer"));
        PCLViewer->resize(966, 806);
        PCLViewer->setMinimumSize(QSize(0, 0));
        PCLViewer->setMaximumSize(QSize(5000, 5000));
        centralwidget = new QWidget(PCLViewer);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        qvtkWidget = new QVTKWidget(centralwidget);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));
        qvtkWidget->setGeometry(QRect(300, 10, 640, 711));
        horizontalSlider_X_Angle = new QSlider(centralwidget);
        horizontalSlider_X_Angle->setObjectName(QString::fromUtf8("horizontalSlider_X_Angle"));
        horizontalSlider_X_Angle->setGeometry(QRect(30, 80, 241, 21));
        horizontalSlider_X_Angle->setMaximum(360);
        horizontalSlider_X_Angle->setPageStep(1);
        horizontalSlider_X_Angle->setValue(0);
        horizontalSlider_X_Angle->setOrientation(Qt::Horizontal);
        horizontalSlider_Y_Angle = new QSlider(centralwidget);
        horizontalSlider_Y_Angle->setObjectName(QString::fromUtf8("horizontalSlider_Y_Angle"));
        horizontalSlider_Y_Angle->setGeometry(QRect(30, 160, 241, 21));
        horizontalSlider_Y_Angle->setMaximum(360);
        horizontalSlider_Y_Angle->setPageStep(1);
        horizontalSlider_Y_Angle->setValue(0);
        horizontalSlider_Y_Angle->setOrientation(Qt::Horizontal);
        horizontalSlider_Z_Angle = new QSlider(centralwidget);
        horizontalSlider_Z_Angle->setObjectName(QString::fromUtf8("horizontalSlider_Z_Angle"));
        horizontalSlider_Z_Angle->setGeometry(QRect(30, 240, 241, 21));
        horizontalSlider_Z_Angle->setMaximum(360);
        horizontalSlider_Z_Angle->setPageStep(1);
        horizontalSlider_Z_Angle->setValue(0);
        horizontalSlider_Z_Angle->setOrientation(Qt::Horizontal);
        lcdNumber_x_angle = new QLCDNumber(centralwidget);
        lcdNumber_x_angle->setObjectName(QString::fromUtf8("lcdNumber_x_angle"));
        lcdNumber_x_angle->setGeometry(QRect(190, 40, 81, 41));
        lcdNumber_x_angle->setSmallDecimalPoint(false);
        lcdNumber_x_angle->setDigitCount(3);
        lcdNumber_x_angle->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_x_angle->setProperty("intValue", QVariant(0));
        lcdNumber_y_angle = new QLCDNumber(centralwidget);
        lcdNumber_y_angle->setObjectName(QString::fromUtf8("lcdNumber_y_angle"));
        lcdNumber_y_angle->setGeometry(QRect(190, 110, 81, 41));
        lcdNumber_y_angle->setDigitCount(3);
        lcdNumber_y_angle->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_y_angle->setProperty("intValue", QVariant(0));
        lcdNumber_z_angle = new QLCDNumber(centralwidget);
        lcdNumber_z_angle->setObjectName(QString::fromUtf8("lcdNumber_z_angle"));
        lcdNumber_z_angle->setGeometry(QRect(190, 190, 81, 41));
        lcdNumber_z_angle->setDigitCount(3);
        lcdNumber_z_angle->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_z_angle->setProperty("intValue", QVariant(0));
        horizontalSlider_p = new QSlider(centralwidget);
        horizontalSlider_p->setObjectName(QString::fromUtf8("horizontalSlider_p"));
        horizontalSlider_p->setGeometry(QRect(30, 320, 160, 29));
        horizontalSlider_p->setMinimum(1);
        horizontalSlider_p->setMaximum(6);
        horizontalSlider_p->setValue(2);
        horizontalSlider_p->setOrientation(Qt::Horizontal);
        lcdNumber_p = new QLCDNumber(centralwidget);
        lcdNumber_p->setObjectName(QString::fromUtf8("lcdNumber_p"));
        lcdNumber_p->setGeometry(QRect(200, 310, 81, 41));
        lcdNumber_p->setDigitCount(1);
        lcdNumber_p->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_p->setProperty("intValue", QVariant(2));
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(40, 50, 101, 21));
        QFont font;
        font.setPointSize(16);
        font.setBold(false);
        font.setItalic(false);
        font.setWeight(50);
        label->setFont(font);
        label_2 = new QLabel(centralwidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(40, 130, 101, 21));
        label_2->setFont(font);
        label_3 = new QLabel(centralwidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(40, 200, 131, 31));
        label_3->setFont(font);
        label_4 = new QLabel(centralwidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(30, 280, 141, 31));
        label_4->setFont(font);
        saveButton = new QPushButton(centralwidget);
        saveButton->setObjectName(QString::fromUtf8("saveButton"));
        saveButton->setGeometry(QRect(10, 620, 131, 41));
        randomcolorbutton = new QPushButton(centralwidget);
        randomcolorbutton->setObjectName(QString::fromUtf8("randomcolorbutton"));
        randomcolorbutton->setGeometry(QRect(10, 670, 131, 41));
        label_5 = new QLabel(centralwidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(40, 380, 161, 41));
        QFont font1;
        font1.setPointSize(16);
        font1.setBold(true);
        font1.setWeight(75);
        label_5->setFont(font1);
        label_6 = new QLabel(centralwidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(40, 460, 131, 31));
        label_6->setFont(font1);
        label_7 = new QLabel(centralwidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(40, 540, 141, 31));
        label_7->setFont(font1);
        x_threshold_left = new QDoubleSpinBox(centralwidget);
        x_threshold_left->setObjectName(QString::fromUtf8("x_threshold_left"));
        x_threshold_left->setGeometry(QRect(50, 420, 62, 27));
        x_threshold_left->setMinimum(-4);
        x_threshold_left->setMaximum(4);
        x_threshold_left->setSingleStep(0.1);
        x_threshold_left->setValue(-4);
        x_threshold_right = new QDoubleSpinBox(centralwidget);
        x_threshold_right->setObjectName(QString::fromUtf8("x_threshold_right"));
        x_threshold_right->setGeometry(QRect(160, 420, 62, 27));
        x_threshold_right->setMinimum(-4);
        x_threshold_right->setMaximum(4);
        x_threshold_right->setSingleStep(0.1);
        x_threshold_right->setValue(4);
        y_threshold_bottom = new QDoubleSpinBox(centralwidget);
        y_threshold_bottom->setObjectName(QString::fromUtf8("y_threshold_bottom"));
        y_threshold_bottom->setGeometry(QRect(50, 500, 62, 27));
        y_threshold_bottom->setMinimum(-4);
        y_threshold_bottom->setMaximum(4);
        y_threshold_bottom->setSingleStep(0.1);
        y_threshold_bottom->setValue(-4);
        y_threshold_top = new QDoubleSpinBox(centralwidget);
        y_threshold_top->setObjectName(QString::fromUtf8("y_threshold_top"));
        y_threshold_top->setGeometry(QRect(160, 500, 62, 27));
        y_threshold_top->setMinimum(-4);
        y_threshold_top->setMaximum(4);
        y_threshold_top->setSingleStep(0.1);
        y_threshold_top->setValue(4);
        z_threshold_near = new QDoubleSpinBox(centralwidget);
        z_threshold_near->setObjectName(QString::fromUtf8("z_threshold_near"));
        z_threshold_near->setGeometry(QRect(50, 580, 62, 27));
        z_threshold_near->setMinimum(-4);
        z_threshold_near->setMaximum(5);
        z_threshold_near->setSingleStep(0.1);
        z_threshold_near->setValue(-4);
        z_threshold_far = new QDoubleSpinBox(centralwidget);
        z_threshold_far->setObjectName(QString::fromUtf8("z_threshold_far"));
        z_threshold_far->setGeometry(QRect(160, 580, 62, 27));
        z_threshold_far->setMaximum(5);
        z_threshold_far->setSingleStep(0.1);
        z_threshold_far->setValue(5);
        PCLViewer->setCentralWidget(centralwidget);

        retranslateUi(PCLViewer);
        QObject::connect(horizontalSlider_p, SIGNAL(sliderMoved(int)), lcdNumber_p, SLOT(display(int)));
        QObject::connect(horizontalSlider_Z_Angle, SIGNAL(sliderMoved(int)), lcdNumber_z_angle, SLOT(display(int)));
        QObject::connect(horizontalSlider_Y_Angle, SIGNAL(sliderMoved(int)), lcdNumber_y_angle, SLOT(display(int)));
        QObject::connect(horizontalSlider_X_Angle, SIGNAL(sliderMoved(int)), lcdNumber_x_angle, SLOT(display(int)));

        QMetaObject::connectSlotsByName(PCLViewer);
    } // setupUi

    void retranslateUi(QMainWindow *PCLViewer)
    {
        PCLViewer->setWindowTitle(QApplication::translate("PCLViewer", "PCLViewer", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("PCLViewer", "X rotation ", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("PCLViewer", "Y rotation", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("PCLViewer", "Z rotation", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("PCLViewer", "Point size", 0, QApplication::UnicodeUTF8));
        saveButton->setText(QApplication::translate("PCLViewer", "Save Point Cloud", 0, QApplication::UnicodeUTF8));
        randomcolorbutton->setText(QApplication::translate("PCLViewer", "Random Colors", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("PCLViewer", "X Threshold", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("PCLViewer", "Y Threshold", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("PCLViewer", "Z Threshold", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class PCLViewer: public Ui_PCLViewer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PCLVIEWER_H
