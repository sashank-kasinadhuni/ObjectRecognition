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
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_PCLViewer
{
public:
    QWidget *centralwidget;
    QVTKWidget *qvtkWidget;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
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
    QDoubleSpinBox *x_rotation;
    QDoubleSpinBox *y_rotation;
    QDoubleSpinBox *z_rotation;

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
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(40, 60, 101, 21));
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
        label_3->setGeometry(QRect(40, 190, 131, 31));
        label_3->setFont(font);
        saveButton = new QPushButton(centralwidget);
        saveButton->setObjectName(QString::fromUtf8("saveButton"));
        saveButton->setGeometry(QRect(10, 620, 131, 41));
        randomcolorbutton = new QPushButton(centralwidget);
        randomcolorbutton->setObjectName(QString::fromUtf8("randomcolorbutton"));
        randomcolorbutton->setGeometry(QRect(10, 670, 131, 41));
        label_5 = new QLabel(centralwidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(40, 260, 161, 41));
        QFont font1;
        font1.setPointSize(16);
        font1.setBold(true);
        font1.setWeight(75);
        label_5->setFont(font1);
        label_6 = new QLabel(centralwidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(40, 370, 131, 31));
        label_6->setFont(font1);
        label_7 = new QLabel(centralwidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(40, 470, 141, 31));
        label_7->setFont(font1);
        x_threshold_left = new QDoubleSpinBox(centralwidget);
        x_threshold_left->setObjectName(QString::fromUtf8("x_threshold_left"));
        x_threshold_left->setGeometry(QRect(40, 316, 71, 31));
        x_threshold_left->setDecimals(3);
        x_threshold_left->setMinimum(-4);
        x_threshold_left->setMaximum(4);
        x_threshold_left->setSingleStep(0.01);
        x_threshold_left->setValue(-4);
        x_threshold_right = new QDoubleSpinBox(centralwidget);
        x_threshold_right->setObjectName(QString::fromUtf8("x_threshold_right"));
        x_threshold_right->setGeometry(QRect(170, 316, 71, 31));
        x_threshold_right->setDecimals(3);
        x_threshold_right->setMinimum(-4);
        x_threshold_right->setMaximum(4);
        x_threshold_right->setSingleStep(0.01);
        x_threshold_right->setValue(4);
        y_threshold_bottom = new QDoubleSpinBox(centralwidget);
        y_threshold_bottom->setObjectName(QString::fromUtf8("y_threshold_bottom"));
        y_threshold_bottom->setGeometry(QRect(40, 416, 71, 31));
        y_threshold_bottom->setDecimals(3);
        y_threshold_bottom->setMinimum(-4);
        y_threshold_bottom->setMaximum(4);
        y_threshold_bottom->setSingleStep(0.01);
        y_threshold_bottom->setValue(-4);
        y_threshold_top = new QDoubleSpinBox(centralwidget);
        y_threshold_top->setObjectName(QString::fromUtf8("y_threshold_top"));
        y_threshold_top->setGeometry(QRect(170, 416, 71, 31));
        y_threshold_top->setDecimals(3);
        y_threshold_top->setMinimum(-4);
        y_threshold_top->setMaximum(4);
        y_threshold_top->setSingleStep(0.01);
        y_threshold_top->setValue(4);
        z_threshold_near = new QDoubleSpinBox(centralwidget);
        z_threshold_near->setObjectName(QString::fromUtf8("z_threshold_near"));
        z_threshold_near->setGeometry(QRect(40, 516, 71, 31));
        z_threshold_near->setDecimals(3);
        z_threshold_near->setMinimum(-4);
        z_threshold_near->setMaximum(5);
        z_threshold_near->setSingleStep(0.01);
        z_threshold_near->setValue(-4);
        z_threshold_far = new QDoubleSpinBox(centralwidget);
        z_threshold_far->setObjectName(QString::fromUtf8("z_threshold_far"));
        z_threshold_far->setGeometry(QRect(170, 516, 71, 31));
        z_threshold_far->setDecimals(3);
        z_threshold_far->setMaximum(5);
        z_threshold_far->setSingleStep(0.01);
        z_threshold_far->setValue(5);
        x_rotation = new QDoubleSpinBox(centralwidget);
        x_rotation->setObjectName(QString::fromUtf8("x_rotation"));
        x_rotation->setGeometry(QRect(180, 60, 71, 31));
        x_rotation->setDecimals(3);
        x_rotation->setMinimum(-180);
        x_rotation->setMaximum(180);
        x_rotation->setSingleStep(0.01);
        y_rotation = new QDoubleSpinBox(centralwidget);
        y_rotation->setObjectName(QString::fromUtf8("y_rotation"));
        y_rotation->setGeometry(QRect(180, 126, 71, 31));
        y_rotation->setDecimals(3);
        y_rotation->setMinimum(-180);
        y_rotation->setMaximum(180);
        y_rotation->setSingleStep(0.01);
        z_rotation = new QDoubleSpinBox(centralwidget);
        z_rotation->setObjectName(QString::fromUtf8("z_rotation"));
        z_rotation->setGeometry(QRect(180, 190, 71, 31));
        z_rotation->setDecimals(3);
        z_rotation->setMinimum(-180);
        z_rotation->setMaximum(180);
        z_rotation->setSingleStep(0.01);
        PCLViewer->setCentralWidget(centralwidget);

        retranslateUi(PCLViewer);

        QMetaObject::connectSlotsByName(PCLViewer);
    } // setupUi

    void retranslateUi(QMainWindow *PCLViewer)
    {
        PCLViewer->setWindowTitle(QApplication::translate("PCLViewer", "PCLViewer", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("PCLViewer", "X rotation ", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("PCLViewer", "Y rotation", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("PCLViewer", "Z rotation", 0, QApplication::UnicodeUTF8));
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
