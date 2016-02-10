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
#include <QtGui/QCheckBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QFrame>
#include <QtGui/QHeaderView>
#include <QtGui/QLCDNumber>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
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
    QDoubleSpinBox *nonMaxSuppBox;
    QLabel *nonMaxSupp;
    QLabel *salientRadius;
    QDoubleSpinBox *salientRadiusBox;
    QLabel *normalRadius;
    QLabel *borderRadius;
    QLabel *gamma21;
    QLabel *gamma32;
    QDoubleSpinBox *normalRadiusBox;
    QDoubleSpinBox *borderRadiusBox;
    QDoubleSpinBox *gamma21Box;
    QDoubleSpinBox *gamma32Box;
    QLCDNumber *ModelResolutionDisp;
    QLabel *modelResolution;
    QLabel *nonMaxSupp_2;
    QLabel *salientRadius_3;
    QLabel *normalRadius_2;
    QLabel *borderRadius_2;
    QDoubleSpinBox *nonMaxSuppBox_2;
    QDoubleSpinBox *salientRadiusBox_2;
    QDoubleSpinBox *normalRadiusBox_2;
    QDoubleSpinBox *borderRadiusBox_2;
    QFrame *line;
    QFrame *line_2;
    QPushButton *CalculateKeypoints;
    QLabel *CubeEdgeLength;
    QLabel *CubeEdgePoints;
    QDoubleSpinBox *CubeEdgeLengthBox;
    QDoubleSpinBox *CubeEdgePointsBox;
    QPushButton *defaultValues;
    QCheckBox *MultiplyResolution;
    QLabel *minNeighbors;
    QDoubleSpinBox *minNeighborsBox;
    QLineEdit *inputModelBox;
    QCheckBox *UsePcdInputBox;
    QCheckBox *DownSampleBox;
    QDoubleSpinBox *downSampleLeafSizeBox;

    void setupUi(QMainWindow *PCLViewer)
    {
        if (PCLViewer->objectName().isEmpty())
            PCLViewer->setObjectName(QString::fromUtf8("PCLViewer"));
        PCLViewer->resize(966, 806);
        PCLViewer->setMinimumSize(QSize(0, 0));
        PCLViewer->setMaximumSize(QSize(5000, 5000));
        PCLViewer->setDocumentMode(false);
        centralwidget = new QWidget(PCLViewer);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        qvtkWidget = new QVTKWidget(centralwidget);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));
        qvtkWidget->setGeometry(QRect(300, 10, 640, 711));
        nonMaxSuppBox = new QDoubleSpinBox(centralwidget);
        nonMaxSuppBox->setObjectName(QString::fromUtf8("nonMaxSuppBox"));
        nonMaxSuppBox->setGeometry(QRect(50, 100, 71, 27));
        QFont font;
        font.setFamily(QString::fromUtf8("Ubuntu Condensed"));
        font.setPointSize(9);
        nonMaxSuppBox->setFont(font);
        nonMaxSuppBox->setDecimals(4);
        nonMaxSuppBox->setValue(4);
        nonMaxSupp = new QLabel(centralwidget);
        nonMaxSupp->setObjectName(QString::fromUtf8("nonMaxSupp"));
        nonMaxSupp->setGeometry(QRect(20, 70, 121, 31));
        nonMaxSupp->setFont(font);
        salientRadius = new QLabel(centralwidget);
        salientRadius->setObjectName(QString::fromUtf8("salientRadius"));
        salientRadius->setGeometry(QRect(140, 70, 101, 31));
        salientRadius->setFont(font);
        salientRadiusBox = new QDoubleSpinBox(centralwidget);
        salientRadiusBox->setObjectName(QString::fromUtf8("salientRadiusBox"));
        salientRadiusBox->setGeometry(QRect(160, 100, 71, 27));
        salientRadiusBox->setFont(font);
        salientRadiusBox->setDecimals(4);
        salientRadiusBox->setValue(6);
        normalRadius = new QLabel(centralwidget);
        normalRadius->setObjectName(QString::fromUtf8("normalRadius"));
        normalRadius->setGeometry(QRect(20, 120, 101, 31));
        normalRadius->setFont(font);
        borderRadius = new QLabel(centralwidget);
        borderRadius->setObjectName(QString::fromUtf8("borderRadius"));
        borderRadius->setGeometry(QRect(140, 120, 101, 31));
        borderRadius->setFont(font);
        gamma21 = new QLabel(centralwidget);
        gamma21->setObjectName(QString::fromUtf8("gamma21"));
        gamma21->setGeometry(QRect(20, 190, 101, 31));
        gamma21->setFont(font);
        gamma32 = new QLabel(centralwidget);
        gamma32->setObjectName(QString::fromUtf8("gamma32"));
        gamma32->setGeometry(QRect(140, 190, 101, 31));
        gamma32->setFont(font);
        normalRadiusBox = new QDoubleSpinBox(centralwidget);
        normalRadiusBox->setObjectName(QString::fromUtf8("normalRadiusBox"));
        normalRadiusBox->setGeometry(QRect(50, 150, 71, 27));
        normalRadiusBox->setFont(font);
        normalRadiusBox->setDecimals(4);
        normalRadiusBox->setValue(4);
        borderRadiusBox = new QDoubleSpinBox(centralwidget);
        borderRadiusBox->setObjectName(QString::fromUtf8("borderRadiusBox"));
        borderRadiusBox->setGeometry(QRect(160, 150, 71, 27));
        borderRadiusBox->setFont(font);
        borderRadiusBox->setDecimals(4);
        borderRadiusBox->setValue(1);
        gamma21Box = new QDoubleSpinBox(centralwidget);
        gamma21Box->setObjectName(QString::fromUtf8("gamma21Box"));
        gamma21Box->setGeometry(QRect(50, 220, 62, 27));
        gamma21Box->setFont(font);
        gamma21Box->setDecimals(3);
        gamma21Box->setMaximum(1);
        gamma21Box->setSingleStep(0.1);
        gamma21Box->setValue(0.975);
        gamma32Box = new QDoubleSpinBox(centralwidget);
        gamma32Box->setObjectName(QString::fromUtf8("gamma32Box"));
        gamma32Box->setGeometry(QRect(160, 220, 62, 27));
        gamma32Box->setFont(font);
        gamma32Box->setDecimals(3);
        gamma32Box->setMaximum(1);
        gamma32Box->setSingleStep(0.1);
        gamma32Box->setValue(0.975);
        ModelResolutionDisp = new QLCDNumber(centralwidget);
        ModelResolutionDisp->setObjectName(QString::fromUtf8("ModelResolutionDisp"));
        ModelResolutionDisp->setGeometry(QRect(90, 330, 101, 21));
        QFont font1;
        font1.setFamily(QString::fromUtf8("Ubuntu Condensed"));
        font1.setPointSize(9);
        font1.setBold(false);
        font1.setWeight(50);
        ModelResolutionDisp->setFont(font1);
        ModelResolutionDisp->setSmallDecimalPoint(true);
        ModelResolutionDisp->setDigitCount(7);
        ModelResolutionDisp->setSegmentStyle(QLCDNumber::Flat);
        ModelResolutionDisp->setProperty("value", QVariant(0));
        modelResolution = new QLabel(centralwidget);
        modelResolution->setObjectName(QString::fromUtf8("modelResolution"));
        modelResolution->setGeometry(QRect(90, 300, 141, 31));
        modelResolution->setFont(font);
        nonMaxSupp_2 = new QLabel(centralwidget);
        nonMaxSupp_2->setObjectName(QString::fromUtf8("nonMaxSupp_2"));
        nonMaxSupp_2->setGeometry(QRect(20, 490, 101, 31));
        nonMaxSupp_2->setFont(font);
        salientRadius_3 = new QLabel(centralwidget);
        salientRadius_3->setObjectName(QString::fromUtf8("salientRadius_3"));
        salientRadius_3->setGeometry(QRect(140, 490, 101, 31));
        salientRadius_3->setFont(font);
        normalRadius_2 = new QLabel(centralwidget);
        normalRadius_2->setObjectName(QString::fromUtf8("normalRadius_2"));
        normalRadius_2->setGeometry(QRect(20, 550, 101, 31));
        normalRadius_2->setFont(font);
        borderRadius_2 = new QLabel(centralwidget);
        borderRadius_2->setObjectName(QString::fromUtf8("borderRadius_2"));
        borderRadius_2->setGeometry(QRect(140, 550, 101, 31));
        borderRadius_2->setFont(font);
        nonMaxSuppBox_2 = new QDoubleSpinBox(centralwidget);
        nonMaxSuppBox_2->setObjectName(QString::fromUtf8("nonMaxSuppBox_2"));
        nonMaxSuppBox_2->setGeometry(QRect(50, 520, 71, 27));
        nonMaxSuppBox_2->setFont(font);
        nonMaxSuppBox_2->setDecimals(4);
        salientRadiusBox_2 = new QDoubleSpinBox(centralwidget);
        salientRadiusBox_2->setObjectName(QString::fromUtf8("salientRadiusBox_2"));
        salientRadiusBox_2->setGeometry(QRect(160, 520, 71, 27));
        salientRadiusBox_2->setFont(font);
        salientRadiusBox_2->setDecimals(4);
        normalRadiusBox_2 = new QDoubleSpinBox(centralwidget);
        normalRadiusBox_2->setObjectName(QString::fromUtf8("normalRadiusBox_2"));
        normalRadiusBox_2->setGeometry(QRect(50, 590, 71, 27));
        normalRadiusBox_2->setFont(font);
        normalRadiusBox_2->setDecimals(4);
        borderRadiusBox_2 = new QDoubleSpinBox(centralwidget);
        borderRadiusBox_2->setObjectName(QString::fromUtf8("borderRadiusBox_2"));
        borderRadiusBox_2->setGeometry(QRect(160, 590, 71, 27));
        borderRadiusBox_2->setFont(font);
        borderRadiusBox_2->setDecimals(4);
        line = new QFrame(centralwidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(0, 180, 291, 16));
        line->setFrameShadow(QFrame::Raised);
        line->setLineWidth(2);
        line->setFrameShape(QFrame::HLine);
        line_2 = new QFrame(centralwidget);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setGeometry(QRect(0, 480, 291, 16));
        line_2->setFrameShadow(QFrame::Raised);
        line_2->setLineWidth(2);
        line_2->setFrameShape(QFrame::HLine);
        CalculateKeypoints = new QPushButton(centralwidget);
        CalculateKeypoints->setObjectName(QString::fromUtf8("CalculateKeypoints"));
        CalculateKeypoints->setGeometry(QRect(10, 640, 111, 61));
        CalculateKeypoints->setFont(font);
        CubeEdgeLength = new QLabel(centralwidget);
        CubeEdgeLength->setObjectName(QString::fromUtf8("CubeEdgeLength"));
        CubeEdgeLength->setGeometry(QRect(20, 240, 121, 31));
        CubeEdgeLength->setFont(font);
        CubeEdgePoints = new QLabel(centralwidget);
        CubeEdgePoints->setObjectName(QString::fromUtf8("CubeEdgePoints"));
        CubeEdgePoints->setGeometry(QRect(140, 240, 121, 31));
        CubeEdgePoints->setFont(font);
        CubeEdgeLengthBox = new QDoubleSpinBox(centralwidget);
        CubeEdgeLengthBox->setObjectName(QString::fromUtf8("CubeEdgeLengthBox"));
        CubeEdgeLengthBox->setGeometry(QRect(50, 270, 62, 27));
        CubeEdgeLengthBox->setFont(font);
        CubeEdgeLengthBox->setDecimals(3);
        CubeEdgeLengthBox->setMaximum(100);
        CubeEdgeLengthBox->setSingleStep(0.1);
        CubeEdgeLengthBox->setValue(0.4);
        CubeEdgePointsBox = new QDoubleSpinBox(centralwidget);
        CubeEdgePointsBox->setObjectName(QString::fromUtf8("CubeEdgePointsBox"));
        CubeEdgePointsBox->setGeometry(QRect(160, 270, 71, 27));
        CubeEdgePointsBox->setFont(font);
        CubeEdgePointsBox->setDecimals(0);
        CubeEdgePointsBox->setMaximum(1000);
        CubeEdgePointsBox->setSingleStep(0.1);
        CubeEdgePointsBox->setValue(10);
        defaultValues = new QPushButton(centralwidget);
        defaultValues->setObjectName(QString::fromUtf8("defaultValues"));
        defaultValues->setGeometry(QRect(160, 640, 111, 61));
        defaultValues->setFont(font);
        MultiplyResolution = new QCheckBox(centralwidget);
        MultiplyResolution->setObjectName(QString::fromUtf8("MultiplyResolution"));
        MultiplyResolution->setGeometry(QRect(60, 400, 161, 31));
        MultiplyResolution->setFont(font);
        MultiplyResolution->setChecked(true);
        minNeighbors = new QLabel(centralwidget);
        minNeighbors->setObjectName(QString::fromUtf8("minNeighbors"));
        minNeighbors->setGeometry(QRect(20, 350, 121, 51));
        minNeighbors->setFont(font);
        minNeighborsBox = new QDoubleSpinBox(centralwidget);
        minNeighborsBox->setObjectName(QString::fromUtf8("minNeighborsBox"));
        minNeighborsBox->setGeometry(QRect(160, 360, 71, 31));
        minNeighborsBox->setFont(font);
        minNeighborsBox->setDecimals(0);
        minNeighborsBox->setMaximum(10000);
        minNeighborsBox->setSingleStep(1);
        minNeighborsBox->setValue(30);
        inputModelBox = new QLineEdit(centralwidget);
        inputModelBox->setObjectName(QString::fromUtf8("inputModelBox"));
        inputModelBox->setGeometry(QRect(80, 30, 113, 27));
        inputModelBox->setFont(font);
        UsePcdInputBox = new QCheckBox(centralwidget);
        UsePcdInputBox->setObjectName(QString::fromUtf8("UsePcdInputBox"));
        UsePcdInputBox->setGeometry(QRect(60, 430, 121, 22));
        UsePcdInputBox->setFont(font);
        DownSampleBox = new QCheckBox(centralwidget);
        DownSampleBox->setObjectName(QString::fromUtf8("DownSampleBox"));
        DownSampleBox->setGeometry(QRect(60, 460, 97, 22));
        DownSampleBox->setFont(font);
        downSampleLeafSizeBox = new QDoubleSpinBox(centralwidget);
        downSampleLeafSizeBox->setObjectName(QString::fromUtf8("downSampleLeafSizeBox"));
        downSampleLeafSizeBox->setGeometry(QRect(160, 460, 71, 27));
        downSampleLeafSizeBox->setFont(font);
        downSampleLeafSizeBox->setDecimals(5);
        downSampleLeafSizeBox->setSingleStep(0.001);
        downSampleLeafSizeBox->setValue(0.01);
        PCLViewer->setCentralWidget(centralwidget);
#ifndef QT_NO_SHORTCUT
        nonMaxSupp->setBuddy(nonMaxSuppBox);
        salientRadius->setBuddy(salientRadiusBox);
        normalRadius->setBuddy(normalRadiusBox);
        borderRadius->setBuddy(borderRadiusBox);
        gamma21->setBuddy(gamma21Box);
        gamma32->setBuddy(gamma32Box);
        nonMaxSupp_2->setBuddy(nonMaxSuppBox_2);
        salientRadius_3->setBuddy(salientRadiusBox_2);
        normalRadius_2->setBuddy(normalRadiusBox_2);
        borderRadius_2->setBuddy(borderRadiusBox_2);
        CubeEdgeLength->setBuddy(gamma21Box);
        CubeEdgePoints->setBuddy(gamma21Box);
        minNeighbors->setBuddy(gamma21Box);
#endif // QT_NO_SHORTCUT
        QWidget::setTabOrder(nonMaxSuppBox, salientRadiusBox);
        QWidget::setTabOrder(salientRadiusBox, normalRadiusBox);
        QWidget::setTabOrder(normalRadiusBox, borderRadiusBox);
        QWidget::setTabOrder(borderRadiusBox, gamma21Box);
        QWidget::setTabOrder(gamma21Box, gamma32Box);
        QWidget::setTabOrder(gamma32Box, nonMaxSuppBox_2);
        QWidget::setTabOrder(nonMaxSuppBox_2, salientRadiusBox_2);
        QWidget::setTabOrder(salientRadiusBox_2, normalRadiusBox_2);
        QWidget::setTabOrder(normalRadiusBox_2, borderRadiusBox_2);

        retranslateUi(PCLViewer);

        QMetaObject::connectSlotsByName(PCLViewer);
    } // setupUi

    void retranslateUi(QMainWindow *PCLViewer)
    {
        PCLViewer->setWindowTitle(QApplication::translate("PCLViewer", "PCLViewer", 0, QApplication::UnicodeUTF8));
        nonMaxSupp->setText(QApplication::translate("PCLViewer", "nonMaxSuppMult", 0, QApplication::UnicodeUTF8));
        salientRadius->setText(QApplication::translate("PCLViewer", "salRadMult", 0, QApplication::UnicodeUTF8));
        normalRadius->setText(QApplication::translate("PCLViewer", "normRadMult", 0, QApplication::UnicodeUTF8));
        borderRadius->setText(QApplication::translate("PCLViewer", "bordRadMult", 0, QApplication::UnicodeUTF8));
        gamma21->setText(QApplication::translate("PCLViewer", "gamma21", 0, QApplication::UnicodeUTF8));
        gamma32->setText(QApplication::translate("PCLViewer", "gamma32", 0, QApplication::UnicodeUTF8));
        modelResolution->setText(QApplication::translate("PCLViewer", "ModelResolution", 0, QApplication::UnicodeUTF8));
        nonMaxSupp_2->setText(QApplication::translate("PCLViewer", "nonMaxSupp", 0, QApplication::UnicodeUTF8));
        salientRadius_3->setText(QApplication::translate("PCLViewer", "salientRadius", 0, QApplication::UnicodeUTF8));
        normalRadius_2->setText(QApplication::translate("PCLViewer", "normalRadius", 0, QApplication::UnicodeUTF8));
        borderRadius_2->setText(QApplication::translate("PCLViewer", "borderRadius", 0, QApplication::UnicodeUTF8));
        CalculateKeypoints->setText(QApplication::translate("PCLViewer", "Calculate \n"
"Keypoints", 0, QApplication::UnicodeUTF8));
        CubeEdgeLength->setText(QApplication::translate("PCLViewer", "CubeEdgeLength", 0, QApplication::UnicodeUTF8));
        CubeEdgePoints->setText(QApplication::translate("PCLViewer", "CubeEdgePoints", 0, QApplication::UnicodeUTF8));
        defaultValues->setText(QApplication::translate("PCLViewer", "Default\n"
"Values", 0, QApplication::UnicodeUTF8));
        MultiplyResolution->setText(QApplication::translate("PCLViewer", "MultiplyResolution", 0, QApplication::UnicodeUTF8));
        minNeighbors->setText(QApplication::translate("PCLViewer", "<html><head/><body><p>MinNeighborsfor</p><p>NonMaxSupp</p></body></html>", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        inputModelBox->setToolTip(QApplication::translate("PCLViewer", "Give a valid .pcd file to load the model from", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        inputModelBox->setText(QString());
        inputModelBox->setPlaceholderText(QApplication::translate("PCLViewer", "PCD Input File", 0, QApplication::UnicodeUTF8));
        UsePcdInputBox->setText(QApplication::translate("PCLViewer", "Use PCD Input File", 0, QApplication::UnicodeUTF8));
        DownSampleBox->setText(QApplication::translate("PCLViewer", "Downsample", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class PCLViewer: public Ui_PCLViewer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PCLVIEWER_H
