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
#include <QtGui/QComboBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QFrame>
#include <QtGui/QHeaderView>
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
    QPushButton *CalculateKeypoints;
    QLabel *CubeEdgeLength;
    QLabel *CubeEdgePoints;
    QDoubleSpinBox *CubeEdgePointsBox;
    QPushButton *defaultValues;
    QLineEdit *inputModelBox;
    QCheckBox *UsePcdInputBox;
    QCheckBox *DownSampleBox;
    QDoubleSpinBox *downSampleLeafSizeBox;
    QDoubleSpinBox *CubeEdgeLengthBox;
    QCheckBox *refineKeypointsBox;
    QFrame *line;
    QDoubleSpinBox *nonMaxNormalRadiusBox;
    QDoubleSpinBox *thresholdBox;
    QLabel *nonMaxNormalRadius;
    QLabel *Threshold;
    QComboBox *responseMethodBox;
    QLabel *responseMethod;
    QCheckBox *useMLSBox;
    QDoubleSpinBox *mlsSearchRadiusBox;
    QLabel *label;
    QLabel *label_2;

    void setupUi(QMainWindow *PCLViewer)
    {
        if (PCLViewer->objectName().isEmpty())
            PCLViewer->setObjectName(QString::fromUtf8("PCLViewer"));
        PCLViewer->resize(967, 806);
        PCLViewer->setMinimumSize(QSize(0, 0));
        PCLViewer->setMaximumSize(QSize(5000, 5000));
        PCLViewer->setDocumentMode(false);
        centralwidget = new QWidget(PCLViewer);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        qvtkWidget = new QVTKWidget(centralwidget);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));
        qvtkWidget->setGeometry(QRect(300, 10, 640, 711));
        CalculateKeypoints = new QPushButton(centralwidget);
        CalculateKeypoints->setObjectName(QString::fromUtf8("CalculateKeypoints"));
        CalculateKeypoints->setGeometry(QRect(10, 640, 111, 61));
        QFont font;
        font.setFamily(QString::fromUtf8("Ubuntu Condensed"));
        font.setPointSize(9);
        CalculateKeypoints->setFont(font);
        CubeEdgeLength = new QLabel(centralwidget);
        CubeEdgeLength->setObjectName(QString::fromUtf8("CubeEdgeLength"));
        CubeEdgeLength->setGeometry(QRect(20, 70, 121, 31));
        CubeEdgeLength->setFont(font);
        CubeEdgePoints = new QLabel(centralwidget);
        CubeEdgePoints->setObjectName(QString::fromUtf8("CubeEdgePoints"));
        CubeEdgePoints->setGeometry(QRect(160, 70, 121, 31));
        CubeEdgePoints->setFont(font);
        CubeEdgePointsBox = new QDoubleSpinBox(centralwidget);
        CubeEdgePointsBox->setObjectName(QString::fromUtf8("CubeEdgePointsBox"));
        CubeEdgePointsBox->setGeometry(QRect(170, 100, 71, 27));
        CubeEdgePointsBox->setFont(font);
        CubeEdgePointsBox->setDecimals(0);
        CubeEdgePointsBox->setMaximum(1000);
        CubeEdgePointsBox->setSingleStep(0.1);
        CubeEdgePointsBox->setValue(10);
        defaultValues = new QPushButton(centralwidget);
        defaultValues->setObjectName(QString::fromUtf8("defaultValues"));
        defaultValues->setGeometry(QRect(160, 640, 111, 61));
        defaultValues->setFont(font);
        inputModelBox = new QLineEdit(centralwidget);
        inputModelBox->setObjectName(QString::fromUtf8("inputModelBox"));
        inputModelBox->setGeometry(QRect(80, 30, 113, 27));
        inputModelBox->setFont(font);
        UsePcdInputBox = new QCheckBox(centralwidget);
        UsePcdInputBox->setObjectName(QString::fromUtf8("UsePcdInputBox"));
        UsePcdInputBox->setGeometry(QRect(40, 10, 121, 22));
        UsePcdInputBox->setFont(font);
        DownSampleBox = new QCheckBox(centralwidget);
        DownSampleBox->setObjectName(QString::fromUtf8("DownSampleBox"));
        DownSampleBox->setGeometry(QRect(40, 140, 97, 22));
        DownSampleBox->setFont(font);
        downSampleLeafSizeBox = new QDoubleSpinBox(centralwidget);
        downSampleLeafSizeBox->setObjectName(QString::fromUtf8("downSampleLeafSizeBox"));
        downSampleLeafSizeBox->setGeometry(QRect(190, 160, 71, 27));
        downSampleLeafSizeBox->setFont(font);
        downSampleLeafSizeBox->setDecimals(5);
        downSampleLeafSizeBox->setSingleStep(0.001);
        downSampleLeafSizeBox->setValue(0.01);
        CubeEdgeLengthBox = new QDoubleSpinBox(centralwidget);
        CubeEdgeLengthBox->setObjectName(QString::fromUtf8("CubeEdgeLengthBox"));
        CubeEdgeLengthBox->setGeometry(QRect(40, 100, 62, 27));
        CubeEdgeLengthBox->setFont(font);
        CubeEdgeLengthBox->setDecimals(3);
        CubeEdgeLengthBox->setMaximum(100);
        CubeEdgeLengthBox->setSingleStep(0.1);
        CubeEdgeLengthBox->setValue(0.4);
        refineKeypointsBox = new QCheckBox(centralwidget);
        refineKeypointsBox->setObjectName(QString::fromUtf8("refineKeypointsBox"));
        refineKeypointsBox->setGeometry(QRect(40, 180, 141, 22));
        refineKeypointsBox->setFont(font);
        line = new QFrame(centralwidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(0, 260, 291, 16));
        line->setFrameShadow(QFrame::Raised);
        line->setLineWidth(2);
        line->setFrameShape(QFrame::HLine);
        nonMaxNormalRadiusBox = new QDoubleSpinBox(centralwidget);
        nonMaxNormalRadiusBox->setObjectName(QString::fromUtf8("nonMaxNormalRadiusBox"));
        nonMaxNormalRadiusBox->setGeometry(QRect(180, 290, 62, 27));
        nonMaxNormalRadiusBox->setFont(font);
        nonMaxNormalRadiusBox->setDecimals(5);
        nonMaxNormalRadiusBox->setValue(0.01);
        thresholdBox = new QDoubleSpinBox(centralwidget);
        thresholdBox->setObjectName(QString::fromUtf8("thresholdBox"));
        thresholdBox->setGeometry(QRect(180, 330, 62, 27));
        thresholdBox->setFont(font);
        thresholdBox->setDecimals(5);
        nonMaxNormalRadius = new QLabel(centralwidget);
        nonMaxNormalRadius->setObjectName(QString::fromUtf8("nonMaxNormalRadius"));
        nonMaxNormalRadius->setGeometry(QRect(50, 300, 111, 17));
        nonMaxNormalRadius->setFont(font);
        Threshold = new QLabel(centralwidget);
        Threshold->setObjectName(QString::fromUtf8("Threshold"));
        Threshold->setGeometry(QRect(50, 340, 66, 17));
        Threshold->setFont(font);
        responseMethodBox = new QComboBox(centralwidget);
        responseMethodBox->setObjectName(QString::fromUtf8("responseMethodBox"));
        responseMethodBox->setGeometry(QRect(180, 370, 78, 27));
        responseMethodBox->setFont(font);
        responseMethod = new QLabel(centralwidget);
        responseMethod->setObjectName(QString::fromUtf8("responseMethod"));
        responseMethod->setGeometry(QRect(50, 380, 81, 17));
        responseMethod->setFont(font);
        useMLSBox = new QCheckBox(centralwidget);
        useMLSBox->setObjectName(QString::fromUtf8("useMLSBox"));
        useMLSBox->setGeometry(QRect(40, 220, 141, 22));
        useMLSBox->setFont(font);
        mlsSearchRadiusBox = new QDoubleSpinBox(centralwidget);
        mlsSearchRadiusBox->setObjectName(QString::fromUtf8("mlsSearchRadiusBox"));
        mlsSearchRadiusBox->setGeometry(QRect(200, 220, 62, 27));
        mlsSearchRadiusBox->setFont(font);
        mlsSearchRadiusBox->setDecimals(4);
        mlsSearchRadiusBox->setValue(0.03);
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(190, 140, 101, 21));
        label->setFont(font);
        label_2 = new QLabel(centralwidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(190, 200, 91, 20));
        label_2->setFont(font);
        PCLViewer->setCentralWidget(centralwidget);

        retranslateUi(PCLViewer);

        QMetaObject::connectSlotsByName(PCLViewer);
    } // setupUi

    void retranslateUi(QMainWindow *PCLViewer)
    {
        PCLViewer->setWindowTitle(QApplication::translate("PCLViewer", "PCLViewer", 0, QApplication::UnicodeUTF8));
        CalculateKeypoints->setText(QApplication::translate("PCLViewer", "Calculate \n"
"Keypoints", 0, QApplication::UnicodeUTF8));
        CubeEdgeLength->setText(QApplication::translate("PCLViewer", "CubeEdgeLength", 0, QApplication::UnicodeUTF8));
        CubeEdgePoints->setText(QApplication::translate("PCLViewer", "CubeEdgePoints", 0, QApplication::UnicodeUTF8));
        defaultValues->setText(QApplication::translate("PCLViewer", "Default\n"
"Values", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        inputModelBox->setToolTip(QApplication::translate("PCLViewer", "Give a valid .pcd file to load the model from", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        inputModelBox->setText(QString());
        inputModelBox->setPlaceholderText(QApplication::translate("PCLViewer", "PCD Input File", 0, QApplication::UnicodeUTF8));
        UsePcdInputBox->setText(QApplication::translate("PCLViewer", "Use PCD Input File", 0, QApplication::UnicodeUTF8));
        DownSampleBox->setText(QApplication::translate("PCLViewer", "Downsample", 0, QApplication::UnicodeUTF8));
        refineKeypointsBox->setText(QApplication::translate("PCLViewer", "Refine Keypoints", 0, QApplication::UnicodeUTF8));
        nonMaxNormalRadius->setText(QApplication::translate("PCLViewer", "nonMax/Normal Radius", 0, QApplication::UnicodeUTF8));
        Threshold->setText(QApplication::translate("PCLViewer", "Threshold", 0, QApplication::UnicodeUTF8));
        responseMethodBox->clear();
        responseMethodBox->insertItems(0, QStringList()
         << QApplication::translate("PCLViewer", "HARRIS", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("PCLViewer", "NOBLE", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("PCLViewer", "LOWE", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("PCLViewer", "TOMASI", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("PCLViewer", "CURVATURE", 0, QApplication::UnicodeUTF8)
        );
        responseMethod->setText(QApplication::translate("PCLViewer", "Response Method", 0, QApplication::UnicodeUTF8));
        useMLSBox->setText(QApplication::translate("PCLViewer", "Use Moving Least Squares", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("PCLViewer", "downsample leafsize", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("PCLViewer", "MLS SearchRadius", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class PCLViewer: public Ui_PCLViewer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PCLVIEWER_H
