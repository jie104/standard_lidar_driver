/********************************************************************************
** Form generated from reading UI file 'playbag.ui'
**
** Created by: Qt User Interface Compiler version 5.12.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PLAYBAG_H
#define UI_PLAYBAG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QTextBrowser *SensorBrowser;
    QSlider *horizontalSlider;
    QTabWidget *tabWidget;
    QWidget *tab1;
    QTextBrowser *textBrowser;
    QWidget *tab2;
    QTextBrowser *textBrowser_3;
    QCheckBox *lidarBox;
    QCheckBox *camera2;
    QCheckBox *imu3;
    QCheckBox *odom4;
    QCheckBox *pose5;
    QCheckBox *checkBox;
    QLabel *label;
    QWidget *widget;
    QVBoxLayout *verticalLayout;
    QPushButton *pushButton;
    QPushButton *PlayButton;
    QPushButton *StepButton;
    QPushButton *StopButton;
    QPushButton *SpeedIncreaseButton;
    QPushButton *SpeedSlowButton;
    QPushButton *ExitButton;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
            MainWindow->setFixedSize(1200,719);
       // MainWindow->resize(1200, 719);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        SensorBrowser = new QTextBrowser(centralWidget);
        SensorBrowser->setObjectName(QString::fromUtf8("SensorBrowser"));
        SensorBrowser->setGeometry(QRect(610, 30, 381, 31));
        horizontalSlider = new QSlider(centralWidget);
        horizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
        horizontalSlider->setGeometry(QRect(70, 540, 361, 16));
        horizontalSlider->setOrientation(Qt::Horizontal);
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(610, 120, 401, 471));
        tabWidget->setStyleSheet(QString::fromUtf8("QTabBar::tab{width:198}"));
        tab1 = new QWidget();
        tab1->setObjectName(QString::fromUtf8("tab1"));
        textBrowser = new QTextBrowser(tab1);
        textBrowser->setObjectName(QString::fromUtf8("textBrowser"));
        textBrowser->setGeometry(QRect(0, 0, 401, 441));
        tabWidget->addTab(tab1, QString());
        tab2 = new QWidget();
        tab2->setObjectName(QString::fromUtf8("tab2"));
        textBrowser_3 = new QTextBrowser(tab2);
        textBrowser_3->setObjectName(QString::fromUtf8("textBrowser_3"));
        textBrowser_3->setGeometry(QRect(-5, 1, 401, 441));
        tabWidget->addTab(tab2, QString());
        lidarBox = new QCheckBox(centralWidget);
        lidarBox->setObjectName(QString::fromUtf8("lidarBox"));
        lidarBox->setGeometry(QRect(580, 80, 92, 23));
        camera2 = new QCheckBox(centralWidget);
        camera2->setObjectName(QString::fromUtf8("camera2"));
        camera2->setGeometry(QRect(640, 80, 92, 23));
        camera2->setStyleSheet(QString::fromUtf8("<input type=\"camera2\"  onclick=\"return false;\" />\n"
"\n"
""));
        imu3 = new QCheckBox(centralWidget);
        imu3->setObjectName(QString::fromUtf8("imu3"));
        imu3->setGeometry(QRect(730, 80, 92, 23));
        odom4 = new QCheckBox(centralWidget);
        odom4->setObjectName(QString::fromUtf8("odom4"));
        odom4->setGeometry(QRect(790, 80, 92, 23));
        pose5 = new QCheckBox(centralWidget);
        pose5->setObjectName(QString::fromUtf8("pose5"));
        pose5->setGeometry(QRect(870, 80, 92, 23));
        checkBox = new QCheckBox(centralWidget);
        checkBox->setObjectName(QString::fromUtf8("checkBox"));
        checkBox->setGeometry(QRect(950, 80, 92, 23));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(440, 530, 67, 31));
        widget = new QWidget(centralWidget);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setGeometry(QRect(110, 130, 291, 381));
        verticalLayout = new QVBoxLayout(widget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        pushButton = new QPushButton(widget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        verticalLayout->addWidget(pushButton);

        PlayButton = new QPushButton(widget);
        PlayButton->setObjectName(QString::fromUtf8("PlayButton"));

        verticalLayout->addWidget(PlayButton);

        StepButton = new QPushButton(widget);
        StepButton->setObjectName(QString::fromUtf8("StepButton"));

        verticalLayout->addWidget(StepButton);

        StopButton = new QPushButton(widget);
        StopButton->setObjectName(QString::fromUtf8("StopButton"));

        verticalLayout->addWidget(StopButton);

        SpeedIncreaseButton = new QPushButton(widget);
        SpeedIncreaseButton->setObjectName(QString::fromUtf8("SpeedIncreaseButton"));

        verticalLayout->addWidget(SpeedIncreaseButton);

        SpeedSlowButton = new QPushButton(widget);
        SpeedSlowButton->setObjectName(QString::fromUtf8("SpeedSlowButton"));

        verticalLayout->addWidget(SpeedSlowButton);

        ExitButton = new QPushButton(widget);
        ExitButton->setObjectName(QString::fromUtf8("ExitButton"));

        verticalLayout->addWidget(ExitButton);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1200, 28));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);
        PlayButton->setDefault(false);
        SpeedSlowButton->setDefault(false);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab1), QApplication::translate("MainWindow", "Tab 1", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab2), QApplication::translate("MainWindow", "Tab 2", nullptr));
        lidarBox->setText(QApplication::translate("MainWindow", "lidar", nullptr));
        camera2->setText(QApplication::translate("MainWindow", "camera", nullptr));
        imu3->setText(QApplication::translate("MainWindow", "imu", nullptr));
        odom4->setText(QApplication::translate("MainWindow", "odom", nullptr));
        pose5->setText(QApplication::translate("MainWindow", "pose", nullptr));
        checkBox->setText(QApplication::translate("MainWindow", "elided", nullptr));
       // label->setText(QApplication::translate("MainWindow", "33", nullptr));
        pushButton->setText(QApplication::translate("MainWindow", "ChooseBag", nullptr));
        PlayButton->setText(QApplication::translate("MainWindow", "Play", nullptr));
        StepButton->setText(QApplication::translate("MainWindow", "Step", nullptr));
        StopButton->setText(QApplication::translate("MainWindow", "Stop/Continue", nullptr));
        SpeedIncreaseButton->setText(QApplication::translate("MainWindow", "SpeedIncrease", nullptr));
        SpeedSlowButton->setText(QApplication::translate("MainWindow", "SpeedSlow", nullptr));
        ExitButton->setText(QApplication::translate("MainWindow", "Exit", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PLAYBAG_H
