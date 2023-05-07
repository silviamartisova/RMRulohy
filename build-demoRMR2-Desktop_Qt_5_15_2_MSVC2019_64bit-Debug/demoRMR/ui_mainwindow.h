/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTableView>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout;
    QPushButton *pushButton_9;
    QLineEdit *lineEdit;
    QPushButton *pushButton;
    QGridLayout *gridLayout_2;
    QPushButton *pushButton_4;
    QPushButton *pushButton_3;
    QPushButton *pushButton_5;
    QPushButton *pushButton_6;
    QPushButton *pushButton_2;
    QPushButton *pushButton_7;
    QPushButton *pushButton_10;
    QLineEdit *lineEdit_5;
    QLineEdit *lineEdit_6;
    QVBoxLayout *verticalLayout_2;
    QPushButton *pushButton_8;
    QPushButton *btnRegulation;
    QTableView *tbPoints;
    QGridLayout *gridLayout_4;
    QLabel *label_6;
    QLabel *label_8;
    QLineEdit *leXpoint;
    QLineEdit *leYpoint;
    QPushButton *btnAddPoint;
    QGridLayout *gridLayout_3;
    QLabel *label_2;
    QLineEdit *leTranslationSpeed;
    QLabel *label;
    QLineEdit *lineEdit_4;
    QLineEdit *lineEdit_3;
    QLabel *label_5;
    QLabel *label_3;
    QLineEdit *lineEdit_2;
    QLineEdit *leRotationSpeed;
    QLabel *label_7;
    QFrame *frame;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(889, 778);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        pushButton_9 = new QPushButton(centralWidget);
        pushButton_9->setObjectName(QString::fromUtf8("pushButton_9"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(pushButton_9->sizePolicy().hasHeightForWidth());
        pushButton_9->setSizePolicy(sizePolicy);
        pushButton_9->setMinimumSize(QSize(162, 0));

        verticalLayout->addWidget(pushButton_9);

        lineEdit = new QLineEdit(centralWidget);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));
        sizePolicy.setHeightForWidth(lineEdit->sizePolicy().hasHeightForWidth());
        lineEdit->setSizePolicy(sizePolicy);
        lineEdit->setMinimumSize(QSize(162, 0));

        verticalLayout->addWidget(lineEdit);

        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        sizePolicy.setHeightForWidth(pushButton->sizePolicy().hasHeightForWidth());
        pushButton->setSizePolicy(sizePolicy);
        pushButton->setMinimumSize(QSize(162, 0));

        verticalLayout->addWidget(pushButton);


        gridLayout->addLayout(verticalLayout, 1, 1, 1, 1);

        gridLayout_2 = new QGridLayout();
        gridLayout_2->setSpacing(6);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        pushButton_4 = new QPushButton(centralWidget);
        pushButton_4->setObjectName(QString::fromUtf8("pushButton_4"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(pushButton_4->sizePolicy().hasHeightForWidth());
        pushButton_4->setSizePolicy(sizePolicy1);

        gridLayout_2->addWidget(pushButton_4, 1, 1, 1, 1);

        pushButton_3 = new QPushButton(centralWidget);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));
        sizePolicy1.setHeightForWidth(pushButton_3->sizePolicy().hasHeightForWidth());
        pushButton_3->setSizePolicy(sizePolicy1);

        gridLayout_2->addWidget(pushButton_3, 2, 1, 1, 1);

        pushButton_5 = new QPushButton(centralWidget);
        pushButton_5->setObjectName(QString::fromUtf8("pushButton_5"));
        sizePolicy1.setHeightForWidth(pushButton_5->sizePolicy().hasHeightForWidth());
        pushButton_5->setSizePolicy(sizePolicy1);

        gridLayout_2->addWidget(pushButton_5, 1, 2, 1, 1);

        pushButton_6 = new QPushButton(centralWidget);
        pushButton_6->setObjectName(QString::fromUtf8("pushButton_6"));
        sizePolicy1.setHeightForWidth(pushButton_6->sizePolicy().hasHeightForWidth());
        pushButton_6->setSizePolicy(sizePolicy1);

        gridLayout_2->addWidget(pushButton_6, 1, 0, 1, 1);

        pushButton_2 = new QPushButton(centralWidget);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        sizePolicy1.setHeightForWidth(pushButton_2->sizePolicy().hasHeightForWidth());
        pushButton_2->setSizePolicy(sizePolicy1);

        gridLayout_2->addWidget(pushButton_2, 0, 1, 1, 1);

        pushButton_7 = new QPushButton(centralWidget);
        pushButton_7->setObjectName(QString::fromUtf8("pushButton_7"));

        gridLayout_2->addWidget(pushButton_7, 2, 2, 1, 1);

        pushButton_10 = new QPushButton(centralWidget);
        pushButton_10->setObjectName(QString::fromUtf8("pushButton_10"));

        gridLayout_2->addWidget(pushButton_10, 0, 2, 1, 1);

        lineEdit_5 = new QLineEdit(centralWidget);
        lineEdit_5->setObjectName(QString::fromUtf8("lineEdit_5"));
        sizePolicy.setHeightForWidth(lineEdit_5->sizePolicy().hasHeightForWidth());
        lineEdit_5->setSizePolicy(sizePolicy);
        lineEdit_5->setMinimumSize(QSize(162, 0));

        gridLayout_2->addWidget(lineEdit_5, 0, 0, 1, 1);

        lineEdit_6 = new QLineEdit(centralWidget);
        lineEdit_6->setObjectName(QString::fromUtf8("lineEdit_6"));
        sizePolicy.setHeightForWidth(lineEdit_6->sizePolicy().hasHeightForWidth());
        lineEdit_6->setSizePolicy(sizePolicy);
        lineEdit_6->setMinimumSize(QSize(162, 0));

        gridLayout_2->addWidget(lineEdit_6, 2, 0, 1, 1);


        gridLayout->addLayout(gridLayout_2, 1, 0, 1, 1);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        pushButton_8 = new QPushButton(centralWidget);
        pushButton_8->setObjectName(QString::fromUtf8("pushButton_8"));
        sizePolicy.setHeightForWidth(pushButton_8->sizePolicy().hasHeightForWidth());
        pushButton_8->setSizePolicy(sizePolicy);
        pushButton_8->setMinimumSize(QSize(162, 0));

        verticalLayout_2->addWidget(pushButton_8);

        btnRegulation = new QPushButton(centralWidget);
        btnRegulation->setObjectName(QString::fromUtf8("btnRegulation"));
        sizePolicy.setHeightForWidth(btnRegulation->sizePolicy().hasHeightForWidth());
        btnRegulation->setSizePolicy(sizePolicy);
        btnRegulation->setMinimumSize(QSize(162, 0));

        verticalLayout_2->addWidget(btnRegulation);

        tbPoints = new QTableView(centralWidget);
        tbPoints->setObjectName(QString::fromUtf8("tbPoints"));

        verticalLayout_2->addWidget(tbPoints);

        gridLayout_4 = new QGridLayout();
        gridLayout_4->setSpacing(6);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        label_6 = new QLabel(centralWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout_4->addWidget(label_6, 0, 0, 1, 1);

        label_8 = new QLabel(centralWidget);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        gridLayout_4->addWidget(label_8, 1, 0, 1, 1);

        leXpoint = new QLineEdit(centralWidget);
        leXpoint->setObjectName(QString::fromUtf8("leXpoint"));

        gridLayout_4->addWidget(leXpoint, 0, 1, 1, 1);

        leYpoint = new QLineEdit(centralWidget);
        leYpoint->setObjectName(QString::fromUtf8("leYpoint"));

        gridLayout_4->addWidget(leYpoint, 1, 1, 1, 1);


        verticalLayout_2->addLayout(gridLayout_4);

        btnAddPoint = new QPushButton(centralWidget);
        btnAddPoint->setObjectName(QString::fromUtf8("btnAddPoint"));

        verticalLayout_2->addWidget(btnAddPoint);

        gridLayout_3 = new QGridLayout();
        gridLayout_3->setSpacing(6);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout_3->addWidget(label_2, 4, 0, 1, 1);

        leTranslationSpeed = new QLineEdit(centralWidget);
        leTranslationSpeed->setObjectName(QString::fromUtf8("leTranslationSpeed"));
        sizePolicy.setHeightForWidth(leTranslationSpeed->sizePolicy().hasHeightForWidth());
        leTranslationSpeed->setSizePolicy(sizePolicy);
        leTranslationSpeed->setFocusPolicy(Qt::NoFocus);
        leTranslationSpeed->setReadOnly(true);

        gridLayout_3->addWidget(leTranslationSpeed, 1, 1, 1, 1);

        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout_3->addWidget(label, 3, 0, 1, 1);

        lineEdit_4 = new QLineEdit(centralWidget);
        lineEdit_4->setObjectName(QString::fromUtf8("lineEdit_4"));
        sizePolicy.setHeightForWidth(lineEdit_4->sizePolicy().hasHeightForWidth());
        lineEdit_4->setSizePolicy(sizePolicy);
        lineEdit_4->setFocusPolicy(Qt::NoFocus);
        lineEdit_4->setReadOnly(true);

        gridLayout_3->addWidget(lineEdit_4, 5, 1, 1, 1);

        lineEdit_3 = new QLineEdit(centralWidget);
        lineEdit_3->setObjectName(QString::fromUtf8("lineEdit_3"));
        sizePolicy.setHeightForWidth(lineEdit_3->sizePolicy().hasHeightForWidth());
        lineEdit_3->setSizePolicy(sizePolicy);
        lineEdit_3->setFocusPolicy(Qt::NoFocus);
        lineEdit_3->setReadOnly(true);

        gridLayout_3->addWidget(lineEdit_3, 4, 1, 1, 1);

        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout_3->addWidget(label_5, 1, 0, 1, 1);

        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout_3->addWidget(label_3, 5, 0, 1, 1);

        lineEdit_2 = new QLineEdit(centralWidget);
        lineEdit_2->setObjectName(QString::fromUtf8("lineEdit_2"));
        sizePolicy.setHeightForWidth(lineEdit_2->sizePolicy().hasHeightForWidth());
        lineEdit_2->setSizePolicy(sizePolicy);
        lineEdit_2->setFocusPolicy(Qt::NoFocus);
        lineEdit_2->setReadOnly(true);

        gridLayout_3->addWidget(lineEdit_2, 3, 1, 1, 1);

        leRotationSpeed = new QLineEdit(centralWidget);
        leRotationSpeed->setObjectName(QString::fromUtf8("leRotationSpeed"));
        sizePolicy.setHeightForWidth(leRotationSpeed->sizePolicy().hasHeightForWidth());
        leRotationSpeed->setSizePolicy(sizePolicy);
        leRotationSpeed->setFocusPolicy(Qt::NoFocus);
        leRotationSpeed->setReadOnly(true);

        gridLayout_3->addWidget(leRotationSpeed, 2, 1, 1, 1);

        label_7 = new QLabel(centralWidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        gridLayout_3->addWidget(label_7, 2, 0, 1, 1);


        verticalLayout_2->addLayout(gridLayout_3);


        gridLayout->addLayout(verticalLayout_2, 2, 1, 1, 1);

        frame = new QFrame(centralWidget);
        frame->setObjectName(QString::fromUtf8("frame"));
        QSizePolicy sizePolicy2(QSizePolicy::MinimumExpanding, QSizePolicy::Expanding);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy2);
        frame->setMinimumSize(QSize(600, 450));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);

        gridLayout->addWidget(frame, 2, 0, 1, 1);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 889, 26));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        pushButton_9->setText(QCoreApplication::translate("MainWindow", "Start", nullptr));
        lineEdit->setText(QCoreApplication::translate("MainWindow", "10", nullptr));
        pushButton->setText(QCoreApplication::translate("MainWindow", "Use camera", nullptr));
        pushButton_4->setText(QCoreApplication::translate("MainWindow", "Stop", nullptr));
        pushButton_3->setText(QCoreApplication::translate("MainWindow", "Back", nullptr));
        pushButton_5->setText(QCoreApplication::translate("MainWindow", "Right", nullptr));
        pushButton_6->setText(QCoreApplication::translate("MainWindow", "Left", nullptr));
        pushButton_2->setText(QCoreApplication::translate("MainWindow", "Forward", nullptr));
        pushButton_7->setText(QCoreApplication::translate("MainWindow", "ClearPoints", nullptr));
        pushButton_10->setText(QCoreApplication::translate("MainWindow", "DoConvolution", nullptr));
        lineEdit_5->setText(QCoreApplication::translate("MainWindow", "0.005", nullptr));
        lineEdit_6->setText(QCoreApplication::translate("MainWindow", "1000", nullptr));
        pushButton_8->setText(QCoreApplication::translate("MainWindow", "Reset robot", nullptr));
        btnRegulation->setText(QCoreApplication::translate("MainWindow", "StartRegulation", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "X:", nullptr));
        label_8->setText(QCoreApplication::translate("MainWindow", "Y:", nullptr));
        leXpoint->setText(QCoreApplication::translate("MainWindow", "4", nullptr));
        leYpoint->setText(QCoreApplication::translate("MainWindow", "0.5", nullptr));
        btnAddPoint->setText(QCoreApplication::translate("MainWindow", "AddPoint", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "Y:", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "X:", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "Tra:", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "Rot:", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "Ang:", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
