#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "qlineedit.h"
#include <QMainWindow>
#include <QTimer>
#ifdef _WIN32
#include<windows.h>
#endif
#include<iostream>
//#include<arpa/inet.h>
//#include<unistd.h>
//#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>
//#include "ckobuki.h"
//#include "rplidar.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#include "robot.h"
#include <QJoysticks.h>
#include <chrono>
#include <QAbstractTableModel>
#include <list>


struct Point {
    float x;
    float y;
};

struct PointI {
    int x;
    int y;
};

union TMapPoint {
    Point point;
    double suradnice[2];
};

struct TMapObject {
    int numOfPoints;
    std::vector<TMapPoint> points;
};

struct TMapArea {
    TMapObject wall;
    int numOfObjects;
    std::vector<TMapObject> obstacle;
};

class PointTableModel : public QAbstractTableModel
{
public:
    PointTableModel(QObject *parent = nullptr)
            : QAbstractTableModel(parent)
    {
    }

    int rowCount(const QModelIndex &parent = QModelIndex()) const override
    {
        return m_points.size();
    }

    int columnCount(const QModelIndex &parent = QModelIndex()) const override
    {
        return 2; // Assuming Point has x and y coordinates
    }

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override
    {
        if (!index.isValid())
            return QVariant();

        if (index.row() >= m_points.size())
            return QVariant();

        const Point &point = *std::next(m_points.begin(), index.row());

        if (role == Qt::DisplayRole)
        {
            if (index.column() == 0)
                return QVariant(point.x);
            else if (index.column() == 1)
                return QVariant(point.y);
        }

        return QVariant();
    }

    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override
    {
        if (role == Qt::DisplayRole && orientation == Qt::Horizontal)
        {
            if (section == 0)
                return QVariant("X");
            else if (section == 1)
                return QVariant("Y");
        }

        return QVariant();
    }

    void push_back(Point point){
        m_points.push_back(point);
        newItemInserted();
    }

    void pop_front(){
        m_points.pop_front();
        newItemInserted();
    }

    Point front(){
        return m_points.front();
    }



private:
    std::list<Point> m_points;

    void newItemInserted(){
        beginInsertRows(QModelIndex(), rowCount(), rowCount());
        endInsertRows();
    }
};



struct RobotState {
    double  x; // position in meters
    double y; // position in meters
    double angle; // orientation in radians
    double forwardSpeed; //mm/s
    double angularSpeed; //omega/s
};

namespace Ui {
class MainWindow;
}

///toto je trieda s oknom.. ktora sa spusti ked sa spusti aplikacia.. su tu vsetky gombiky a spustania...
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    bool useCamera1;
  //  cv::VideoCapture cap;

    int actIndex;
    //    cv::Mat frame[3];

    cv::Mat frame[3];
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    int processThisLidar(LaserMeasurement laserData);

    int processThisRobot(TKobukiData robotdata);

int processThisCamera(cv::Mat cameraData);

private slots:
    void on_pushButton_9_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_clicked();
    void getNewFrame();

    void on_pushButton_8_clicked();

    void on_btnAddPoint_clicked();

    void on_btnRegulation_clicked();

private:

    //--skuste tu nic nevymazat... pridavajte co chcete, ale pri odoberani by sa mohol stat nejaky drobny problem, co bude vyhadzovat chyby
    Ui::MainWindow *ui;
     void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
     int updateLaserPicture;
     LaserMeasurement copyOfLaserData;
     std::string ipaddress;
     Robot robot;
     TKobukiData robotdata;
     int datacounter;
     int lidarDataCounter = 0;
     int paintEventCounter = 0;
     QTimer *timer;

     QJoysticks *instance;
     bool connected = false;
     long double tickToMeter = 0.000085292090497737556558; // [m/tick]
     long double wheelbase = 0.23;
     const unsigned short ENCODER_MAX = 65535;  // define maximum encoder value
     float regulatorTranslateProportionalElement = 2000;
     float regulatorAngularProportionalElement = 3.141592*2;
     int rampTranslateConstant = 10; // mm/s
     float rampAngularConstant = 0.1; //omega/s
     int translateSaturationValue = 350;//mm/s;
     float angularSaturationValue = 3.14159/2;//omega/s

     long double oldEncoderLeft;
     long double oldEncoderRight;
     RobotState state = {0, 0, 0}; // starting at (0, 0)
     PointTableModel *pointsModel;
     std::chrono::steady_clock::time_point regulation_start_time;
     int wall_edge_detect_frequenc = 2;
     bool regulating = false;
     bool risingEdgeOfRegulating = false;
     float checkLineEdit(QLineEdit *lineEdit);
     void toogleRegulationButton();

     void updateRobotState(long double encoderLeft, long double encoderRight);
     void regulate();
     void stopRobot();
     void evaluateSaturation();
     void evaluateAngleRamp(float targetangle);
//     bool checkIfPointIsInRobotsWay(float x, float y, float x1, float y1, float x2, float y2);
     bool checkIfPointIsInRobotsWay();

     int robotZone = 30;

     bool mapping = false;
     int gridSizeBlockInMM = 100;
     int rows = 10;
     int cols  = 10;
     vector<vector<int>> map;  // initialize with zeros

     void printMap();
     void save_map();
     void mapResize();


public slots:
     void setUiValues();
signals:
     void uiValuesChanged(); ///toto nema telo


};

#endif // MAINWINDOW_H
