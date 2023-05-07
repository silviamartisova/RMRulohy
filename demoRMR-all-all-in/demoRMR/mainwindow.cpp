#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
///TOTO JE DEMO PROGRAM...AK SI HO NASIEL NA PC V LABAKU NEPREPISUJ NIC,ALE SKOPIRUJ SI MA NIEKAM DO INEHO FOLDERA
/// AK HO MAS Z GITU A ROBIS NA LABAKOVOM PC, TAK SI HO VLOZ DO FOLDERA KTORY JE JASNE ODLISITELNY OD TVOJICH KOLEGOV
/// NASLEDNE V POLOZKE Projects SKONTROLUJ CI JE VYPNUTY shadow build...
/// POTOM MIESTO TYCHTO PAR RIADKOV NAPIS SVOJE MENO ALEBO NEJAKY INY LUKRATIVNY IDENTIFIKATOR
/// KED SA NAJBLIZSIE PUSTIS DO PRACE, SKONTROLUJ CI JE MIESTO TOHTO TEXTU TVOJ IDENTIFIKATOR
/// AZ POTOM ZACNI ROBIT... AK TO NESPRAVIS, POJDU BODY DOLE... A NIE JEDEN,ALEBO DVA ALE BUDES RAD
/// AK SA DOSTANES NA SKUSKU


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)

{

    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    ipaddress="192.168.1.14";//192.168.1.14toto je na niektory realny robot.. na lokal budete davat "127.0.0.1"
  //  cap.open("http://192.168.1.11:8000/stream.mjpg");
    ui->setupUi(this);
    datacounter=0;
  //  timer = new QTimer(this);
//    connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
    actIndex=-1;
    useCamera1=false;

    pointsModel = new PointTableModel();
    ui->tbPoints->setModel(pointsModel);

//    pointsModel->push_back(Point{1, 1});
//    pointsModel->push_back(Point{0.2, 0});
//    pointsModel->push_back(Point{1.2, -0.2});
//    pointsModel->push_back(Point{0, 0});
//    pointsModel->push_back(Point{0, 3.5});
//    pointsModel->push_back(Point{1, 0});


    datacounter=0;


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    paintEventCounter++;
//    doConvolution();
    QPainter painter(this);
    ///prekreslujem obrazovku len vtedy, ked viem ze mam nove data. paintevent sa
    /// moze pochopitelne zavolat aj z inych dovodov, napriklad zmena velkosti okna
    painter.setBrush(Qt::black);//cierna farba pozadia(pouziva sa ako fill pre napriklad funkciu drawRect)
    QPen pero;
    pero.setStyle(Qt::SolidLine);//styl pera - plna ciara
    pero.setWidth(3);//hrubka pera -3pixely
    pero.setColor(Qt::green);//farba je zelena
    QRect rect;
    rect= ui->frame->geometry();//ziskate porametre stvorca,do ktoreho chcete kreslit
    rect.translate(0,15);
    painter.drawRect(rect);


    if(useCamera1==true && actIndex>-1)/// ak zobrazujem data z kamery a aspon niektory frame vo vectore je naplneny
    {
//        std::cout<<actIndex<<std::endl;
        QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888);//kopirovanie cvmat do qimage
        painter.drawImage(rect,image.rgbSwapped());
    }
    else
    {
        if(updateLaserPicture==1) ///ak mam nove data z lidaru
        {
            updateLaserPicture=0;

            painter.setPen(pero);
            //teraz tu kreslime random udaje... vykreslite to co treba... t.j. data z lidaru
         //   std::cout<<copyOfLaserData.numberOfScans<<std::endl;

            for(int k=0, i = 0;k<copyOfLaserData.numberOfScans/*360*/;k++)
            {
                int dist=copyOfLaserData.Data[k].scanDistance/20; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
                int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x(); //prepocet do obrazovky
                int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();//prepocet do obrazovky

                pero.setColor(Qt::green);
                if(i < convolutionResults.size() && k == convolutionResults[i].index){
                    i++;
                    pero.setColor(Qt::red);
                }

                if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                {
//                    pero.setColor(Qt::green);
//                    if(k == 0)pero.setColor(Qt::yellow);
//                    if(k == 1)pero.setColor(Qt::red);
//                    if(k == upEdgeIndex)pero.setColor(Qt::magenta);
                    if(k == k_wall)pero.setColor(Qt::magenta);
                    if(k == indexOfBlockingPoint)pero.setColor(Qt::white);

                    painter.setPen(pero);
                    painter.drawEllipse(QPoint(xp, yp),2,2);
                }               

                // U2

//                if((paintEventCounter % wall_edge_detect_frequenc == 0) || risingEdgeOfRegulating){
//                        if(k == 0)first_point = prev_point = PointI{xp, yp};

//                    cur_point = PointI{xp, yp};

//                    distance = sqrt(pow(cur_point.x - prev_point.x, 2) + pow(cur_point.y - prev_point.y, 2));
////                    cout << "distance: " << distance << endl;
//                    if (distance > robotZone*2)
//                    {
////                        cout << "edge found! " << paintEventCounter << endl;
//                        // Endpoint of an obstacle wall detected
//                        wall_endpoints.push_back(prev_point);
//                        wall_endpoints.push_back(cur_point);
//                        cout << "PAINTEVENT: " << k << endl;

//                        if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
//                        {
//                            pero.setColor(Qt::red);
//                            painter.setPen(pero);
//                            painter.drawEllipse(QPoint(xp, yp),2,2);
//                        }
//                        if(rect.contains(prev_point.x,prev_point.y))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
//                        {
//                            pero.setColor(Qt::red);
//                            painter.setPen(pero);
//                            painter.drawEllipse(QPoint(prev_point.x, prev_point.y),2,2);
//                        }
//                    }

//                    prev_point = cur_point;

//                }

            }
// U2
//            if(((paintEventCounter % wall_edge_detect_frequenc == 0) || risingEdgeOfRegulating) && pointsModel->rowCount()>0){

//                Point destPoint = pointsModel->front();
//                // Translate point to be relative to robot's position
//                double rel_point_x = destPoint.x-state.x;
//                double rel_point_y = destPoint.y-state.y;

//                /////////////////////
//                float dx = destPoint.x - state.x;
//                float dy = destPoint.y - state.y;
//                cout << "dx: " << dx << " dy: " << dy << endl;
//                float angle_between_point_and_robots_view = std::atan2(dy, dx) - state.angle;
////                angle_between_point_and_robots_view = fmod(3.14159265358979*2 -angle_between_point_and_robots_view + 3.14159265358979, 3.14159265358979*2) - 3.14159265358979;

//                cout << angle_between_point_and_robots_view*180/3.14159265358979 << endl;


//                // Apply 2D rotation to point
//                double cos_theta = cos(state.angle+angle_between_point_and_robots_view);
//                double sin_theta = sin(state.angle+angle_between_point_and_robots_view);
//                double rot_point_x = cos_theta * rel_point_x - sin_theta * rel_point_y;
//                double rot_point_y = sin_theta * rel_point_x + cos_theta * rel_point_y;
//                // Translate point back to absolute position
//                double new_point_x = rot_point_x;
//                double new_point_y = rot_point_y;
////                // Translate point back to absolute position
////                double new_point_x = state.x + rot_point_x;
////                double new_point_y = state.y + rot_point_y;

//                new_point_x *= 100;
//                new_point_y *= 100;
//                new_point_x += rect.topLeft().x() + rect.width()/2;
//                new_point_y += rect.topLeft().y() + rect.height()/2;
////                new_point_x += - state.x*50 + rect.topLeft().x() + rect.width()/2;
////                new_point_y += - state.y*50 + rect.topLeft().y() + rect.height()/2;

//                cout << "x: " << new_point_x<< " y: " << new_point_y<< endl;
//                pero.setColor(Qt::magenta);
//                painter.setPen(pero);
//                painter.drawEllipse(QPoint(new_point_x, new_point_y),2,2);

//            }

            pero.setColor(Qt::white);
            painter.setPen(pero);
            painter.drawEllipse(QPoint(rect.topLeft().x()+rect.width()/2, rect.topLeft().y()+rect.height()/2),2,2);
            int xp=rect.width()-(rect.width()/2+newDistance/10*sin((360.0-newAngle)*3.14159/180.0))+rect.topLeft().x(); //prepocet do obrazovky
            int yp=rect.height()-(rect.height()/2+newDistance/10*cos((360.0-newAngle)*3.14159/180.0))+rect.topLeft().y();//prepocet do obrazovky
            pero.setColor(Qt::red);
            painter.setPen(pero);
            painter.drawEllipse(QPoint(xp, yp),2,2);
        }
    }
}


/// toto je slot. niekde v kode existuje signal, ktory je prepojeny. pouziva sa napriklad (v tomto pripade) ak chcete dostat data z jedneho vlakna (robot) do ineho (ui)
/// prepojenie signal slot je vo funkcii  on_pushButton_9_clicked
void  MainWindow::setUiValues()
{
     ui->lineEdit_2->setText(QString::number(state.x));
     ui->lineEdit_3->setText(QString::number(state.y));
     ui->lineEdit_4->setText(QString::number(state.angle));
     ui->leTranslationSpeed->setText(QString::number(state.forwardSpeed));
     ui->leRotationSpeed->setText(QString::number(state.angularSpeed));
}

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int MainWindow::processThisRobot(TKobukiData robotdata)
{


    ///tu mozete robit s datami z robota
    /// ale nic vypoctovo narocne - to iste vlakno ktore cita data z robota
    ///teraz tu posielam rychlosti na zaklade toho co setne joystick a vypisujeme data z robota(kazdy 5ty krat. ale mozete skusit aj castejsie). vyratajte si polohu. a vypiste spravnu
    /// tuto joystick cast mozete vklude vymazat,alebo znasilnit na vas regulator alebo ake mate pohnutky... kazdopadne, aktualne to blokuje gombiky cize tak
//    if(forwardspeed==0 && rotationspeed!=0)
//        robot.setRotationSpeed(rotationspeed);
//    else if(forwardspeed!=0 && rotationspeed==0)
//        robot.setTranslationSpeed(forwardspeed);
//    else if((forwardspeed!=0 && rotationspeed!=0))
//        robot.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
//    else
//        robot.setTranslationSpeed(0);

///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

    updateRobotState(robotdata.EncoderLeft, robotdata.EncoderRight);
    state.angle=state.angle>3.14159265358979 ? state.angle-2*3.14159265358979:state.angle<-3.14159265358979?state.angle+2*3.14159265358979:state.angle;



    if(regulating)regulate();

    if(datacounter%5)
    {


        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
                // ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
                //ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
                //ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
                /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
                /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
                /// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
        ///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
        emit uiValuesChanged();
        ///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde

    }
    datacounter++;

    return 0;

}

float MainWindow::getPointsDistance(Point a, Point b){
    return sqrtf(powf(a.x - b.x, 2) + powf(a.y - b.y, 2));
}

void MainWindow::findNearestGap(){
    Point prev_point;
    Point cur_point;
    Point first_point;
    float distance;
    Point upPoint;
    Point lowPoint;

//    cout << "k is: " << indexOfBlockingPoint << endl;
    for(int k = indexOfBlockingPoint, i = indexOfBlockingPoint; k<indexOfBlockingPoint+copyOfLaserData.numberOfScans/*360*/; k++, i++){
        i = k % copyOfLaserData.numberOfScans;
        float x = state.x + copyOfLaserData.Data[i].scanDistance/1000*cos(state.angle+(360-copyOfLaserData.Data[i].scanAngle)*3.14159/180.0);
        float y = state.y + copyOfLaserData.Data[i].scanDistance/1000*sin(state.angle+(360-copyOfLaserData.Data[i].scanAngle)*3.14159/180.0);


        if(k == indexOfBlockingPoint)first_point = prev_point = Point{x, y};
        cur_point = Point{x, y};
        distance = getPointsDistance(cur_point, prev_point);
        if (distance > robotZone/100.0*2)// /100 is to change cm to m
        {
//            cout << "up edge found! " << i << endl;
            upPoint = prev_point;
            break;
        }
        prev_point = cur_point;
        upEdgeIndex = i;

//        cout << "i: " << i << endl;
    }

    for(int k = indexOfBlockingPoint, i = indexOfBlockingPoint; k>=-copyOfLaserData.numberOfScans+indexOfBlockingPoint; k--, i--){
        if(i < 0) i += copyOfLaserData.numberOfScans-1;
        float x = state.x + copyOfLaserData.Data[i].scanDistance/1000*cos(state.angle+(360-copyOfLaserData.Data[i].scanAngle)*3.14159/180.0);
        float y = state.y + copyOfLaserData.Data[i].scanDistance/1000*sin(state.angle+(360-copyOfLaserData.Data[i].scanAngle)*3.14159/180.0);


        if(k == indexOfBlockingPoint)first_point = prev_point = Point{x, y};
        cur_point = Point{x, y};
        distance = getPointsDistance(cur_point, prev_point);
        if (distance > robotZone/100.0*2)// /100 is to change cm to mm
        {
//            cout << "low edge found! " << i << endl;
            lowPoint = prev_point;
            break;
        }
        prev_point = cur_point;
        lowEdgeIndex = i;
//        cout << "i: " << i << endl;
    }

    // Zratenie potencialnej dlzky
    if((getPointsDistance(upPoint, Point{(float)state.x, (float)state.y}) + getPointsDistance(upPoint, pointsModel->front())) > (getPointsDistance(lowPoint, Point{(float)state.x, (float)state.y}) + getPointsDistance(lowPoint, pointsModel->front()))){

        // skusme spravit posun do jednej a do druhej strany oproti najdenemu bodu, novy bod by mal mat:
        // uvazujme posun o dlzku "r", potom distance noveho bodu by mal byt sqrt(distancaStary^2 + r^2)
        // angle noveho bodu by potom mal byt angleStarehoBodu + atan2(r, distance)
        float oldDistance = copyOfLaserData.Data[lowEdgeIndex].scanDistance/1000;
        float r = robotZone/100.0/1.2;
        float newDistance = sqrtf(oldDistance*oldDistance + r*r);
        float newAngle = (360-copyOfLaserData.Data[lowEdgeIndex].scanAngle)*3.14159/180.0 + atan2(r, oldDistance);
        newDistance*=1.2;
        float x = state.x + newDistance*cos(state.angle+newAngle);
        float y = state.y + newDistance*sin(state.angle+newAngle);
        bool free = checkIfPointIsInRobotsWay(Point{x,y});
        risingEdgeOfRegulating = true;
        if(!free){
            pointsModel->push_front(Point{x,y});
//            cout << "inserting low point. X: " << x << " Y: " << y << endl;
        }
        else{
            float oldDistance = copyOfLaserData.Data[lowEdgeIndex].scanDistance/1000;
            float r = -robotZone/100.0/1.2;
            float newDistance = sqrtf(oldDistance*oldDistance + r*r);
            float newAngle = (360-copyOfLaserData.Data[lowEdgeIndex].scanAngle)*3.14159/180.0 + atan2(r, oldDistance);
            newDistance*=1.2;
    //        newAngle*=1.05;
            float x = state.x + newDistance*cos(state.angle+newAngle);
            float y = state.y + newDistance*sin(state.angle+newAngle);
            bool free = checkIfPointIsInRobotsWay(Point{x,y});
            risingEdgeOfRegulating = true;
            if(!free){
                pointsModel->push_front(Point{x,y});
//                cout << "iinserting low point. X: " << x << " Y: " << y << endl;
            }
        }

    }else{

        float oldDistance = copyOfLaserData.Data[upEdgeIndex].scanDistance/1000;
        float r = robotZone/100.0/1.2;
        float newDistance = sqrtf(oldDistance*oldDistance + r*r);
        float newAngle = (360-copyOfLaserData.Data[upEdgeIndex].scanAngle)*3.14159/180.0 + atan2(r, oldDistance);
        newDistance*=1.2;
        float x = state.x + newDistance*cos(state.angle+newAngle);
        float y = state.y + newDistance*sin(state.angle+newAngle);
        bool free = checkIfPointIsInRobotsWay(Point{x,y});
        risingEdgeOfRegulating = true;
        if(!free){
            pointsModel->push_front(Point{x,y});
//            cout << "inserting up point. X: " << x << " Y: " << y << endl;
        }else {
            float oldDistance = copyOfLaserData.Data[upEdgeIndex].scanDistance/1000;
            float r = -robotZone/100.0/1.2;
            float newDistance = sqrtf(oldDistance*oldDistance + r*r);
            float newAngle = (360-copyOfLaserData.Data[upEdgeIndex].scanAngle)*3.14159/180.0 + atan2(r, oldDistance);
            newDistance*=1.2;
            float x = state.x + newDistance*cos(state.angle+newAngle);
            float y = state.y + newDistance*sin(state.angle+newAngle);
            bool free = checkIfPointIsInRobotsWay(Point{x,y});
            risingEdgeOfRegulating = true;
            if(!free){
                pointsModel->push_front(Point{x,y});
//                cout << "iinserting up point. X: " << x << " Y: " << y << endl;
            }
        }
    }
}

void MainWindow::wallFollowing(){

    double minDistance = copyOfLaserData.Data->scanDistance;
    double angleOfMinDistance = copyOfLaserData.Data->scanAngle;
    for(int k=1;k<copyOfLaserData.numberOfScans/*360*/;k++)
    {
        if(!(copyOfLaserData.Data[k].scanDistance > 130 && copyOfLaserData.Data[k].scanDistance < 3000 && !(copyOfLaserData.Data[k].scanDistance < 640 && copyOfLaserData.Data[k].scanDistance > 700)))continue; // vyhod zle data
        if(copyOfLaserData.Data[k].scanDistance < minDistance){
            minDistance = copyOfLaserData.Data[k].scanDistance;
            angleOfMinDistance = copyOfLaserData.Data[k].scanAngle;
            k_wall = k;
        }
    }

    float wantedRobotDistance = 450;
    float angleShift = (-90) * (rightWall ? 1 : -1);
    angleShift += (minDistance - wantedRobotDistance)/5*(rightWall ? 1 : -1);
    newDistance = minDistance < 600 ? 600 : minDistance;
    newAngle = angleOfMinDistance + angleShift;
    newAngle=newAngle>180 ? newAngle-360:newAngle<-180?newAngle+360:newAngle;
//    newAngle=newAngle>PI ? newAngle-PI*2:newAngle<-PI?newAngle+PI*2:newAngle;




    float x = state.x*1000 + minDistance*cos(state.angle+(360-angleOfMinDistance)*3.14159/180.0);
    float y = state.y*1000 + minDistance*sin(state.angle+(360-angleOfMinDistance)*3.14159/180.0);
//    x_wall_follow = (x*cos(PI/2)+y*sin(PI/2))/1000;
//    y_wall_follow = (-x*sin(PI/2)+y*cos(PI/2))/1000;
    x_wall_follow = (state.x*1000 + newDistance*cos(state.angle+(360-newAngle)*3.14159/180.0))/1000;
    y_wall_follow = (state.y*1000 + newDistance*sin(state.angle+(360-newAngle)*3.14159/180.0))/1000;
//    cout << "index of nearest point: " << k_wall << endl;
    cout << "x_wall: " << x/1000 << ", y_wall: " << y/1000 << ", angle: " << angleOfMinDistance << ", distance: " << minDistance/1000 << endl;
//    cout << "x_new: " << x_wall_follow << ", y_new: " << y_wall_follow << endl;
    cout << "new engle: " << newAngle << ", angleShift: " << angleShift << endl;
}


int MainWindow::processThisLidar(LaserMeasurement laserData)
{

    memcpy(&copyOfLaserData,&laserData,sizeof(LaserMeasurement));

//    if(wallFolower){
//        wallFollowing();
//    }
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru#
    // Uloha 2
    if((risingEdgeOfRegulating || (lidarDataCounter%20==0)) && pointsModel->rowCount() > 0){
        risingEdgeOfRegulating = false;


        if(pointsModel->rowCount() > 1){
            tmpPoint = pointsModel->back();
//            distanceFromDestination = getPointsDistance(Point{(float)state.x, (float)state.y}, tmpPoint);
    //            cout << distanceFromDestination << ", " << oldDistanceFromDestination << endl;
//            if(!wallFolower && oldDistanceFromDestination < distanceFromDestination){
//                lastBeforeDistanceFromDestination = distanceFromDestination;
//                wallFolower = true;
//                pointsModel->pop_front();
//                cout << "wallFolower: " << wallFolower << endl;
//            }
//            oldDistanceFromDestination = distanceFromDestination;

//        }else if(!wallFolower){
//            oldDistanceFromDestination = 9999;
//            distanceFromDestination = 9999;
        }
//        else if(wallFolower){
//            distanceFromDestination = getPointsDistance(Point{(float)state.x, (float)state.y}, tmpPoint);
//    //            cout << distanceFromDestination << ", last: " << lastBeforeDistanceFromDestination << endl;
//            if(lastBeforeDistanceFromDestination > distanceFromDestination && !checkIfPointIsInRobotsWay(tmpPoint)){
//                wallFolower = false;
//                cout << "wallFolower: " << wallFolower << endl;
//            }
//        }


        if(checkIfPointIsInRobotsWay(pointsModel->front())){
            cout << "There is a barier on the way to point!" << endl;

            if(wallFolower){

            }else{
                findNearestGap();
            }
        }
    }


//    // U3
//    for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
//    {
//        if(mapping && copyOfLaserData.Data[k].scanDistance > 130 && copyOfLaserData.Data[k].scanDistance < 3000 && !(copyOfLaserData.Data[k].scanDistance < 640 && copyOfLaserData.Data[k].scanDistance > 700)){
//            float x = state.x*1000 + copyOfLaserData.Data[k].scanDistance*cos(state.angle+(360-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0);
//            float y = state.y*1000 + copyOfLaserData.Data[k].scanDistance*sin(state.angle+(360-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0);

//            x /= gridSizeBlockInMM;
//            y /= gridSizeBlockInMM;

//            if (y >= map.size() || x >= map[0].size()) {
//                rows = fmax(y + 1, (int)map.size());
//                cols = fmax(x + 1, (int)map[0].size());
//                mapResize();
//            }

//            // Insert an obstacle at the scaled coordinates
////            cout << "x: " << x << " y: " << y << " robotx: " << state.x << " roboty: " << state.y << " robot angle: " << state.angle <<  endl;

//            map[y][x] = 1;
//        }
//    }
//    save_map();
////    printMap();

    updateLaserPicture=1;
    update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia

    lidarDataCounter++;
    return 0;

}

///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z kamery
int MainWindow::processThisCamera(cv::Mat cameraData)
{

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka
    updateLaserPicture=1;
    return 0;
}
void MainWindow::on_pushButton_9_clicked() //start button
{
    if(connected){
        R = !R;
        return;
    }

    state.forwardSpeed=0;
    state.angularSpeed=0;
    state.x = 0.5;
    state.y = 0.5;

    //U4
//    loadMap();
//    printMap();
//    expandWalls();
//    printMap();
//    mainLogigOfU4();
//    printMap();



//    mapResize();
//    printMap();
//    rows = 14;
//    cols = 13;
//    mapResize();
//    printMap();
//    map[12][11] = 5;
//    rows = 18;
//    cols = 14;
//    mapResize();
//    printMap();
//    rows = 13;
//    cols = 14;
//    mapResize();
//    printMap();
//    map[8][11] = 9;
//    printMap();
//    return;


    // tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    connect(this,SIGNAL(uiValuesChanged()),this,SLOT(setUiValues()));

    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robot.setLaserParameters(ipaddress,52999,5299,/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::processThisLidar,this,std::placeholders::_1));
    robot.setRobotParameters(ipaddress,53000,5300,std::bind(&MainWindow::processThisRobot,this,std::placeholders::_1));
    //---simulator ma port 8889, realny robot 8000
    robot.setCameraParameters("http://"+ipaddress+":8889/stream.mjpg",std::bind(&MainWindow::processThisCamera,this,std::placeholders::_1));

    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robot.robotStart();



    // ziskanie joystickov
    instance = QJoysticks::getInstance();


    /// prepojenie joysticku s jeho callbackom... zas cez lambdu. neviem ci som to niekde spominal,ale lambdy su super. okrem toho mam este rad ternarne operatory a spolocneske hry ale to tiez nikoho nezaujima
    /// co vas vlastne zaujima? citanie komentov asi nie, inak by ste citali toto a ze tu je blbosti
    connect(
        instance, &QJoysticks::axisChanged,
        [this]( const int js, const int axis, const qreal value) { if(/*js==0 &&*/ axis==1){state.forwardSpeed=-value*300;}
            if(/*js==0 &&*/ axis==0){state.angularSpeed=-value*(3.14159/2.0);}}
    );



    oldEncoderLeft = robotdata.EncoderLeft;
    oldEncoderRight = robotdata.EncoderRight;
    connected = true;
}

void MainWindow::on_pushButton_2_clicked() //forward
{
    //pohyb dopredu
    mapping = true;
    robot.setTranslationSpeed(300);

}

void MainWindow::on_pushButton_3_clicked() //back
{
    mapping = true;
    robot.setTranslationSpeed(-250);
}

void MainWindow::on_pushButton_6_clicked() //left
{
    mapping = false;
    robot.setRotationSpeed(3.14159/2);
}

void MainWindow::on_pushButton_5_clicked()//right
{
    mapping = false;
    robot.setRotationSpeed(-3.14159/2);
}

void MainWindow::on_pushButton_4_clicked() //stop
{
    stopRobot();
    mapping = false;
    printMap();
}




void MainWindow::on_pushButton_clicked()
{
    if(useCamera1==true)
    {
        useCamera1=false;

        ui->pushButton->setText("use camera");
    }
    else
    {
        useCamera1=true;

        ui->pushButton->setText("use laser");
    }
}

void MainWindow::getNewFrame()
{

}

void MainWindow::updateRobotState(long double encoderLeft, long double encoderRight) {

    // calculate the difference in encoder counts for each wheel
    long double deltaEncoderLeft = encoderLeft - oldEncoderLeft;
    long double deltaEncoderRight = encoderRight - oldEncoderRight;

    // handle encoder overflow by adding or subtracting the maximum encoder value
    if (deltaEncoderLeft > ENCODER_MAX / 2) {
        deltaEncoderLeft -= ENCODER_MAX;
    } else if (deltaEncoderLeft < -ENCODER_MAX / 2) {
        deltaEncoderLeft += ENCODER_MAX;
    }
    if (deltaEncoderRight > ENCODER_MAX / 2) {
        deltaEncoderRight -= ENCODER_MAX;
    } else if (deltaEncoderRight < -ENCODER_MAX / 2) {
        deltaEncoderRight += ENCODER_MAX;
    }

    // calculate the distance traveled by each wheel
    long double distanceLeft = deltaEncoderLeft * tickToMeter;
    long double distanceRight = deltaEncoderRight * tickToMeter;

    // calculate the average distance traveled by the robot
    long double distance = (distanceLeft + distanceRight) / 2;

    // calculate the change in angle of the robot
    long double deltaAngle = (distanceRight - distanceLeft) / wheelbase;

    if(distanceRight == distanceLeft){
        // update the robot's position and orientation
        state.x += distance * cos(state.angle + deltaAngle / 2);
        state.y += distance * sin(state.angle + deltaAngle / 2);
        state.angle += deltaAngle;

    }
    else{

        state.x += ((wheelbase*(distanceRight + distanceLeft))/(2*(distanceRight - distanceLeft)))*(sin(state.angle+deltaAngle)-sin(state.angle));
        state.y -= ((wheelbase*(distanceRight + distanceLeft))/(2*(distanceRight - distanceLeft)))*(cos(state.angle+deltaAngle)-cos(state.angle));
        state.angle += deltaAngle;
    }
    oldEncoderLeft = encoderLeft;
    oldEncoderRight = encoderRight;

//    cout << "x: " << state.x << "; y: " << state.y << "; angle: " << state.angle << endl;
}



void MainWindow::on_pushButton_8_clicked()
{
    state.angle = 0;
    state.x = 0;
    state.y = 0;
    mapping = true;
}

void MainWindow::regulate(){
    float dx;
    float dy;

    if(wallFolower){
        dx = x_wall_follow - state.x;
        dy = y_wall_follow - state.y;
    }
    else{
        if(pointsModel->rowCount() == 0){
            // There are no more points to go
            toogleRegulationButton();
            cout << "No points to go!" << endl;
            return;
        }
        Point destinationPoint = pointsModel->front();


    //    if(wallFolower && checkIfPointIsInRobotsWay(destinationPoint)){
    //        pointsModel->pop_front();
    //        cout << "cant reach the point, probably fake corner!" << endl;
    //        return;
    //    }

        // Destination reached
        if((abs(state.x - destinationPoint.x) < 0.01) && (abs(state.y - destinationPoint.y) < 0.01)){
            pointsModel->pop_front();
            risingEdgeOfRegulating = true;
            cout << "point x: " << destinationPoint.x << " y: " << destinationPoint.y << " reached!!" << endl;
            return;
        }
        cout << "x: " << x_wall_follow << ", y: " << y_wall_follow << endl;

//         Calculate the distance and angle between the robot and the destination
        dx = destinationPoint.x - state.x;
        dy = destinationPoint.y - state.y;
    }
//    cout << "forwarSpeed: " << state.forwardSpeed << " angular speed: " << state.angularSpeed << endl;

    float distance = std::sqrt(dx*dx + dy*dy);
    float angle = std::atan2(dy, dx) - state.angle;
//    cout << "angle before:  " << angle << endl;
    // difference between -π and π.
    angle = fmod(angle + 3.14159265358979, 3.14159265358979*2) - 3.14159265358979;
//    cout << "angle after:  " << angle << endl;

    // Check if angle isnt too big
    if(abs(angle) > 0.785398){ // 45 degrees
        state.forwardSpeed = 0;
        evaluateAngleRamp(angle);
        evaluateSaturation();
        robot.setRotationSpeed(state.angularSpeed);
//        cout << "angle is too big!  " << angle << endl;
        return;
    }

    // Create ramp effect, if needed
    double acceleration = distance * regulatorTranslateProportionalElement - state.forwardSpeed;
    if(acceleration > rampTranslateConstant){
        state.forwardSpeed += rampTranslateConstant;
    }else{
        state.forwardSpeed = distance * regulatorTranslateProportionalElement;
    }
    evaluateAngleRamp(angle);
    evaluateSaturation();

    if(state.angularSpeed == 0){
        robot.setTranslationSpeed(state.forwardSpeed);
    }else
    robot.setArcSpeed(state.forwardSpeed, state.forwardSpeed/state.angularSpeed);

}

float MainWindow::checkLineEdit(QLineEdit *lineEdit) {
    QString input = lineEdit->text();
    bool ok = false;
    float value = input.toFloat(&ok);
    if (input.isEmpty()) {
        cout << "Error " << "Input is empty" << endl;
    } else if (!ok) {
        cout << "Error " << "Input is not a valid number" << endl;
    } else {
        return value;
    }
    return 0.01;
}

void MainWindow::on_btnAddPoint_clicked()
{
    float x = checkLineEdit(ui->leXpoint);
    float y = checkLineEdit(ui->leYpoint);
    if(x != 0.01f && y != 0.01f){
        pointsModel->push_back(Point {x, y});
        ui->leXpoint->clear();
        ui->leYpoint->clear();
    }
}

void MainWindow::toogleRegulationButton(){
    if(ui->btnRegulation->text() == "StartRegulation"){
        ui->btnRegulation->setText("StopRegulation");
        regulating = true;
        risingEdgeOfRegulating = true;

        regulation_start_time = std::chrono::steady_clock::now();
        cout << "Starting regulation" << endl;
    }else{
        ui->btnRegulation->setText("StartRegulation");
        regulating = false;
        stopRobot();

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - regulation_start_time);
        cout << "StopingRegulation" << endl;
        std::cout << "Time taken: " << duration.count() << " milliseconds" << std::endl;
    }
}

void MainWindow::stopRobot(){
    robot.setTranslationSpeed(0);
    state.angularSpeed = 0;
    state.forwardSpeed = 0;
}

void MainWindow::evaluateSaturation(){
    // Saturation
    if(state.forwardSpeed > translateSaturationValue){
        state.forwardSpeed = translateSaturationValue;
    }
    else if(state.forwardSpeed < -translateSaturationValue){
        state.forwardSpeed = -translateSaturationValue;
    }

    if(state.angularSpeed > angularSaturationValue){
        state.angularSpeed = angularSaturationValue;
    }
    else if(state.angularSpeed < -angularSaturationValue){
        state.angularSpeed = -angularSaturationValue;
    }
}

void MainWindow::evaluateAngleRamp(float targetangle){

    float diff = targetangle * regulatorAngularProportionalElement - state.angularSpeed;
    int dir = (diff > 0) ? 1 : -1;

    float increment = rampAngularConstant < std::fabs(diff) ? rampAngularConstant : std::fabs(diff);
    state.angularSpeed += dir * increment;
}


void MainWindow::on_btnRegulation_clicked()
{
    toogleRegulationButton();
}

bool MainWindow::checkIfPointIsInRobotsWay(Point destPoint){

//    cout << "checking for point: x: " << destPoint.x << " ,y: " << destPoint.y << endl;
    float dx = destPoint.x - state.x;
    float dy = destPoint.y - state.y;
    float distance = std::sqrt(dx*dx + dy*dy);
//    float distance = std::sqrt(pow(destPoint.x - state.x, 2) + pow(destPoint.y - state.y, 2));
    float angle = std::atan2(dy, dx) - state.angle; // uhol medzi robotom a cielom

//    cout << angle << endl << endl;

    for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
    {
        if(!(copyOfLaserData.Data[k].scanDistance > 130 && copyOfLaserData.Data[k].scanDistance < 3000 && !(copyOfLaserData.Data[k].scanDistance < 640 && copyOfLaserData.Data[k].scanDistance > 700)))continue; // vyhod zle data

        float checkingAngle = ((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0)-angle;
        checkingAngle=checkingAngle>3.14159265358979 ? checkingAngle-2*3.14159265358979:checkingAngle<-3.14159265358979?checkingAngle+2*3.14159265358979:checkingAngle;
//        cout << angle << " < " << state.angle << endl;
        if(abs(checkingAngle ) > 3.14159265358979/2)continue;
        float b = (copyOfLaserData.Data[k].scanDistance/10)*sin(checkingAngle);
//        cout << "b: " << b << " checking angle: " << checkingAngle << endl;
        if(abs(b) < robotZone/2){
            if(copyOfLaserData.Data[k].scanDistance < distance*1000){
//                cout << "distance: " << copyOfLaserData.Data[k].scanDistance << " < " << distance*1000 << endl;
                indexOfBlockingPoint = k;
                return true;
            }
        }
    }
    return false;
}


void MainWindow::printMap(){
    cout << endl << "MAP:" << endl;
    for (int i = 0; i < map.size(); i++) {
        for (int j = 0; j < map[i].size(); j++) {
            cout << map[i][j] << " ";
        }
        cout << endl;
    }
}

void MainWindow::save_map() {
    ofstream outfile("mapa.txt", ios::trunc);

    if (!outfile) {
        cerr << "Error: Could not open file " << "mapa.txt" << endl;
        return;
    }

    for (const auto& row : map) {
        for (int val : row) {
            outfile << val << " ";
        }
        outfile << endl;
    }

    outfile.close();
}

void MainWindow::mapResize(){
   map.resize(rows);
   for (auto& row : map) {
       row.resize(cols, 0);  // initialize the new columns with zeros
   }
}

void MainWindow::loadMap(){

    ifstream file("mapa.txt");

    if (file.is_open()) {
        string line;
        while (getline(file, line)) {
            vector<int> row;
            for (char c : line) {
                if (c == '0') {
                    row.push_back(0);
                } else if (c == '1') {
                    row.push_back(1);
                }
            }
            map.push_back(row);
        }
        file.close();
    } else {
        cout << "Unable to open file";
        return;
    }
}

void MainWindow::mainLogigOfU4(){

    // Set -1 to tile where robot is located
    int row = state.y*1000 / gridSizeBlockInMM;
    int col = state.x*1000 / gridSizeBlockInMM;
    map[row][col] = -1;

    // Set a random destination
        srand(time(NULL));
        int rand_row = rand() % map.size();
        int rand_col = rand() % map[0].size();

        while (map[rand_row][rand_col] != 0) {
            rand_row = rand() % map.size();
            rand_col = rand() % map[0].size();
        }

        map[rand_row][rand_col] = 2;

//    map[10][10] = 2;

    printMap();
    // Run flood fill from the destination to the robot
    floodFill(map, rand_row, rand_col);

    printMap();


    Direction direction;
    int oldValue = 999;
    while(true){

        direction = checkDirection(row, col);

        while(true){
            switch (direction){
            case LEFT:
                col--;
                break;
            case RIGHT:
                col++;
                break;
            case UP:
                row--;
                break;
            case DOWN:
                row++;
                break;
            }
            if(map[row][col] < oldValue && map[row][col] != 1 && map[row][col] != 0){
                oldValue = map[row][col];
                cout << oldValue << endl;
            }else break;
        }

        switch (direction){
        case LEFT:
            col++;
            break;
        case RIGHT:
            col--;
            break;
        case UP:
            row++;
            break;
        case DOWN:
            row--;
            break;
        }

        pointsModel->push_back(Point{col*gridSizeBlockInMM/1000.0f, row*gridSizeBlockInMM/1000.0f});


        if(oldValue == 2){
            break;
        }
    }
}

void MainWindow::floodFill(vector<vector<int>>& map, int row, int col) {

    int val = 2;
    queue<Node> q;
    q.push(Node(row, col, val));

    while (!q.empty()) {
        Node node = q.front();
        q.pop();

        row = node.row;
        col = node.col;
        val = node.val;

        if (row < 0 || row >= map.size() || col < 0 || col >= map[0].size()) {
            continue;
        }

        if (map[row][col] == -1) {
            break;
        }

        if (map[row][col] != 0 && map[row][col] != 2) {
            continue;
        }
        if (map[row][col] != 2) {
            map[row][col] = val;
        }
//        cout << "inserting to: " << row  << " " << col << " with value: " << val << endl;
        q.push(Node(row-1, col, val+1));
        q.push(Node(row+1, col, val+1));
        q.push(Node(row, col-1, val+1));
        q.push(Node(row, col+1, val+1));
    }
}

Direction MainWindow::checkDirection(int workingRow, int workingCollumn){

    bool left = true;
    bool right = true;
    bool up = true;
    bool down = true;

    int leftValue = map[workingRow][workingCollumn-1];
    int rightValue = map[workingRow][workingCollumn+1];
    int upValue = map[workingRow-1][workingCollumn];
    int downValue = map[workingRow+1][workingCollumn];

    if(downValue == 1 || downValue == 0 || downValue == -1)down = false;
    if(upValue == 1 || upValue == 0 || upValue == -1)up = false;
    if(rightValue == 1 || rightValue == 0 || rightValue == -1)right = false;
    if(leftValue == 1 || leftValue == 0 || leftValue == -1)left = false;

    int smallest;
    if(left)smallest = leftValue;
    else if(right)smallest = rightValue;
    else if(up)smallest = upValue;
    else smallest = downValue;

    if (right && rightValue < smallest) {
        smallest = rightValue;
    }
    if (up && upValue < smallest) {
        smallest = upValue;
    }
    if (down && downValue < smallest) {
        smallest = downValue;
    }

    if (smallest == leftValue) {
        return LEFT;
    } else if (smallest == rightValue) {
        return RIGHT;
    } else if (smallest == upValue) {
        return UP;
    } else {
        return DOWN;
    }
}

void MainWindow::expandWalls(){
    // Define the expansion factor
    int expansionFactor = 3;

    // Create a new 2D vector to store the expanded map
    vector<vector<int>> expandedMap(map.size(), vector<int>(map[0].size(), 0));

    // Iterate over all cells in the map
    for (int i = 0; i < map.size(); i++) {
        for (int j = 0; j < map[0].size(); j++) {
            // Check if the cell represents a wall
            if (map[i][j] == 1) {
                // Replace the wall cell with a larger block of wall cells
                for (int k = i-expansionFactor; k <= i+expansionFactor; k++) {
                    for (int l = j-expansionFactor; l <= j+expansionFactor; l++) {
                        // Check if the cell is within the bounds of the map
                        if (k >= 0 && k < map.size() && l >= 0 && l < map[0].size()) {
                            // Replace the cell with a wall
                            expandedMap[k][l] = 1;
                        }
                    }
                }
            } else {
                // Copy the cell from the original map
                expandedMap[i][j] = map[i][j];
            }
        }
    }

    // Use the expanded map for further processing
    map = expandedMap;

}


void MainWindow::on_pushButton_7_clicked()
{
    pointsModel->clear();
}

void MainWindow::doConvolution(){
    convolutionResults.clear();
    float threshold = checkLineEdit(ui->lineEdit);
    float threshold2 = checkLineEdit(ui->lineEdit_5);
    float threshold3 = checkLineEdit(ui->lineEdit_6);
    int pointShift = 2;

//    double one = copyOfLaserData.Data[0].scanDistance;
//    double two = copyOfLaserData.Data[1].scanDistance;
//    for(int k=3; k<copyOfLaserData.numberOfScans/*360*/; k++)
//    {
//        if(one > two < copyOfLaserData.Data[k].scanDistance)
//            convolutionResults.push_back(k);
//        else if(one < two > copyOfLaserData.Data[k].scanDistance)
//            convolutionResults.push_back(k);
//        two = one;
//        one = copyOfLaserData.Data[k].scanDistance;
    float robotZoneInM = robotZone/100.0;
    for(int k=pointShift;k<copyOfLaserData.numberOfScans-pointShift/*360*/;k++)
    {
        if(!(copyOfLaserData.Data[k].scanDistance > 130 && copyOfLaserData.Data[k].scanDistance < 3000 && !(copyOfLaserData.Data[k].scanDistance < 640 && copyOfLaserData.Data[k].scanDistance > 700)))continue; // vyhod zle data
        if(fabs(copyOfLaserData.Data[k].scanDistance-0.5*copyOfLaserData.Data[k-1].scanDistance-0.5*copyOfLaserData.Data[k+1].scanDistance) > threshold/pow((threshold3/copyOfLaserData.Data[k].scanDistance), 1.5)){

            // vypocet suradnic dvoch bodoh na stranach rohu a rohu
            float x1 = state.x + copyOfLaserData.Data[k-pointShift].scanDistance/1000*cos(state.angle+(360-copyOfLaserData.Data[k-pointShift].scanAngle)*3.14159/180.0);
            float y1 = state.y + copyOfLaserData.Data[k-pointShift].scanDistance/1000*sin(state.angle+(360-copyOfLaserData.Data[k-pointShift].scanAngle)*3.14159/180.0);
            float x3 = state.x + copyOfLaserData.Data[k+pointShift].scanDistance/1000*cos(state.angle+(360-copyOfLaserData.Data[k+pointShift].scanAngle)*3.14159/180.0);
            float y3 = state.y + copyOfLaserData.Data[k+pointShift].scanDistance/1000*sin(state.angle+(360-copyOfLaserData.Data[k+pointShift].scanAngle)*3.14159/180.0);
            float x2 = state.x + copyOfLaserData.Data[k].scanDistance/1000*cos(state.angle+(360-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0);
            float y2 = state.y + copyOfLaserData.Data[k].scanDistance/1000*sin(state.angle+(360-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0);

            // vypocet suradnice osi rohu
            float mid_x = (x1 + x3) / 2;
            float mid_y = (y1 + y3) / 2;


//            float diff = sqrt(pow((x3 - x1),2) + pow((y3 - y1),2));
            float diff_p1_x = mid_x - x1;
            float diff_p1_y = mid_y - y1;
            float diff_p3_x = mid_x - x3;
            float diff_p3_y = mid_y - y3;

            // zapametanie si znamienka pre jednotlive suradnice a body
            int signum_p1_x = diff_p1_x < 0 ? -1 : 1;
            int signum_p1_y = diff_p1_y < 0 ? -1 : 1;
            int signum_p3_x = diff_p3_x < 0 ? -1 : 1;
            int signum_p3_y = diff_p3_y < 0 ? -1 : 1;

            // dat vvsetky rozdiely na rovnake znamienko
            diff_p1_x = fabs(diff_p1_x);
            diff_p1_y = fabs(diff_p1_y);
            diff_p3_x = fabs(diff_p3_x);
            diff_p3_y = fabs(diff_p3_y);

            // vypocitat a zistit, ci sa robot nachadza blizsie k osi rohu ako k samotnemu rohu
            float diff_robot_midpoint = sqrt(pow((state.x - mid_x),2) + pow((state.y - mid_y),2));

            // musi to byt fakovy roh, neexsituje ze by mohlo byt toto tak blizko
            if(sqrt(pow((x2 - mid_x),2) + pow((y2 - mid_y),2)) < threshold2)continue;

            int signum = copyOfLaserData.Data[k].scanDistance/1000 > diff_robot_midpoint ? 1 : -1;

            // vsetky rozdiely dat na -1 ak je robot z druhej strany
            diff_p1_x *= signum;
            diff_p1_y *= signum;
            diff_p3_x *= signum;
            diff_p3_y *= signum;

            // zistenie, ktora strana je viac x-ova a ktora y-lonova
            if(fabs(x1 - x2) < fabs(y1 - y2)){ // Viac stena suradnice - y

                // vytvorenie x suradnice pre robota
                if(diff_p1_x < robotZoneInM){
                    mid_x += (robotZoneInM - diff_p1_x*signum)*signum_p1_x*signum;
                }

                // vytvorenie y suradnice pre robota
                if(diff_p3_y < robotZoneInM){
                    mid_y += (robotZoneInM - diff_p3_y*signum)*signum_p3_y*signum;
                }

            }else{ // Viac stena suradnice - x

                // vytvorenie y suradnice pre robota
                if(diff_p1_y < robotZoneInM){
                    mid_y += (robotZoneInM - diff_p1_y*signum)*signum_p1_y*signum;
                }

                // vytvorenie x suradnice pre robota
                if(diff_p3_x < robotZoneInM){
                    mid_x += (robotZoneInM - diff_p3_x*signum)*signum_p3_x*signum;
                }

            }


            ///////////////////////////////////////////////////////////
//            float diff_x = mid_x - state.x;
//            float diff_y = mid_y - state.y;

//            if(diff_x < diff_y && diff_x < robotZoneInM){
//                mid_x += diff_x - robotZoneInM*diff_x<0?-1:1;
//                mid_y
//            }

//            float smaller_diff = diff_x < diff_y ? diff_x : diff_y;

//            if(smaller_diff < robotZoneInM){
//                mid_x += smaller_diff - robotZoneInM;
//                mid_y += smaller_diff - robotZoneInM;
//            }

            convolutionResults.push_back(ConvolutionPoint{k, Point{mid_x, mid_y}});
            k+=4;
        }

    }
    CheckForNewCorners();
}

void MainWindow::CheckForNewCorners(){
    for (const auto& p1 : convolutionResults) {
        bool found = false;
        float distance;
        for (const auto& p2 : AllCornersPoints) {
          distance = std::hypot(p1.point.x - p2.x, p1.point.y - p2.y);
          if (distance < cornerPointsThreshold) {
            found = true;
            break;
          }
        }

        if (!found) {
          cout << "new corner found, x: " << p1.point.x << ", y: " << p1.point.y << ", distance: " << distance << endl;
          AllCornersPoints.push_back(p1.point);
          pointsModel->push_back(p1.point);
        }
    }
}

void MainWindow::on_pushButton_10_clicked()
{
    wallFolower = !wallFolower;
    toogleRegulationButton();
    cout << "switch" << endl;
}

