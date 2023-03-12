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
    ipaddress="127.0.0.1";//192.168.1.14toto je na niektory realny robot.. na lokal budete davat "127.0.0.1"
  //  cap.open("http://192.168.1.11:8000/stream.mjpg");
    ui->setupUi(this);
    datacounter=0;
  //  timer = new QTimer(this);
//    connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
    actIndex=-1;
    useCamera1=false;

    pointsModel = new PointTableModel();
    ui->tbPoints->setModel(pointsModel);

//    pointsModel->push_back(Point{1, 0});
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


    std::vector<PointI> wall_endpoints;
    PointI prev_point;
    PointI cur_point;
    PointI first_point;
    float distance;


    if(useCamera1==true && actIndex>-1)/// ak zobrazujem data z kamery a aspon niektory frame vo vectore je naplneny
    {
        std::cout<<actIndex<<std::endl;
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
            for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
            {
                int dist=copyOfLaserData.Data[k].scanDistance/20; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
                int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x(); //prepocet do obrazovky
                int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();//prepocet do obrazovky

                if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                {
                    pero.setColor(Qt::green);
                    painter.setPen(pero);
                    painter.drawEllipse(QPoint(xp, yp),2,2);
                }

                if(paintEventCounter % wall_edge_detect_frequenc == 0){
                        if(k == 0)first_point = prev_point = PointI{xp, yp};

                    cur_point = PointI{xp, yp};

                    distance = sqrt(pow(cur_point.x - prev_point.x, 2) + pow(cur_point.y - prev_point.y, 2));
//                    cout << "distance: " << distance << endl;
                    if (distance > robotZone*2)
                    {
//                        cout << "edge found! " << paintEventCounter << endl;
                        // Endpoint of an obstacle wall detected
                        wall_endpoints.push_back(prev_point);
                        wall_endpoints.push_back(cur_point);

                        if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                        {
                            pero.setColor(Qt::red);
                            painter.setPen(pero);
                            painter.drawEllipse(QPoint(xp, yp),2,2);
                        }
                        if(rect.contains(prev_point.x,prev_point.y))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                        {
                            pero.setColor(Qt::red);
                            painter.setPen(pero);
                            painter.drawEllipse(QPoint(prev_point.x, prev_point.y),2,2);
                        }
                    }

                    prev_point = cur_point;

                }
            }
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

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z lidaru
int MainWindow::processThisLidar(LaserMeasurement laserData)
{


    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru#

    // Uloha 2
    if(risingEdgeOfRegulating || (lidarDataCounter%20==0 && regulating)){
        risingEdgeOfRegulating = false;
        if(checkIfPointIsInRobotsWay()){
            cout << "There is a barier on the way to point!" << endl;
            pointsModel->pop_front();
            toogleRegulationButton();

        }
    }

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
        return;
    }

    state.forwardSpeed=0;
    state.angularSpeed=0;
    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    connect(this,SIGNAL(uiValuesChanged()),this,SLOT(setUiValues()));

    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robot.setLaserParameters(ipaddress,52999,5299,/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::processThisLidar,this,std::placeholders::_1));
    robot.setRobotParameters(ipaddress,53000,5300,std::bind(&MainWindow::processThisRobot,this,std::placeholders::_1));
    //---simulator ma port 8889, realny robot 8000
    robot.setCameraParameters("http://"+ipaddress+":8000/stream.mjpg",std::bind(&MainWindow::processThisCamera,this,std::placeholders::_1));

    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robot.robotStart();



    //ziskanie joystickov
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
    robot.setTranslationSpeed(500);

}

void MainWindow::on_pushButton_3_clicked() //back
{
    robot.setTranslationSpeed(-250);

}

void MainWindow::on_pushButton_6_clicked() //left
{
robot.setRotationSpeed(3.14159/2);

}

void MainWindow::on_pushButton_5_clicked()//right
{
robot.setRotationSpeed(-3.14159/2);

}

void MainWindow::on_pushButton_4_clicked() //stop
{
    stopRobot();

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
        cout << "!!!!!!!!!!!!!!!!!!!!!" << endl;
    } else if (deltaEncoderLeft < -ENCODER_MAX / 2) {
        deltaEncoderLeft += ENCODER_MAX;
        cout << "!!!!!!!!!!!!!!!!!!!!!" << endl;
    }
    if (deltaEncoderRight > ENCODER_MAX / 2) {
        deltaEncoderRight -= ENCODER_MAX;
        cout << "!!!!!!!!!!!!!!!!!!!!!" << endl;
    } else if (deltaEncoderRight < -ENCODER_MAX / 2) {
        deltaEncoderRight += ENCODER_MAX;
        cout << "!!!!!!!!!!!!!!!!!!!!!" << endl;
    }

    // calculate the distance traveled by each wheel
    long double distanceLeft = deltaEncoderLeft * tickToMeter;
    long double distanceRight = deltaEncoderRight * tickToMeter;
//    cout << "DistanceLeft: " << distanceLeft;
//    cout << " DistanceRight: " << distanceRight;

    // calculate the average distance traveled by the robot
    long double distance = (distanceLeft + distanceRight) / 2;

//    // calculate the change in angle of the robot
    long double deltaAngle = (distanceRight - distanceLeft) / wheelbase;

    if(distanceRight == distanceLeft){

    //    cout << " DeltaAngle: " << deltaAngle << endl;

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

    //
    // handle the possibility of the robot moving backwards
//    if (distance < 0) {
//        state.angle += 3.141592;
//        state.x += distance * cos(state.angle + deltaAngle / 2);
//        state.y += distance * sin(state.angle + deltaAngle / 2);
//    }

    oldEncoderLeft = encoderLeft;
    oldEncoderRight = encoderRight;

//    cout << "x: " << state.x << "; y: " << state.y << "; angle: " << state.angle << endl;
}



void MainWindow::on_pushButton_8_clicked()
{
    state.angle = 0;
    state.x = 0;
    state.y = 0;
}

void MainWindow::regulate(){
    cout << "forwarSpeed: " << state.forwardSpeed << " angular speed: " << state.angularSpeed << endl;

    if(pointsModel->rowCount() == 0){
        // There are no more points to go
        toogleRegulationButton();
        cout << "No points to go!" << endl;
        return;
    }
    Point destinationPoint = pointsModel->front();

    // Destination reached
    if((abs(state.x - destinationPoint.x) < 0.01) && (abs(state.y - destinationPoint.y) < 0.01)){
        pointsModel->pop_front();
        cout << "point x: " << destinationPoint.x << " y: " << destinationPoint.y << " reached!!" << endl;
    }

    // Calculate the distance and angle between the robot and the destination
    float dx = destinationPoint.x - state.x;
    float dy = destinationPoint.y - state.y;
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
        cout << "angle is too big!  " << angle << endl;
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

bool MainWindow::checkIfPointIsInRobotsWay(){

    Point destinationPoint = pointsModel->front();
    float dx = destinationPoint.x - state.x;
    float dy = destinationPoint.y - state.y;
    float distance = std::sqrt(dx*dx + dy*dy);
    float angle = std::atan2(dy, dx) - state.angle;

    cout << angle << endl << endl;

    for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
    {
        float checkingAngle = ((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0)-angle;
//        cout << angle << " < " << state.angle << endl;
        if(abs(checkingAngle - state.angle) > 3.14159265358979/2)continue;
        float b = (copyOfLaserData.Data[k].scanDistance/10)*sin(checkingAngle);
        cout << "b: " << b << " checking angle: " << checkingAngle << endl;
        if(abs(b) < robotZone/2){
            cout << "distance: " << copyOfLaserData.Data[k].scanDistance << " < " << distance*1000 << endl;
            if(copyOfLaserData.Data[k].scanDistance < distance*1000){
                return true;
            }
        }
    }
    return false;
}




