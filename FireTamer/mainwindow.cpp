/****************************************************************************
**
** Copyright (C) 2012 Denis Shienkov <denis.shienkov@gmail.com>
** Copyright (C) 2012 Laszlo Papp <lpapp@kde.org>
** Contact: http://www.qt-project.org/legal
**
** This file is part of the QtSerialPort module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL21$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and Digia. For licensing terms and
** conditions see http://qt.digia.com/licensing. For further information
** use the contact form at http://qt.digia.com/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 or version 3 as published by the Free
** Software Foundation and appearing in the file LICENSE.LGPLv21 and
** LICENSE.LGPLv3 included in the packaging of this file. Please review the
** following information to ensure the GNU Lesser General Public License
** requirements will be met: https://www.gnu.org/licenses/lgpl.html and
** http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** In addition, as a special exception, Digia gives you certain additional
** rights. These rights are described in the Digia Qt LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "settingsdialog.h"
#include <QMessageBox>
#include <QtSerialPort/QSerialPort>
#include <QDateTime>
#include "rexuscontrol.h"
#include <QFile>
#include <QMediaPlayer>

QString logString = "";
QString tempString ="";
bool MainValveByte = false;
QByteArray receivedArray;
bool receivingString = false;
float oldHighPressure = 0;
float newHighPressure = 0;
int oldTime = 0;
int newTime = 1;
float lmin = 0;
int testval = 0;
int LOsaid = 0;
int SOEsaid = 0;
int SODSsaid = 0;


bool DEBUG = true;
//! [0]
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    //! [0]
    ui->setupUi(this);

    //! [1]
    serial = new QSerialPort(this);
    //! [1]
    settings = new SettingsDialog;
    ui->actionConnect->setEnabled(true);
    ui->actionDisconnect->setEnabled(false);
    ui->actionQuit->setEnabled(true);
    ui->actionConfigure->setEnabled(true);

    initActionsConnections();

    connect(serial, SIGNAL(error(QSerialPort::SerialPortError)), this,
            SLOT(handleError(QSerialPort::SerialPortError)));

    //! [2]
    connect(serial, SIGNAL(readyRead()), this, SLOT(readData()));
    //! [2]

    //! [3]
    rexuscontrol = new RexusControl(this);
}
//! [3]

MainWindow::~MainWindow()
{
    delete settings;
    delete ui;
}

//! [4]
void MainWindow::openSerialPort()
{
    SettingsDialog::Settings p = settings->settings();
    serial->setPortName(p.name);
    serial->setBaudRate(p.baudRate);
    serial->setDataBits(p.dataBits);
    serial->setParity(p.parity);
    serial->setStopBits(p.stopBits);
    serial->setFlowControl(p.flowControl);
    if (serial->open(QIODevice::ReadWrite)) {
        ui->actionConnect->setEnabled(false);
        ui->actionDisconnect->setEnabled(true);
        ui->actionConfigure->setEnabled(false);
        ui->statusBar->showMessage(tr("Connected to %1 : %2, %3, %4, %5, %6")
                                   .arg(p.name).arg(p.stringBaudRate).arg(p.stringDataBits)
                                   .arg(p.stringParity).arg(p.stringStopBits).arg(p.stringFlowControl));
    } else {
        QMessageBox::critical(this, tr("Error"), serial->errorString());

        ui->statusBar->showMessage(tr("Open error"));
    }
}
//! [4]

//! [5]
void MainWindow::closeSerialPort()
{
    serial->close();
    ui->actionConnect->setEnabled(true);
    ui->actionDisconnect->setEnabled(false);
    ui->actionConfigure->setEnabled(true);
    ui->statusBar->showMessage(tr("Disconnected"));
}
//! [5]

void MainWindow::about()
{
    QMessageBox::about(this, tr("About FireTamer"),
                       tr("FireTamer is the control program for the UB-FIRE REXUS"
                          "team as used on the REXUS 20 rocket in 2016 "
                          "For more infos visit our website at http://ub-fire.de"));
}

//! [6]
void MainWindow::writeData(const QByteArray &data)
{
    serial->write(data);
}
//! [6]

//! [7]
void MainWindow::readData()
{
    while (serial->bytesAvailable()!=0){
       QByteArray data = serial->read(1); //receive one character
       if (ui->checkBox_logStream->isChecked()){
           QFile file("data.txt");
           file.open(QIODevice::Append);
           file.write(data);
           file.close();
       }

       //qDebug(data);
        if (data.contains("$")){
            receivedArray.append(data);
        }
        if (receivedArray.contains("$")){
            if (data.contains(";")){
                receivedArray.append(data);

                int command = receivedArray.mid(1, 2).toInt();
                int dataLength = receivedArray.length() -5;
                QByteArray dataValue = receivedArray.mid(4, dataLength);
                 processCommand(command , dataValue);
                receivedArray.clear();
            } else {
                if (data.contains("$") ==0){
                receivedArray.append(data);
            }}
        }
    }
}
void MainWindow::handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::ResourceError) {
        QMessageBox::critical(this, tr("Critical Error"), serial->errorString());
        closeSerialPort();
    }
}
void MainWindow::processCommand(int command, QByteArray data){

    switch (command){
    case 99:{ //Text
        addToLogBox(data);
        break;
    }
    case 98:{ //Error Message
        ErrAddToLogBox(data);
        this->ui->ErrorLED->setPixmap(QPixmap(":/images/ledred.png") );
        break;
    }
    case 1:{ //Status
        bool bits[8]={0,0,0,0,0,0,0,0};
        for (int i = 0; i < 8; i++) {
            bits[i] = (data.toInt() >> i) & 1; //convert the incoming data String to bit values
        }
        if (bits[7]==1){
            this->ui->armedModeLED->setPixmap(QPixmap(":/images/ledgreen.png") );
            }else{
            if (bits[3]==1){
                this->ui->armedModeLED->setPixmap(QPixmap(":/images/ledorange.png") );
                }else{
                this->ui->armedModeLED->setPixmap(QPixmap(":/images/ledoff.png") );
                }
            }
        if (bits[6]==1){
            this->ui->batteryOkLED->setPixmap(QPixmap(":/images/ledgreen.png") );
            }else{
            this->ui->batteryOkLED->setPixmap(QPixmap(":/images/ledred.png") );
            }
        if (bits[5]==1){
            this->ui->sdOkLED->setPixmap(QPixmap(":/images/ledgreen.png") );
            }else{
            this->ui->sdOkLED->setPixmap(QPixmap(":/images/ledred.png") );
            }
        if (bits[4]==1){
            this->ui->readyLED->setPixmap(QPixmap(":/images/ledgreen.png") );
            }else{
            this->ui->readyLED->setPixmap(QPixmap(":/images/ledred.png") );
            }
        if (bits[2]==1){
            this->ui->loLED->setPixmap(QPixmap(":/images/ledgreen.png") );

            if (!LOsaid){
                QMediaPlayer* player = new QMediaPlayer;
                player->setMedia(QUrl("qrc:/sounds/LO.mp3"));
                player->setVolume(100);
                player->play();
                addToLogBox("Lift off received!");
                LOsaid = 1;

               // QTime timeSinceLO = QTime::currentTime();
              //  QString timeSinceLOString = timeSinceLO.secsTo();

            }

            }else{
            this->ui->loLED->setPixmap(QPixmap(":/images/ledoff.png") );
            }
        if (bits[1]==1){
            this->ui->soeLED->setPixmap(QPixmap(":/images/ledgreen.png") );
            if (!SOEsaid){
                addToLogBox("SOE received!");
                SOEsaid = 1;
            }
            }else{
            this->ui->soeLED->setPixmap(QPixmap(":/images/ledoff.png") );
            }
        if (bits[0]==1){
            this->ui->sodsLED->setPixmap(QPixmap(":/images/ledgreen.png") );
            if (!SODSsaid){
                addToLogBox("SODS received!");
                SODSsaid = 1;
            }
            }else{
            this->ui->sodsLED->setPixmap(QPixmap(":/images/ledoff.png") );
            }
    }break;
    case 2: //Input Voltage
        this->ui->inputVoltageLCD->display((data.toFloat()/1000));
        break;
    case 3: //Battery Voltage
        this->ui->batteryVoltageLCD->display(data.toFloat()/1000);
        break;
    case 4: //Paket ID
        this->ui->paketIDLCD->display(data.toInt());
        break;
    case 5: //Time since Power On
        this->ui->poLCD->display(data.toInt());

        break;
    case 10: //Module Temperature
        this->ui->moduleTempLCD->display(data.toInt());
        break;
    case 11: //Chamber 1 Temperature
        this->ui->ch1TempLCD->display(data.toInt()-273);
        break;
    case 12: //Chamber 2 Temperature
        this->ui->ch2TempLCD->display(data.toFloat()-273);
        break;
    case 13: //Chamber 3 Temperature
        this->ui->ch3TempLCD->display(data.toFloat()-273);
        break;
    case 14: //Chamber 4 Temperature
        this->ui->ch4TempLCD->display(data.toFloat()-273);
        break;
    case 15: //Chamber 5 Temperature
        this->ui->ch5TempLCD->display(data.toFloat()-273);
        break;
    case 16: //NTC1 Temperature
       this->ui->ntc1LCD->display(data.toFloat()-273);
        break;
    case 20: //Module Pressure
        this->ui->modulePressLCD->display(data.toInt());
        break;
    case 21: //Chamber1 Pressure
        this->ui->ch1PressLCD->display(data.toInt());
        break;
    case 22: //Chamber2 Pressure
        this->ui->ch2PressLCD->display(data.toInt());
        break;
    case 23: //Chamber3 Pressure
        this->ui->ch3PressLCD->display(data.toInt());
        break;
    case 24: //Chamber4 Pressure
        this->ui->ch4PressLCD->display(data.toInt());
        break;
    case 25: //Chamber5 Pressure
        this->ui->ch5PressLCD->display(data.toInt());
        break;
    case 26: //High Pressure
       this->ui->highPressLCD->display((data.toFloat())/10);
        break;
    case 27: //Mid Pressure
        this->ui->midPressLCD->display(data.toFloat()/1000);
        break;
    case 30: //Facing Sample
        this->ui->facingSampleLCD->display(data.toInt());
        break;
    case 31: //Motor Powered
        if (data.toInt() == 1){
        this->ui->motorPoweredLED->setPixmap(QPixmap(":/images/ledgreen.png") );
        }else{
        this->ui->motorPoweredLED->setPixmap(QPixmap(":/images/ledoff.png") );
        }
    case 32: //Turntable locked

        if (data.toInt() == 1){
        this->ui->tableLockedLCD->setPixmap(QPixmap(":/images/ledgreen.png") );
        }else{
        this->ui->tableLockedLCD->setPixmap(QPixmap(":/images/ledoff.png") );
        }
    case 33:{ //HP Outputs
        bool bits[8]={0,0,0,0,0,0,0,0};
        for (int i = 0; i < 8; i++) {
            bits[i] = (data.toInt() >> i) & 1; //convert the incoming data String to bit values
        }
        if (bits[3]==1){
            this->ui->relay1LED->setPixmap(QPixmap(":/images/ledgreen.png") );
            }else{
            this->ui->relay1LED->setPixmap(QPixmap(":/images/ledoff.png") );
            }
        if (bits[4]==1){
            this->ui->relay2LED->setPixmap(QPixmap(":/images/ledgreen.png") );
            }else{
            this->ui->relay2LED->setPixmap(QPixmap(":/images/ledoff.png") );
            }
        if (bits[5]==1){
            this->ui->relay3LED->setPixmap(QPixmap(":/images/ledgreen.png") );
            }else{
            this->ui->relay3LED->setPixmap(QPixmap(":/images/ledoff.png") );
            }
        if (bits[6]==1){
            this->ui->relay4LED->setPixmap(QPixmap(":/images/ledgreen.png") );
            }else{
            this->ui->relay4LED->setPixmap(QPixmap(":/images/ledoff.png") );
            }
        if (bits[7]==1){
            this->ui->relay5LED->setPixmap(QPixmap(":/images/ledgreen.png") );
            }else{
            this->ui->relay5LED->setPixmap(QPixmap(":/images/ledoff.png") );
            }
    }

    case 34:{ //Switchboard
        bool bits[8]={0,0,0,0,0,0,0,0};
        for (int i = 0; i < 8; i++) {
            bits[i] = (data.toInt() >> i) & 1; //convert the incoming data String to bit values
        }
        if (bits[5]==1){
            this->ui->mainValveLED->setPixmap(QPixmap(":/images/ledgreen.png") );
            }else{
            this->ui->mainValveLED->setPixmap(QPixmap(":/images/ledoff.png") );
            }
        if (bits[4]==1){
            this->ui->chambersLED->setPixmap(QPixmap(":/images/ledgreen.png") );
            }else{
            this->ui->chambersLED->setPixmap(QPixmap(":/images/ledoff.png") );
            }

        if (bits[3]==1){
            this->ui->closeLED->setPixmap(QPixmap(":/images/ledgreen.png") );
            }else{
            this->ui->closeLED->setPixmap(QPixmap(":/images/ledoff.png") );
            }
        if (bits[2]==1){
            this->ui->evacLED->setPixmap(QPixmap(":/images/ledgreen.png") );
            }else{
            this->ui->evacLED->setPixmap(QPixmap(":/images/ledoff.png") );
            }
    }
    case 36: //Camera Powered
        if (data.toInt() == 1){
        this->ui->cameraPoweredLED->setPixmap(QPixmap(":/images/ledgreen.png") );
        }else{
        this->ui->cameraPoweredLED->setPixmap(QPixmap(":/images/ledoff.png") );
        }

    case 37: //Camera Recording
        if (data.toInt() == 1){
        this->ui->cameraRecordingLED->setPixmap(QPixmap(":/images/ledgreen.png") );
        }else{
        this->ui->cameraRecordingLED->setPixmap(QPixmap(":/images/ledoff.png") );
        }
    }
}

void MainWindow::switchMainValve(){
if (MainValveByte){

  writeData("0*");
   qDebug("Click 0");

   MainValveByte = 0;
}
else if (!MainValveByte){
writeData("1*");
   qDebug("Click 1");

   MainValveByte = 1;
}
}

void MainWindow::switchEvacValve1(){
    writeData("2*");
}
void MainWindow::switchEvacValve2(){
writeData("3*");
}
void MainWindow::switchEvacValve3(){
writeData("4*");
}
//! [8]
void MainWindow::doThis()
{
    QMessageBox::about(this, tr("About FireTamer"),
                       tr("Roffelkartoffel"));
}

void MainWindow::clearLogBox()
{
    ui->logbox->clear();
    logString = "";
}

void MainWindow::setArmed(){
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.setText("Do you really want to ARM the experiment?");
        msgBox.setWindowTitle("Set experiment to ARMED MODE.");
        msgBox.setInformativeText("Please make sure everything is set up and ready to go!");
        msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
        msgBox.setDefaultButton(QMessageBox::No);
        int ret = msgBox.exec();
        switch (ret) {
        case QMessageBox::Yes:
            expMode(2);
            break;
        case QMessageBox::No:
            break;
        default:
            // should never be reached
            break;
        }
 }

void MainWindow::setDisarmed(){
    expMode(0);
}
void MainWindow::setTest(){
     expMode(1);
}

void MainWindow::expMode(int state){
    switch(state){
        case 0:{
        serial->write("50*"); //disarming experiment
        break;
    }
    case 1:{
        serial->write("51*"); //putting to Testmode
        break;
    }
    case 2:
        serial->write("52*"); //Arming Experiment
        break;
    }
}
void MainWindow::sendHome(){
    serial->write("57*");  //send Home
}
void MainWindow::CameraON(){
    serial->write("58*");   //Camera On
}
void MainWindow::CameraOFF(){
    serial->write("59*");   //Camera OFF
}

void MainWindow::addToLogBox(QString text){

    QTime time = QTime::currentTime();
    QString timeString = time.toString();
    logString =timeString + ": " + text + "\n" + logString;
    ui->logbox->setPlainText(logString);
}
void MainWindow::ErrAddToLogBox(QString text){

    QTime time = QTime::currentTime();
    QString timeString = time.toString();
    logString =timeString + ": " + "[ERROR]" + text + "\n" + logString;
    ui->logbox->setPlainText(logString);
}
void MainWindow::ResetExp()
{


serial->write("70*");
}
void MainWindow::ClearError(){
     this->ui->ErrorLED->setPixmap(QPixmap(":/images/ledoff.png") );
}

void MainWindow::sayReady(){
    QMediaPlayer* player = new QMediaPlayer;
    player->setMedia(QUrl("qrc:/sounds/UB-Ready.mp3"));
    player->setVolume(100);
    player->play();
}

void MainWindow::initActionsConnections()
{
    connect(ui->actionConnect, SIGNAL(triggered()), this, SLOT(openSerialPort()));
    connect(ui->actionDisconnect, SIGNAL(triggered()), this, SLOT(closeSerialPort()));
    connect(ui->actionQuit, SIGNAL(triggered()), this, SLOT(close()));
    connect(ui->actionConfigure, SIGNAL(triggered()), settings, SLOT(show()));
    connect(ui->actionClear, SIGNAL(triggered()), this, SLOT(clearLogBox()));
    connect(ui->actionAbout, SIGNAL(triggered()), this, SLOT(about()));
    connect(ui->actionAboutQt, SIGNAL(triggered()), qApp, SLOT(aboutQt()));
    connect(ui->actionArm_Experiment, SIGNAL(triggered()), this, SLOT(setArmed()));
    connect(ui->actionTest_Expirement, SIGNAL(triggered()), this, SLOT(setTest()));
    connect(ui->actionDisarm_Experiment, SIGNAL(triggered()), this, SLOT(setDisarmed()));
    connect(ui->actionResetExp, SIGNAL(triggered()), this, SLOT(ResetExp()));
    connect(ui->mainValveButton,SIGNAL(clicked()),this, SLOT(switchMainValve()));
    connect(ui->closeValveButton,SIGNAL(clicked()),this, SLOT(switchEvacValve2()));
    connect(ui->chambersValveButton,SIGNAL(clicked()),this, SLOT(switchEvacValve1()));
    connect(ui->evacValvebutton,SIGNAL(clicked()),this, SLOT(switchEvacValve3()));
    connect(ui->homeButton,SIGNAL(clicked()),this, SLOT(sendHome()));
    connect(ui->CameraONButton,SIGNAL(clicked()),this, SLOT(CameraON()));
    connect(ui->CameraOFFButton,SIGNAL(clicked()),this, SLOT(CameraOFF()));
    connect(ui->ClearErrorButton,SIGNAL(clicked()),this, SLOT(ClearError()));
     connect(ui->readyButton,SIGNAL(clicked()),this, SLOT(sayReady()));

}


