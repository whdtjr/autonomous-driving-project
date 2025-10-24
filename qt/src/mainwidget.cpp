#include "mainwidget.h"
#include "ui_mainwidget.h"
#include <QPixmap>
#include <QGraphicsScene>
#include <QHBoxLayout>
#include "onvif_browser_widget.h"

MainWidget::MainWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::MainWidget)
{
    ui->setupUi(this);

    // Set up socket client
    pSocketClient = new SocketClient(this);
    pSocketClient->connectToServerSlot(bFlag);
    connect(pSocketClient, SIGNAL(socketRecvDataSig(QString)), this, SLOT(updateRecvDataSlot(QString)));

    // Setup traffic light images
    light_images[0] = QPixmap(":/images/green.png");
    light_images[1] = QPixmap(":/images/yellow.png");
    light_images[2] = QPixmap(":/images/red.png");
    ui->lightLabel->setPixmap(light_images[state_led]);

    // Replace the left camera QLabel area with the ONVIF Browser widget
    // The UI has a QHBoxLayout named "horizontalLayout" containing cameraLabel on the left.
    if (ui->horizontalLayout && ui->cameraLabel) {
        int idx = ui->horizontalLayout->indexOf(ui->cameraLabel);
        if (idx >= 0) {
            ui->horizontalLayout->removeWidget(ui->cameraLabel);
            ui->cameraLabel->deleteLater();
            onvifWidget_ = new OnvifBrowserWidget(this);
            ui->horizontalLayout->insertWidget(idx, onvifWidget_, /*stretch*/7);
        }
    }
}

MainWidget::~MainWidget()
{
    delete ui;
}

void MainWidget::on_pushButton_clicked()
{
    state_led = (state_led+1) % 3;      // 0 -> 1 -> 2 -> 0 ...
    ui->lightLabel->setPixmap(light_images[state_led]);
    ui->timeLabel->setText("00:10");
}

void MainWidget::updateRecvDataSlot(QString strRecvData)
{
    strRecvData.chop(1);    // remove trailing \n
    qDebug() << ">>Recv Data : " << strRecvData;

    // [JAB_QT]LED@0xff => @JAB_QT@LED@0xff
    strRecvData.replace("[","@");
    strRecvData.replace("]","@");
    QStringList strList = strRecvData.split("@");

    if (strList.size() <= 3) {
        qWarning() << "Invalid message format, parts:" << strList;
        return;
    }

    if(strList.size() >= 5)
    {
        if(strList[4].indexOf("GREEN") == 0)    state_led = 0;
        if(strList[4].indexOf("YELLOW") == 0)   state_led = 1;
        if(strList[4].indexOf("RED") == 0)      state_led = 2;
        if(strList.size() > 5 && strList[5].indexOf("Z1") == 0)   ui->zoneLabel->setText("Zone 1");
    }

    ui->lightLabel->setPixmap(light_images[state_led]);
    if (strList.size() > 3) ui->timeLabel->setText(strList[3]);
}

void MainWidget::socketSendToLinux(int keyNo)
{
    Q_UNUSED(keyNo);
}

SocketClient* MainWidget::getpSocketClient()
{
    return pSocketClient;
}

