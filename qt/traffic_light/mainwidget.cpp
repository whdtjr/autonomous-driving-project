#include "mainwidget.h"
#include "ui_mainwidget.h"
#include <QPixmap>
#include <QGraphicsScene>

MainWidget::MainWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::MainWidget)
{
    ui->setupUi(this);

    pSocketClient = new SocketClient(this);
    pSocketClient->connectToServerSlot(bFlag);
    connect(pSocketClient, SIGNAL(socketRecvDataSig(QString)), this, SLOT(updateRecvDataSlot(QString)));

    light_images[0] = QPixmap(":/images/green.png");
    light_images[1] = QPixmap(":/images/yellow.png");
    light_images[2] = QPixmap(":/images/red.png");

    ui->lightLabel->setPixmap(light_images[state_led]);
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
    strRecvData.chop(1);    //끝 문자 1개 '\n' 제거

    qDebug() << ">>Recv Data : " << strRecvData;

    //[JAB_QT]LED@0xff => @JAB_QT@LED@0xff
    strRecvData.replace("[","@");
    strRecvData.replace("]","@");
    QStringList strList = strRecvData.split("@");

    qDebug() <<"strList.size = " << strList.size();

    // 반드시 크기 검사 먼저 (인덱스 접근 전)
    if (strList.size() <= 3) {
        qWarning() << "Invalid message format, parts:" << strList;

        qDebug() << ">>strList[0] : " << strList[0];
        qDebug() << ">>strList[1] : " << strList[1];
        qDebug() << ">>strList[2] : " << strList[2];

        return;
    }

    if(strList.size() <= 6)
    {
        qDebug() << ">>strList[0] : " << strList[0];
        qDebug() << ">>strList[1] : " << strList[1];
        qDebug() << ">>strList[2] : " << strList[2];
        qDebug() << ">>strList[3] : " << strList[3];
        qDebug() << ">>strList[4] : " << strList[4];
        qDebug() << ">>strList[5] : " << strList[5];
    }


#if 0
    if(strList[2].indexOf("LED") == 0)
    {
        bool bFlag;
        int ledNo = strList[3].toInt(&bFlag, 16);
        if(bFlag)
            emit ledWriteSig(ledNo);
    }
    else if((strList[2].indexOf("LAMP") == 0) || (strList[2].indexOf("PLUG") == 0))
    {
        emit tab3RecvDataSig(strRecvData);
    }
    else if(strList[2].indexOf("SENSOR") == 0)
    {
        emit tab4RecvDataSig(strRecvData);
        emit tab5RecvDataSig(strRecvData);
    }
#endif

    //[JAB_QT]TIME@시간@색깔@Z1 => @JAB_QT@TIME@시간@색깔@Z1
    if(strList.size() >= 5)
    {
        if(strList[4].indexOf("GREEN") == 0)    state_led = 0;
        if(strList[4].indexOf("YELLOW") == 0)   state_led = 1;
        if(strList[4].indexOf("RED") == 0)      state_led = 2;
        if(strList[5].indexOf("Z1") == 0)   ui->zoneLabel->setText("Zone 1");
    }

    ui->lightLabel->setPixmap(light_images[state_led]);
    ui->timeLabel->setText(strList[3]);    
}

void MainWidget::socketSendToLinux(int keyNo)
{
#if 0
    QString strSendData = QString::number(keyNo);
    strSendData = "[JAB_LIN]KEY@" + strSendData;
    //qDebug() << strSendData;
    pSocketClient->socketWriteDataSlot(strSendData);
#endif
}
