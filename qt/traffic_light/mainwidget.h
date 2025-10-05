#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <QWidget>
#include <QDebug>
#include "socketclient.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWidget;
}
QT_END_NAMESPACE

class MainWidget : public QWidget
{
    Q_OBJECT

public:
    MainWidget(QWidget *parent = nullptr);
    ~MainWidget();
    SocketClient* getpSocketClient();

private slots:
    void on_pushButton_clicked();
    void updateRecvDataSlot(QString);
    void socketSendToLinux(int keyNo);

private:
    Ui::MainWidget *ui;
    SocketClient *pSocketClient;

    int state_led = 0;    // 0 : green, 1 : yello , 2 : red
    QPixmap light_images[3];

    bool bFlag;

};
#endif // MAINWIDGET_H
