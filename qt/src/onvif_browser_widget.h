#ifndef ONVIF_BROWSER_WIDGET_H
#define ONVIF_BROWSER_WIDGET_H

#include <QWidget>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QSettings>
#include <atomic>

class RtspPlayerWindow;

class OnvifBrowserWidget : public QWidget
{
    Q_OBJECT
public:
    explicit OnvifBrowserWidget(QWidget* parent = nullptr);
    ~OnvifBrowserWidget() override;

private slots:
    void onSearch();
    void onDoubleClick(int row, int col);

private:
    QLineEdit *timeoutEdit_{}, *retriesEdit_{}, *typesEdit_{}, *userEdit_{}, *passEdit_{};
    QPushButton*   searchBtn_{};
    QTableWidget*  table_{};
    QLabel*        status_{};
};

#endif // ONVIF_BROWSER_WIDGET_H
