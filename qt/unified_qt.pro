QT       += core gui network concurrent

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# Use pkg-config for OpenCV and tinyxml2 when available
CONFIG += link_pkgconfig
PKGCONFIG += opencv4 tinyxml2

# Fallback: try to link tinyxml2 by name if pkg-config is missing
LIBS += -ltinyxml2

win32: LIBS += -lws2_32

SOURCES += \
    src/main.cpp \
    src/mainwidget.cpp \
    src/socketclient.cpp \
    src/onvif_browser_widget.cpp

HEADERS += \
    src/mainwidget.h \
    src/socketclient.h \
    src/onvif_browser_widget.h

FORMS += \
    src/mainwidget.ui

RESOURCES += \
    src/images.qrc

DISTFILES += README.md

TARGET = unified_qt
TEMPLATE = app

