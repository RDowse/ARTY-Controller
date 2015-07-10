#-------------------------------------------------
#
# Project created by QtCreator 2015-07-10T13:58:58
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Controller
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    #test.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui

CONFIG += mobility
MOBILITY = 


LIBS += -L$$PWD/../roscpp_android_ndk/lib/ -lroscpp  -lboost_signals -lboost_filesystem -lrosconsole -lrosconsole_print -lrosconsole_backend_interface -lboost_regex -lxmlrpcpp -lroscpp_serialization -lrostime -lboost_date_time -lcpp_common -lboost_system -lboost_thread -lconsole_bridge

INCLUDEPATH += $$PWD/../roscpp_android_ndk/include
DEPENDPATH += $$PWD/../roscpp_android_ndk/include

#PRE_TARGETDEPS += $$PWD/../roscpp_android_ndk/lib/libroscpp.a
