#-------------------------------------------------
#
# Project created by QtCreator 2015-07-10T13:58:58
#
#-------------------------------------------------

QT += core gui androidextras qml quick widgets svg declarative

QMAKE_CXXFLAGS += -Wno-unused-local-typedefs -Wno-unused-parameter

greaterThan(QT_MAJOR_VERSION, 5): QT += widgets

TARGET = Controller
TEMPLATE = app

SOURCES += main.cpp\
    hapticfeedback.cpp \
    imagehandler.cpp \
    roshandler.cpp \
    r.cpp \
    roshandlerwrapper.cpp

HEADERS  += \
    hapticfeedback.h \
    imagehandler.h \
    Any.h \
    roshandler.h \
    r.h \
    roshandlerwrapper.h \
    publisherwrapper.hxx \
    subscriberwrapper.hxx

FORMS    +=

CONFIG += mobility
MOBILITY = 

RESOURCES += qml.qrc

LIBS += -L$$PWD/../roscpp_android_ndk/lib/ -lroscpp  -lboost_signals -lboost_filesystem -lrosconsole -lrosconsole_print -lrosconsole_backend_interface -lboost_regex -lxmlrpcpp -lroscpp_serialization -lrostime -lcpp_common -lboost_date_time  -lboost_system -lboost_thread  -lconsole_bridge

INCLUDEPATH += $$PWD/../roscpp_android_ndk/include
DEPENDPATH += $$PWD/../roscpp_android_ndk/include

PRE_TARGETDEPS += $$PWD/../roscpp_android_ndk/lib/libroscpp.a

DISTFILES += \
    Vibrate.java \
    android/AndroidManifest.xml \
    android/gradle/wrapper/gradle-wrapper.jar \
    android/gradlew \
    android/res/values/libs.xml \
    android/build.gradle \
    android/gradle/wrapper/gradle-wrapper.properties \
    android/gradlew.bat

RESOURCES += \
    qml.qrc

ANDROID_PACKAGE_SOURCE_DIR = $$PWD/android
