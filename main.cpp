#include <QApplication>
#include <QtQuick>
#include <QQmlApplicationEngine>

#include <ros/ros.h>
#include <std_msgs/String.h>

//#include <QtDeclarative>

#include "roshandler.h"

#define LASTERR strerror(errno)


Q_DECL_EXPORT int main(int argc, char *argv[])
{
    qmlRegisterType<ROShandler>("uk.ac.imperial.prl", 1, 0, "ROShandler");

    QApplication app(argc, argv);
    QQmlApplicationEngine engine;
    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));

    return app.exec();
}
