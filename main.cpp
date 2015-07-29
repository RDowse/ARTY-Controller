#include <QApplication>
#include <QtQuick>
#include <QQmlApplicationEngine>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "roshandler.h"
#include "imagehandler.h"

#define LASTERR strerror(errno)

Q_DECL_EXPORT int main(int argc, char* argv[])
{
    qmlRegisterType<ROShandler>("uk.ac.imperial.prl", 1, 0, "ROShandler");

    auto* imagehandler = new ImageHandler();
    QApplication app(argc, argv);
    QQmlApplicationEngine engine;
    engine.addImageProvider("imagehandler", imagehandler);
    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));
    engine.rootContext()->setContextProperty("ImageHandler", imagehandler);

    return app.exec();
}
