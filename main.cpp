#include <QApplication>
#include <QtQuick>
#include <QQmlApplicationEngine>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "roshandlerwrapper.h"
#include "imagehandler.h"

#define LASTERR strerror(errno)

Q_DECL_EXPORT int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    qmlRegisterType<ROSHandlerWrapper>("uk.ac.imperial.prl", 1, 0, "ROSHandlerWrapper");
    qmlRegisterSingletonType(
        QUrl("qrc:/Settings.qml"), "uk.ac.imperial.settings", 1, 0, "Settings");
    auto* imagehandler = new ImageHandler();

    QQmlApplicationEngine engine;
    engine.addImageProvider("imagehandler", imagehandler);
    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));
    engine.rootContext()->setContextProperty("ImageHandler", imagehandler);

    return app.exec();
}
