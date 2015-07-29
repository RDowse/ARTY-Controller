#ifndef IMAGEHANDLER_H
#define IMAGEHANDLER_H

#include <QObject>
#include <QQuickImageProvider>

class ImageHandler :public QObject, public QQuickImageProvider
{
    Q_OBJECT
private:
    QImage image_;
public:
    ImageHandler();
    QImage
    requestImage(const QString& id, QSize* size, const QSize& requestedSize);
public slots:
    void setImage(QImage image);
signals:
    Q_SIGNAL void signalNewFrameReady(int frameNumber);
};

#endif // IMAGEHANDLER_H
