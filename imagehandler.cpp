#include "imagehandler.h"

ImageHandler::ImageHandler()
    : QQuickImageProvider(QQmlImageProviderBase::Image)
{
}

void ImageHandler::setImage(QImage image) { image_ = image; }

QImage ImageHandler::requestImage(
    const QString& id, QSize* size, const QSize& requestedSize)
{
    int width = 100;
    int height = 50;

    if(image_.isNull()){
        QImage image(requestedSize.width() > 0 ? requestedSize.width() : width,
            requestedSize.height() > 0 ? requestedSize.height() : height,
            QImage::Format_RGB32);
        image.fill(QColor("yellow").rgba());
        return image;
    } else{
        return image_;
    }

}
