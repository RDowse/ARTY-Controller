#ifndef HAPTICFEEDBACK_H
#define HAPTICFEEDBACK_H

#include <QObject>
#include <QtAndroidExtras/QAndroidJniObject>

class HapticFeedback : public QObject
{
    Q_OBJECT
public:
    HapticFeedback();
    explicit HapticFeedback(QObject *parent);
    Q_INVOKABLE void vibrate();
};

#endif // HAPTICFEEDBACK_H
