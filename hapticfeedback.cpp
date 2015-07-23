#include "hapticfeedback.h"

HapticFeedback::HapticFeedback()
{

}

HapticFeedback::HapticFeedback(QObject *parent) : QObject(parent){}

void HapticFeedback::vibrate(){
    QAndroidJniObject::callStaticMethod<void>(
                "org/qtproject/example/Chronometer/Vibrate", "start", "(I)V", 300);
}
