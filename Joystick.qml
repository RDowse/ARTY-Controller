import QtQuick 2.5
import uk.ac.imperial.settings 1.0

Item{
    id: joystick
    property real size: 240
    width: size
    height: size
    property real radius: width/2;
    property real startPosX
    property real startPosY
    property bool touched: false

    function onTouch(touch){
        touched = touch;
    }
    function resetPos(){
        x = startPosX;
        y = startPosY;
    }

    Behavior on y {
        SmoothedAnimation {
            easing.type: Easing.Linear
            duration: 100
        }
    }
    Behavior on x {
        SmoothedAnimation {
            easing.type: Easing.Linear
            duration: 100
            }
    }

    Rectangle{
        id: joystickImage
        anchors.fill: parent
        radius: joystick.radius
        color: touched ? Settings.joystickColourActive : Settings.joystickColourInactive

        SequentialAnimation{
            ScaleAnimator {
                target: joystick
                from: 1.0
                to: 1.5
                duration: 1000
            }
            ScaleAnimator {
                target: joystick
                from: 1.5
                to: 1.0
                duration: 1000
            }
            loops: Animation.Infinite
            running: touched
        }
    }
}
