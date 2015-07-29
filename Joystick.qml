import QtQuick 2.5

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
            duration: 50
        }
    }
    Behavior on x {
        SmoothedAnimation {
            easing.type: Easing.Linear
            duration: 50
            }
    }

    Image {
        id: image
        source: touched ? "images/Redbubble.svg" : "images/Bluebubble.svg"
        anchors.fill: parent
        sourceSize.width: 256
        sourceSize.height: 256

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
