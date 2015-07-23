import QtQuick 2.5
import QtQuick.Controls 1.1
import QtQuick.Layouts 1.1
import uk.ac.imperial.prl 1.0
import QtQuick.Window 2.0
import QtMultimedia 5.0
import QtQml 2.2

ApplicationWindow {
    title: qsTr("Arty Controller")
    id: mainWindow
    width: 800
    height: 600
    property int margin: 11
    visible: true
    visibility: Window.FullScreen
    //contentOrientation: Window.

    ROShandler {
        id: roshandler
        onLog: {
            console.log(msg);
            status.text = msg;
        }
    }

    Loader {
        id: loader
        anchors.fill: parent
    }

    menuBar: MenuBar {
        Menu {
            title: qsTr("File")
            MenuItem {
                text: qsTr("Clear")
                onTriggered: multiTouchTracking.clear();
            }
            MenuItem {
                text: qsTr("Reconnect")
                onTriggered: roshandler.restartROS();
            }
            MenuItem {
                text: qsTr("Settings")
                onTriggered: loader.source = "settings.qml";
            }
            MenuItem {
                text: qsTr("Exit")
                onTriggered: Qt.quit();
            }
        }
    }


    Label{
        //Shows log messages
        id: status
        Layout.minimumHeight: 30
        Layout.fillHeight: true
        Layout.fillWidth: true
        text: qsTr("Console")
    }

    GroupBox{
        //Area where the joystick can move.
        id: controller
        anchors.fill: parent
        Layout.fillWidth: true
        Layout.fillHeight: true
        Rectangle{
            id: area
            height: parent.height
            width: parent.height
            color: "#97FFFF"
            radius: width*0.5
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter  
        }
        Image {
            id: joystick
            source: touchArea.touch ? "Redbubble.svg" : "Bluebubble.svg"
            width: area.width / 5
            height: area.height / 5
            sourceSize.width: 256
            sourceSize.height: 256
            x: startPosX
            y: startPosY
            property real radius: width/2;
            property real startPosX: parent.width / 2 - radius
            property real startPosY: parent.height / 2 - radius
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
                running: touchArea.touch
            }
        }

        Canvas{
            //area for drawing dots to track the multitouch points.

            id:multiTouchTracking
            anchors.fill: parent
            antialiasing: true
            property real radius: 25
            property list<TouchPoint> touchPoints

            function updateTouchPoints(touchPoints){
                this.touchPoints = touchPoints;
                requestPaint();
            }

            function clear(){
                var context = getContext("2d");

                context.clearRect(0, 0, width, height);
            }

            onPaint: {
                var context = getContext("2d");
                context.save();

                context.clearRect(0, 0, width, height);
                for( var i = 0; i < touchPoints.length; i++){
                    context.beginPath();
                    context.arc(touchPoints[i].x, touchPoints[i].y, radius, 0, 2 * Math.PI, false);
                    context.fillStyle = 'green';
                    context.fill();
                    context.lineWidth = 5;
                    context.strokeStyle = '#003300';
                    context.stroke();
                 }
            }
        }

        MultiPointTouchArea {
            id: touchArea
            anchors.fill: parent
            property bool touch: false;
            property bool ready: true;
            maximumTouchPoints: 20;
            function averagePoints(touchPoints){
                var sumX = 0, sumY = 0;
                for( var i = 0; i < touchPoints.length; i++){
                    sumX += parseFloat(touchPoints[i].x);
                    sumY += parseFloat(touchPoints[i].y);
                }
                var avgX = sumX/touchPoints.length;
                var avgY = sumY/touchPoints.length;
                return {x:avgX, y:avgY};
            }

            function calcVelocityAngle(){
                var x = joystick.x - joystick.startPosX;
                var y = joystick.startPosY - joystick.y;

                var tempAng = Math.atan2(x,y);
                var tempDist = Math.sqrt(x*x + y*y);

                var velocity = Math.cos(tempAng)*(tempDist/area.radius);
                var angle = Math.sin(-tempAng)*(tempDist/area.radius);

                return {velocity:velocity,angle:angle};
            }

            function setPosition(x,y){
                var diffX = x - (joystick.startPosX + joystick.radius);
                var diffY = (joystick.startPosY + joystick.radius) - y;
                var tempAng = Math.atan2(diffX,diffY);
                var distance = Math.sqrt(diffX*diffX + diffY*diffY);

                //Check if the joystick is out of bounds.
                if(distance  > area.radius){
                    joystick.x = joystick.startPosX + area.radius * Math.sin(tempAng);
                    joystick.y = joystick.startPosY - area.radius * Math.cos(tempAng);
                }else{
                    joystick.x = x - joystick.radius;
                    joystick.y = y - joystick.radius;
                }
            }

            function sendVelocityAngle(velocity,angle){
                roshandler.setVelAng(velocity, angle);
            }

            onTouchUpdated:{
                //When the joystick is released reset the position.
                if(touchPoints.length === 0){
                    joystick.x = joystick.startPosX;
                    joystick.y = joystick.startPosY;
                    roshandler.setVelAng(0.0,0.0);
                    touch = false;
                    offSound.play();
                    ready = true;
                    return;
                } else if(ready){
                    onSound.play();
                    ready = false;
                }
                touch = true;
                var newPosition = averagePoints(touchPoints);
                setPosition(newPosition.x,newPosition.y);
                var newVelocityAngle = calcVelocityAngle();
                sendVelocityAngle(newVelocityAngle.velocity, newVelocityAngle.angle);
                multiTouchTracking.updateTouchPoints(touchPoints);
            }

            SoundEffect {
                id: onSound
                source: "sounds/on/S3K_33.wav"
                muted: true
            }
            SoundEffect{
                id: offSound
                source: "sounds/off/S3K_4D.wav"
                muted: true
            }
        }
    }
}


