import QtQuick 2.0
import QtQuick.Controls 1.1
import QtQuick.Layouts 1.1
import uk.ac.imperial.prl 1.0
import QtQuick.Window 2.0

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

    menuBar: MenuBar {
        Menu {
            title: qsTr("File")
            MenuItem {
                text: qsTr("Settings")
                onTriggered: ld.source="settings.qml";
            }
            MenuItem {
                text: qsTr("Exit")
                onTriggered: Qt.quit();
            }
            Loader{
                id:ld;
                anchors.fill: parent;
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
            color: "#d3d3d3"
            radius: width*0.5
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter

            Image {
                id: joystick
                source: "Bluebubble.svg"
                width: parent.width / 5
                height: parent.height / 5
                x: startPosX
                y: startPosY
                property real radius: width/2;
                property real startPosX: parent.width / 2 - radius
                property real startPosY: parent.height / 2 - radius
                Behavior on y {
                    SmoothedAnimation {
                        easing.type: Easing.Linear
                        duration: 200
                    }
                }
                Behavior on x {
                    SmoothedAnimation {
                        easing.type: Easing.Linear
                        duration: 200
                        }
                }
            }

            MultiPointTouchArea {
                id: touchArea
                anchors.fill: parent
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

                    var velocity = Math.cos(tempAng);
                    var angle = Math.sin(-tempAng);

                    return {velocity:velocity,angle:angle};
                }

                function setPosition(x,y){
                    var diffX = x- joystick.startPosX;
                    var diffY = joystick.startPosY - y;
                    var tempAng = Math.atan2(diffX,diffY);
                    var distance = Math.sqrt(diffX*diffX + diffY*diffY);

                    //Check if the joystick is out of bounds.
                    if(distance  > area.radius){
                        joystick.x = joystick.startPosX + area.radius * Math.sin(tempAng) - joystick.radius;
                        joystick.y = joystick.startPosY - area.radius * Math.cos(tempAng) - joystick.radius;
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
                        return;
                    }

                    var newPosition = averagePoints(touchPoints);

                    setPosition(newPosition.x,newPosition.y);

                    var newVelocityAngle = calcVelocityAngle();

                    sendVelocityAngle(newVelocityAngle.velocity, newVelocityAngle.angle);
                }
            }
        }
    }
}


