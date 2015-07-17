import QtQuick 2.0
import QtQuick.Controls 1.1
import QtQuick.Layouts 1.1
import uk.ac.imperial.prl 1.0

ApplicationWindow {
    title: qsTr("Arty Controller")
    id: mainWindow
    width: 800
    height: 600
    property int margin: 11
    visible: true


    menuBar: MenuBar {
        Menu {
            title: qsTr("File")
            MenuItem {
                text: qsTr("&Open")
                onTriggered: console.log("Open action triggered");
            }
            MenuItem {
                text: qsTr("Exit")
                onTriggered: Qt.quit();
            }
        }
    }

    ROShandler {
        id: roshandler
        onLog: {
            console.log(msg);
            status.text = msg;
        }
    }

    Column {
        id: mainLayout
        anchors.fill: parent
        anchors.margins: margin
        spacing: 5

        Label{
            //Shows log messages
            id: status
            Layout.minimumHeight: 30
            Layout.fillHeight: true
            Layout.fillWidth: true
            text: qsTr("message")
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
                    width: parent.width / 5;
                    height: parent.height / 5;
                    x: startPosX
                    y: startPosY
                    property real centerX: parent.width / 2
                    property real centerY: parent.height / 2
                    property real joyStickCenter: joystick.width / 2
                    property real startPosX: centerX - joyStickCenter
                    property real startPosY: centerY - joyStickCenter
                    property double velocity: 0.0
                    property double angle: 0.0


                    Behavior on y {
                        SmoothedAnimation {
                            easing.type: Easing.Linear
                            duration: 200
                        }
                    }
                    //anchors.verticalCenter: parent.verticalCenter
                    Behavior on x {
                        SmoothedAnimation {
                            easing.type: Easing.Linear
                            duration: 200
                            }
                        }
                    Drag.active: joystickMouseArea.drag.active
                    Drag.hotSpot.x: 10
                    Drag.hotSpot.y: 10

                    MouseArea{
                        id: joystickMouseArea
                        anchors.fill: parent
                        drag.target: joystick
                        drag.minimumY: 0
                        drag.maximumY: parent.parent.height - joystick.height
                        drag.minimumX: 0
                        drag.maximumX: parent.parent.width - joystick.width

                        onReleased: {
                            joystick.x = joystick.startPosX;
                            joystick.y = joystick.startPosY;
                            roshandler.setVelAng(0.0,0.0);
                        }

                        onPositionChanged: {
                            var x = joystick.x - joystick.startPosX;
                            var y = joystick.startPosY - joystick.y;
                            var tempVel = 2*Math.sqrt(x*x + y*y)/parent.parent.height;
                            var tempAng = Math.atan2(x,y);

                            joystick.velocity = Math.cos(tempAng);
                            joystick.angle = Math.sin(-tempAng);
                            console.log("Velocity: "+ joystick.velocity + " Angle: " + joystick.angle);
                            roshandler.setVelAng(joystick.velocity, joystick.angle);
                        }
                    }
                }
            }
        }
    }
}

