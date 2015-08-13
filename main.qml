import QtQuick 2.5
import QtQuick.Controls 1.1
import QtQuick.Layouts 1.1
import uk.ac.imperial.prl 1.0
import QtQuick.Window 2.0
import QtQml 2.2
import "/"

ApplicationWindow {
    title: qsTr("Arty Controller")
    id: mainWindow
    property int margin: 11
    visible: true
    visibility: Window.FullScreen

    ROSHandlerWrapper {
        id: roshandler
        onLog: {
            //console.log(msg);
            status.text = msg;
        }
    }

    menuBar: MenuBar {
        Menu {
            title: qsTr("File")
            MenuItem {
                text: qsTr("Clear")
                onTriggered: multiTouchTracking.clear();
            }
            MenuItem {
                text: qsTr("Settings")
                onTriggered: {
                    mainLayout.visible = false;
                    loader.visible = true;
                }
            }
            MenuItem {
                text: qsTr("Exit")
                onTriggered: Qt.quit();
            }
        }
    }

    GridLayout {
        id:mainLayout
        anchors.fill: parent
        columns: 2
        rows: 1
        columnSpacing: 2

        Column{
            //set to visible to use these features
            visible: false

            Layout.alignment: Qt.AlignTop
            Layout.column: 1
            Layout.row: 1
            Label{
                //Shows log messages
                id: status
                Layout.minimumHeight: 30
                Layout.fillHeight: true
                width: 600
                text: qsTr("Console")
            }

            Item{
                visible: false
                id:mapArea
                width: 600
                height: 400
                function setPosition(position){
                    chairPosition.x = position.x;
                    chairPosition.y = position.y;
                }
                Image {
                    id:mapImage
                    source: "image://imagehandler/map"
                    cache: false
                    anchors.fill: parent

                    // http://stackoverflow.com/a/17734973
                    function reload() {
                        var oldSource = source;
                        source = "";
                        source = oldSource;
                    }
                }
                Rectangle{
                    id: chairPosition
                    height: 25
                    width: 25
                    radius: width*0.5
                    color: "#00FF00"
                    x: parent.x + parent.width/2
                    y: parent.y + parent.height/2
                }
            }

            TouchPointTracking{
                id:touchPointTracking
                Layout.fillWidth: true
                Layout.fillHeight: true
                height: 600
                width: 600
            }
        }

        Controller{
            id: controller
            Layout.column: 2
            Layout.row: 1
            Layout.fillHeight: true
            Layout.fillWidth: true
        }

    }

    Loader {
        id: loader
        source: "SettingsMenu.qml"
        anchors.fill: parent
        focus: false
        visible: false
    }

//    Connections {
//        target: roshandler
//        onMapUpdate: {
//            ImageHandler.setImage(map);
//            mapImage.reload();
//        }
//        onPosUpdate: {
//            mapPos.setPosition(position);
//        }
//    }

}
