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
                text: qsTr("Clear")
                onTriggered: multiTouchTracking.clear();
            }
            MenuItem {
                text: qsTr("Reconnect")
                onTriggered: roshandler.restartROS();
            }
            MenuItem {
                text: qsTr("Settings")
                onTriggered: {
                    loader.source = "SettingsMenu.qml";
                }
            }
            MenuItem {
                text: qsTr("Exit")
                onTriggered: Qt.quit();
            }
        }
    }

    GridLayout {
        anchors.fill: parent
        columns: 2
        rows: 1
        columnSpacing: 2

        Column{
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

            Image {
                id:mapArea
                source: "image://imagehandler/map"
                cache: false
                width: 600
                height: 400

                // http://stackoverflow.com/a/17734973
                function reload() {
                    var oldSource = source;
                    source = "";
                    source = oldSource;
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
        anchors.fill: parent
    }

    Connections {
        target: roshandler
        onMapUpdate: {
            ImageHandler.setImage(map);
            mapArea.reload();
        }
    }
}
