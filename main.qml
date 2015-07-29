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


    Label{
        //Shows log messages
        id: status
        Layout.minimumHeight: 30
        Layout.fillHeight: true
        Layout.fillWidth: true
        text: qsTr("Console")
    }

    Controller{
        id: controller
    }

    Loader {
        id: loader
        anchors.fill: parent
    }


    Image {
        id:mapArea
        source: "image://imagehandler/map"
        cache: false
        width: 300
        height: 300
        // http://stackoverflow.com/a/17734973
        function reload() {
            var oldSource = source;
            source = "";
            source = oldSource;
        }
    }

    Connections {
        target: roshandler
        onMapUpdate: {
            ImageHandler.setImage(map);
            mapArea.reload();
        }
    }
}
