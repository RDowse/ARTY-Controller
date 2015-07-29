import QtQuick 2.0
import QtQuick.Layouts 1.2
import QtQuick.Controls 1.4

Rectangle{
    id: settingsMenu
    anchors.fill: mainWindow
    color: "#FFFFFFFF"

    GridLayout {
        id: grid
        columns: 2
        anchors.fill: parent
        Text {
            text: "Master IP"
        }
        TextEdit {
            id: editMasterIP
            text: qsTr(roshandler.getMasterIP());
        }
        Button {
            text: "Back"
            onClicked: settingsMenu.visible = false
        }
    }
}
