import QtQuick 2.0
import QtQuick.Layouts 1.2
import QtQuick.Controls 1.4

Rectangle{
    id: settingsMenu
    anchors.fill: parent
    color: "#FFFFFFFF"

    GridLayout {
        id: grid
        columns: 2
        columnSpacing: 10
        anchors.fill: parent

        Text {
            text: "Master IP"
        }
        TextEdit {
            property string masterIP: roshandler.getMasterIP()
            Layout.fillWidth: true
            id: editMasterIP
            text:  qsTr(masterIP);
            focus: true
        }
        Button {
            text: "Back"
            onClicked: {
                settingsMenu.visible = false;
                controller.visible = true;
            }
        }
    }
}
