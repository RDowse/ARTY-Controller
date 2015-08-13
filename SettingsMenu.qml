import QtQuick 2.0
import QtQuick.Layouts 1.2
import QtQuick.Controls 1.4
import QtQuick.Dialogs 1.2
import uk.ac.imperial.settings 1.0

Rectangle{
    id: settingsMenu
    anchors.fill: parent
    color: "#FFFFFFFF"

    signal pageExit

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
        Button{
            text: "Set joystick color"
            onClicked:{
                colorDialogJoystick.visible = true;
            }
        }
        Button{
            text: "Set background color"
            onClicked:{
                colorDialogBackground.visible = true;
            }
        }
        ColorDialog {
            id: colorDialogJoystick
            title: "Please choose a color"
            onAccepted: {
                Settings.joystickColourInactive = colorDialogJoystick.color;
                visible = false;
            }
            onRejected: {
                console.log("Canceled")
                visible = false;
            }
        }
        ColorDialog {
            id: colorDialogBackground
            title: "Please choose a color"
            onAccepted: {
                Settings.controllerAreaColour = colorDialogBackground.color
                visible = false;
            }
            onRejected: {
                console.log("Canceled")
                visible = false;
            }
        }
        Button {
            text: "Back"
            onClicked: {
                roshandler.setMasterIP(editMasterIP.text);
                loader.visible = false;
                mainLayout.visible = true;
            }
        }
    }
}
