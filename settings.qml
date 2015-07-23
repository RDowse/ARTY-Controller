import QtQuick 2.0
import QtQuick.Layouts 1.2

Rectangle{
    width: 800
    height: 600
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

    }
}
