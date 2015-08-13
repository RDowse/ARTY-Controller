pragma Singleton
import QtQuick 2.0

QtObject {
    property int screenHeight: 2048
    property int screenWidth: 1536

    property int fontPixelSize: 14
    property int joystickSize: 60
    property int toolButtonHeight: 64
    property int menuButtonSpacing: 0

    //Colours
    property string controllerAreaColour: "#E2ECEE"
    property string joystickColourActive: "#FF0000"
    property string joystickColourInactive: "#0000FF"

    //Images
    property string joystickImg1: "images/SonicRedbubble.svg"
    property string joystickImg2: "images/SonicBluebubble.svg"
}
