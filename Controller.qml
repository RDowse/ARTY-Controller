import QtQuick 2.0
import QtMultimedia 5.0
import QtQuick.Layouts 1.2
import uk.ac.imperial.settings 1.0
import uk.ac.imperial.prl 1.0

Item{
    //Area where the joystick can move.
    id: controller
    //property string controllerAreaColour:

    Rectangle{
        id: area
        height: parent.height
        width: parent.height
        color: Settings.controllerAreaColour
        radius: width*0.5
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
    }

    Joystick {
        id: joystick
        x: controller.width / 2 - radius
        y: controller.height / 2 - radius
        startPosX: controller.width / 2 - radius
        startPosY: controller.height / 2 - radius
    }

    MultiPointTouchArea {
        id: touchArea
        anchors.fill: parent
        property bool ready: true;
        maximumTouchPoints: 20;
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
            var tempDist = Math.sqrt(x*x + y*y);

            var velocity = Math.cos(tempAng)*(tempDist/area.radius);
            var angle = Math.sin(-tempAng)*(tempDist/area.radius);

            return {velocity:velocity,angle:angle};
        }

        function setPosition(x,y){
            var diffX = x - (joystick.startPosX + joystick.radius);
            var diffY = (joystick.startPosY + joystick.radius) - y;
            var tempAng = Math.atan2(diffX,diffY);
            var distance = Math.sqrt(diffX*diffX + diffY*diffY);

            //Check if the joystick is out of bounds.
            if(distance  > area.radius){
                joystick.x = joystick.startPosX + area.radius * Math.sin(tempAng);
                joystick.y = joystick.startPosY - area.radius * Math.cos(tempAng);
            }else{
                joystick.x = x - joystick.radius;
                joystick.y = y - joystick.radius;
            }
        }

        function sendVelocityAngle(velocity,angle){
            roshandler.setTwistMsg(velocity,angle);
        }

        onTouchUpdated:{
            //When the joystick is released reset the position.
            if(touchPoints.length === 0){
                joystick.resetPos();
                sendVelocityAngle(0.0,0.0);
                joystick.onTouch(false);
                offSound.play();
                ready = true;
                return;
            } else if(ready){
                onSound.play();
                ready = false;
            }
            joystick.onTouch(true);
            var newPosition = averagePoints(touchPoints);
            setPosition(newPosition.x,newPosition.y);
            var newVelocityAngle = calcVelocityAngle();
            sendVelocityAngle(newVelocityAngle.velocity, newVelocityAngle.angle);
            touchPointTracking.updateTouchPoints(touchPoints);
        }

        SoundEffect {
            id: onSound
            source: "sounds/on/S3K_33.wav"
            muted: true
        }
        SoundEffect{
            id: offSound
            source: "sounds/off/S3K_4D.wav"
            muted: true
        }
    }

}
