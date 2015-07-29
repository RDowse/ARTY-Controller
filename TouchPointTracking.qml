import QtQuick 2.0

Item{
    id:touchPointTracking
    property list<TouchPoint> touchPoints
    property real sizeRatio: height/controller.height

    function updateTouchPoints(touchPoints){
        this.touchPoints = touchPoints;
        multiTouchTracking.requestPaint();
    }

    function clear(){
        var context = getContext("2d");
        context.clearRect(0, 0, width, height);
    }

    Rectangle{
        color: "#AFAFAF"
        anchors.fill: parent
    }

    Canvas{
        //area for drawing dots to track the multitouch points.

        id:multiTouchTracking
        anchors.fill: parent
        antialiasing: true
        property real radius: 5

        onPaint: {
            var context = getContext("2d");
            context.save();

            context.clearRect(0, 0, width, height);
            for( var i = 0; i < touchPoints.length; i++){
                context.beginPath();
                context.arc(touchPoints[i].x*sizeRatio, touchPoints[i].y*sizeRatio, radius, 0, 2 * Math.PI, false);
                context.fillStyle = 'green';
                context.fill();
                context.lineWidth = 5;
                context.strokeStyle = '#003300';
                context.stroke();
             }
        }
    }
}
