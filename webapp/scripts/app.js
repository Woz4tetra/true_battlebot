var twist;
var cmdVel;
var teleop;

var ros;
var cageCornerPub;
var requestFieldPub;
var recordService;
var recordState = false;

function moveAction(linear, angular) {
    if (linear !== undefined && angular !== undefined) {
        twist.linear.x = linear;
        twist.angular.z = angular;
    } else {
        twist.linear.x = 0;
        twist.angular.z = 0;
    }
    cmdVel.publish(twist);
}

function initVelocityPublisher() {
    // Init message with zero values.
    twist = new ROSLIB.Message({
        linear: {
            x: 0,
            y: 0,
            z: 0,
        },
        angular: {
            x: 0,
            y: 0,
            z: 0,
        },
    });
    // Init topic object
    cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: "/cmd_vel",
        messageType: "geometry_msgs/Twist",
    });
    // Register publisher within ROS system
    cmdVel.advertise();
}

function initTeleopKeyboard() {
    // Use w, s, a, d keys to drive your robot

    // Check if keyboard controller was aready created
    if (teleop == null) {
        // Initialize the teleop.
        teleop = new KEYBOARDTELEOP.Teleop({
            ros: ros,
            topic: "/cmd_vel",
        });
    }

    // Add event listener for slider moves
    robotSpeedRange = document.getElementById("robot-speed");
    robotSpeedRange.oninput = function () {
        teleop.scale = robotSpeedRange.value / 100;
    };
}

function initSummarySubscriber() {
    var listener = new ROSLIB.Topic({
        ros: ros,
        name: "/system_summary",
        messageType: "bw_interfaces/SystemSummary",
    });

    listener.subscribe(function (message) {
        document.getElementById("summary-robot").innerHTML = message.robot;
        document.getElementById("summary-map").innerHTML = message.map;
        document.getElementById("summary-version").innerHTML = message.version;
    });
}

function publishCageCorner(type) {
    cageCornerPub.publish({ type: type });
}

function initCageCornerPublisher() {
    cageCornerPub = new ROSLIB.Topic({
        ros: ros,
        name: "/set_cage_corner",
        messageType: "bw_interfaces/CageCorner",
    });
    cageCornerPub.advertise();
}

function initRequestFieldPublisher() {
    requestFieldPub = new ROSLIB.Topic({
        ros: ros,
        name: "/manual_plane_request",
        messageType: "std_msgs/Empty",
    });
    requestFieldPub.advertise();
}

function publishRequestField() {
    requestFieldPub.publish({});
}

function initRecordService() {
    recordService = new ROSLIB.Service({
        ros: ros,
        name: "/set_record",
        serviceType: "std_srvs/SetBool",
    });
}

function setRecordState(button, state) {
    recordState = state;
    if (state) {
        next_text = "Stop Recording";
    } else {
        next_text = "Start Recording";
    }
    console.log(`Setting record state to ${state}`);
    button.disabled = true;
    button.innerHTML = "Waiting...";
    recordService.callService(
        new ROSLIB.ServiceRequest({ data: state }),
        function (result) {
            console.log(
                `Request: ${state}. Result for service call on ${recordService.name}: ${result}`
            );
            button.disabled = false;
            button.innerHTML = next_text;
        }
    );
}

window.onload = function () {
    var robot_ip = location.hostname;

    // // Init handle for rosbridge_websocket
    ros = new ROSLIB.Ros({
        url: "ws://" + robot_ip + ":9090",
    });

    initSummarySubscriber();
    initCageCornerPublisher();
    initRequestFieldPublisher();
    initRecordService();
};
