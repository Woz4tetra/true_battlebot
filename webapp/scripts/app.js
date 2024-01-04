var ros;
var cageCornerPub;
var robotModePub;
var requestFieldPub;
var recordService;
var recordState = false;

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
    console.log(`Publishing corner ${type}`);
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

function publishRobotMode(mode) {
    console.log(`Publishing robot mode ${mode}`);
    robotModePub.publish({ mode: mode });
}

function initRobotModePublisher() {
    robotModePub = new ROSLIB.Topic({
        ros: ros,
        name: "/behavior_mode",
        messageType: "bw_interfaces/BehaviorMode",
    });
    robotModePub.advertise();
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
    console.log("Publishing request field");
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

    ros = new ROSLIB.Ros({
        url: "ws://" + robot_ip + ":9090",
    });

    ros.on("connection", function () {
        console.log("Connected to websocket server.");
    });

    ros.on("error", function (error) {
        console.log("Error connecting to websocket server: ", error);
    });

    ros.on("close", function () {
        console.log("Connection to websocket server closed.");
    });

    initSummarySubscriber();
    initCageCornerPublisher();
    initRobotModePublisher();
    initRequestFieldPublisher();
    initRecordService();
};
