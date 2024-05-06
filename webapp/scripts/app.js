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

function initTreeSnapshotSubscriber() {
    var listener = new ROSLIB.Topic({
        ros: ros,
        name: "/tree_snapshot",
        messageType: "std_msgs/String",
    });

    listener.subscribe(function (message) {
        document.getElementById("tree-snapshot").innerHTML = message.data;
    });
}

function initCarets() {
    var toggler = document.getElementsByClassName("caret");
    console.log(`Found ${toggler.length} carets`);
    var i;

    for (i = 0; i < toggler.length; i++) {
        toggler[i].addEventListener("click", function () {
            console.log("Clicked caret");
            this.parentElement
                .querySelector(".nested")
                .classList.toggle("active");
            this.classList.toggle("caret-down");
        });
    }
}

function initHealthSummarySubscriber() {
    var listener = new ROSLIB.Topic({
        ros: ros,
        name: "/health",
        messageType: "bw_interfaces/HealthSummary",
    });

    listener.subscribe(function (message) {
        document.getElementById("health-active-count").innerHTML =
            "Active nodes: " + message.active_nodes.length;
        document.getElementById("health-dead-count").innerHTML =
            "Dead nodes: " + message.dead_nodes.length;

        let active_list = document.getElementById("health-active-list");
        active_list.innerHTML = "";
        message.active_nodes.forEach((node) => {
            element = document.createElement("div");
            element.classList.add("grid-item");
            element.innerHTML = node;
            active_list.appendChild(element);
        });

        let dead_list = document.getElementById("health-dead-list");
        dead_list.innerHTML = "";
        message.dead_nodes.forEach((node) => {
            element = document.createElement("div");
            element.classList.add("grid-item");
            element.innerHTML = node;
            dead_list.appendChild(element);
        });
    });
}

function initCarets() {
    var toggler = document.getElementsByClassName("caret");
    console.log(`Found ${toggler.length} carets`);
    var i;

    for (i = 0; i < toggler.length; i++) {
        toggler[i].addEventListener("click", function () {
            this.parentElement
                .querySelector(".nested")
                .classList.toggle("active");
            this.classList.toggle("caret-down");
        });
    }
}

window.onload = function () {
    var robot_ip = location.hostname;
    let url = "ws://" + robot_ip + ":9090";

    ros = new ROSLIB.Ros({
        url: url,
    });

    var connection_status = document.getElementById("connection-status");
    var connection_icon = document.getElementById("connection-icon");
    connection_icon.src = "resources/error_black_24dp.svg";
    connection_icon.className = "filter-red";
    connection_icon.ondragstart = function () {
        return false;
    };

    ros.on("connection", function () {
        connection_status.innerHTML = "Connected";
        connection_icon.src = "resources/check_circle_black_24dp.svg";
        connection_icon.className = "filter-green";
        console.log("Connected to websocket server.");
    });

    ros.on("error", function (error) {
        console.log("Error connecting to websocket server: ", error);
        connection_status.innerHTML = "Error: " + error;
        connection_icon.src = "resources/error_black_24dp.svg";
        connection_icon.className = "filter-red";
    });

    ros.on("close", function () {
        console.log("Connection to websocket server closed.");
        connection_status.innerHTML = "Disconnected";
        connection_icon.src = "resources/error_black_24dp.svg";
        connection_icon.className = "filter-red";
        ros.connect(url);
    });

    initSummarySubscriber();
    initCageCornerPublisher();
    initRobotModePublisher();
    initRequestFieldPublisher();
    initRecordService();
    initTreeSnapshotSubscriber();
    initCarets();
    initHealthSummarySubscriber();
    console.log("App loaded");
};
