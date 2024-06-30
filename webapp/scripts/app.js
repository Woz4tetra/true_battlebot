var ros;
var cageCornerPub;
var robotModePub;
var requestFieldPub;
var recordService;
var recordState = false;
var connection_status;
var connection_icon;
var opponent_template_names = [];
var connection_icons = {
    connected: "resources/check_circle_black_24dp.svg",
    disconnected: "resources/error_black_24dp.svg",
};

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

function initOpponentConfigurationPublisher() {
    opponentConfigurationPub = new ROSLIB.Topic({
        ros: ros,
        name: "/configured_opponents",
        messageType: "bw_interfaces/ConfiguredOpponents",
    });
    opponentConfigurationPub.advertise();
}

function initOpponentTemplateSubscriber() {
    opponentTemplateSub = new ROSLIB.Topic({
        ros: ros,
        name: "/opponent_templates",
        messageType: "bw_interfaces/ConfiguredOpponents",
    });

    opponentTemplateSub.subscribe(function (message) {
        updateOpponentConfiguationOptions(message.names);
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

function onSetNumOpponents(number_input) {
    if (number_input.value == "" || isNaN(number_input.value)) {
        return;
    }
    number_input.value = Math.max(1, Math.min(6, number_input.value));

    updateOpponentConfigurationGrid();
}

function getNumOpponents() {
    return document.getElementById("num-opponents").value;
}

function updateOpponentConfigurationGrid() {
    num_opponents = getNumOpponents();
    container = document.getElementById("opponent-configuration-grid");
    num_nodes = container.children.length;
    if (num_opponents === num_nodes) {
        return;
    }
    console.log(`Updating opponent configuration grid to ${num_opponents}`);
    for (var i = num_nodes; i < num_opponents; i++) {
        var node = document
            .getElementById("opponent-sub-configuration-template")
            .cloneNode(true);
        getOpponentConfigurationSubcomponent(
            node,
            "opponent-config-index"
        ).forEach((element) => {
            element.innerHTML = i + ": ";
        });
        container.appendChild(node);
    }
    for (var i = num_nodes; i > num_opponents; i--) {
        container.removeChild(container.lastChild);
    }
}

function getOpponentConfigurationSubcomponent(grid_container, node_id) {
    subcomponents = [];
    for (var i = 0; i < grid_container.children.length; i++) {
        component = grid_container.children[i];
        if (component.id === node_id) {
            subcomponents.push(component);
        }
    }
    return subcomponents;
}

function publishOpponentConfiguration() {
    container = document.getElementById("opponent-configuration-grid");
    keys = [];
    for (
        var child_index = 0;
        child_index < container.children.length;
        child_index++
    ) {
        keys.push(
            getOpponentConfigurationSubcomponent(
                container.children[child_index],
                "opponent-config-selector"
            )[0].value
        );
    }
    console.log(`Publishing opponent configuration: ${keys}`);
    opponentConfigurationPub.publish({ names: keys });
}

function updateOpponentConfiguationOptions(options) {
    if (opponent_template_names.toString() === options.toString()) {
        return;
    }
    console.log(`Updating opponent configuration options: ${options}`);
    opponent_template_names = options;
    container = document.getElementById("opponent-configuration-grid");
    for (
        var child_index = 0;
        child_index < container.children.length;
        child_index++
    ) {
        selector = getOpponentConfigurationSubcomponent(
            container.children[child_index],
            "opponent-config-selector"
        )[0];
        selector.innerHTML = "";
        options.forEach((option) => {
            var option_element = document.createElement("option");
            option_element.text = option;
            option_element.value = option;
            selector.add(option_element);
        });
    }
}

function preloadImages(array) {
    if (!preloadImages.list) {
        preloadImages.list = [];
    }
    var list = preloadImages.list;
    for (var i = 0; i < array.length; i++) {
        var img = new Image();
        img.onload = function () {
            var index = list.indexOf(this);
            if (index !== -1) {
                // remove image from the array once it's loaded
                // for memory consumption reasons
                list.splice(index, 1);
            }
        };
        list.push(img);
        img.src = array[i];
    }
}

function reconnectRosBridge() {
    preloadImages(Object.values(connection_icons));

    console.log("Reconnecting to ROS bridge");
    var robot_ip = location.hostname;
    if (ros) {
        ros.close();
    }
    let url = "ws://" + robot_ip + ":9090";

    ros = new ROSLIB.Ros({
        url: url,
    });
    console.log("Connecting to " + url);
    ros.connect(url);

    ros.on("connection", function () {
        connection_status.innerHTML = "Connected";
        connection_icon.src = connection_icons.connected;
        connection_icon.className = "filter-green";
        console.log("Connected to websocket server.");
    });

    ros.on("error", function (error) {
        console.log("Error connecting to websocket server: ", error);
        connection_status.innerHTML = "Error: " + error;
        connection_icon.src = connection_icons.disconnected;
        connection_icon.className = "filter-red";
    });

    ros.on("close", function () {
        console.log("Connection to websocket server closed.");
        connection_status.innerHTML = "Disconnected";
        connection_icon.src = connection_icons.disconnected;
        connection_icon.className = "filter-red";
    });

    initSummarySubscriber();
    initCageCornerPublisher();
    initRobotModePublisher();
    initRequestFieldPublisher();
    initRecordService();
    initTreeSnapshotSubscriber();
    initCarets();
    initHealthSummarySubscriber();
    initOpponentConfigurationPublisher();
    initOpponentTemplateSubscriber();
}

window.onload = function () {
    connection_status = document.getElementById("connection-status");
    connection_icon = document.getElementById("connection-icon");
    connection_icon.src = "resources/error_black_24dp.svg";
    connection_icon.className = "filter-red";
    connection_icon.ondragstart = function () {
        return false;
    };

    document.getElementById("num-opponents").value = 1;
    updateOpponentConfigurationGrid();
    updateOpponentConfiguationOptions(["default"]);

    reconnectRosBridge();

    console.log("App loaded");
};
