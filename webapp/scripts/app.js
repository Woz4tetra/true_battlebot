var ros;
var cageCornerPub;
var robotModePub;
var requestFieldPub;
var recordService;
var recordState = false;
var connection_status,
    teleop_controller_status,
    teleop_connected_status,
    teleop_is_ready_status,
    teleop_is_armed_status,
    field_stable_status;
var connection_icon,
    teleop_controller_icon,
    teleop_connected_icon,
    teleop_is_ready_icon,
    teleop_is_armed_icon,
    field_stable_icon;
var opponent_template_names = [];
var connection_icons = {
    connected: "resources/check_circle_black_24dp.svg",
    disconnected: "resources/error_black_24dp.svg",
};
var lastPublishedCorner = 0;

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
    lastPublishedCorner = type;
    console.log(`Publishing corner ${type}`);
    cageCornerPub.publish({ type: type });
}

function republishCageCorner() {
    publishCageCorner(lastPublishedCorner);
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

function initTelemetrySubscriber() {
    var listener = new ROSLIB.Topic({
        ros: ros,
        name: "/mini_bot/telemetry_status",
        messageType: "bw_interfaces/TelemetryStatus",
    });

    listener.subscribe(function (message) {
        document.getElementById(
            "teleop-battery-voltage"
        ).innerHTML = `Battery: ${message.battery_voltage.toFixed(2)} V`;
        document.getElementById("teleop-mode").innerHTML = message.flight_mode;
        setConnectionIconState(
            teleop_controller_icon,
            message.controller_connected
        );
        setConnectionIconState(teleop_connected_icon, message.is_connected);
        setConnectionIconState(teleop_is_ready_icon, message.is_ready);
        setConnectionIconState(teleop_is_armed_icon, message.is_armed);
        teleop_controller_status.innerHTML = message.controller_connected
            ? "Controller Connected"
            : "Controller Disconnected";
        teleop_connected_status.innerHTML = message.is_connected
            ? "Telemetry Connected"
            : "Telemetry Disconnected";
        teleop_is_ready_status.innerHTML = message.is_ready
            ? "Is ready"
            : "Not ready";
        teleop_is_armed_status.innerHTML = message.is_armed
            ? "Is armed"
            : "Not armed";
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

function initFieldStableSubscriber() {
    fieldStableSub = new ROSLIB.Topic({
        ros: ros,
        name: "/filter/field/is_settled",
        messageType: "std_msgs/Bool",
    });

    fieldStableSub.subscribe(function (message) {
        setConnectionIconState(field_stable_icon, message.data);
        field_stable_status.innerHTML = message.data
            ? "Field ready"
            : "Field not found";
    });
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

function initConnectionIcon(element_id) {
    icon_element = document.getElementById(element_id);
    icon_element.src = "resources/error_black_24dp.svg";
    icon_element.className = "filter-red";
    icon_element.ondragstart = function () {
        return false;
    };
    return icon_element;
}

function setConnectionIconState(icon, state, text) {
    if (state) {
        icon.src = connection_icons.connected;
        icon.className = "filter-green";
    } else {
        icon.src = connection_icons.disconnected;
        icon.className = "filter-red";
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
        console.log("Connected to websocket server.");
        connection_status.innerHTML = "Connected";
        setConnectionIconState(connection_icon, true);
    });

    ros.on("error", function (error) {
        console.log("Error connecting to websocket server: ", error);
        connection_status.innerHTML = "Error: " + error;
        setConnectionIconState(connection_icon, false);
    });

    ros.on("close", function () {
        console.log("Connection to websocket server closed.");
        connection_status.innerHTML = "Disconnected";
        setConnectionIconState(connection_icon, false);
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
    initTelemetrySubscriber();
    initFieldStableSubscriber();
}

document.addEventListener("keypress", function (event) {
    if (event.key === "r") {
        reconnectRosBridge();
    }
});

document.addEventListener("DOMContentLoaded", function () {
    const resizable = function (resizer) {
        const direction =
            resizer.getAttribute("data-direction") || "horizontal";
        const prevSibling = resizer.previousElementSibling;
        const nextSibling = resizer.nextElementSibling;

        // The current position of mouse
        let x = 0;
        let y = 0;
        let prevSiblingHeight = 0;
        let prevSiblingWidth = 0;

        // Handle the mousedown event
        // that's triggered when user drags the resizer
        const mouseDownHandler = function (e) {
            // Get the current mouse position
            x = e.clientX;
            y = e.clientY;
            const rect = prevSibling.getBoundingClientRect();
            prevSiblingHeight = rect.height;
            prevSiblingWidth = rect.width;

            // Attach the listeners to document
            document.addEventListener("mousemove", mouseMoveHandler);
            document.addEventListener("mouseup", mouseUpHandler);
        };

        const mouseMoveHandler = function (e) {
            // How far the mouse has been moved
            const dx = e.clientX - x;
            const dy = e.clientY - y;

            switch (direction) {
                case "vertical":
                    const h =
                        ((prevSiblingHeight + dy) * 100) /
                        resizer.parentNode.getBoundingClientRect().height;
                    prevSibling.style.height = h + "%";
                    break;
                case "horizontal":
                default:
                    const w =
                        ((prevSiblingWidth + dx) * 100) /
                        resizer.parentNode.getBoundingClientRect().width;
                    prevSibling.style.width = w + "%";
                    break;
            }

            const cursor =
                direction === "horizontal" ? "col-resize" : "row-resize";
            resizer.style.cursor = cursor;
            document.body.style.cursor = cursor;

            prevSibling.style.userSelect = "none";
            prevSibling.style.pointerEvents = "none";

            nextSibling.style.userSelect = "none";
            nextSibling.style.pointerEvents = "none";
        };

        const mouseUpHandler = function () {
            resizer.style.removeProperty("cursor");
            document.body.style.removeProperty("cursor");

            prevSibling.style.removeProperty("user-select");
            prevSibling.style.removeProperty("pointer-events");

            nextSibling.style.removeProperty("user-select");
            nextSibling.style.removeProperty("pointer-events");

            // Remove the handlers of mousemove and mouseup
            document.removeEventListener("mousemove", mouseMoveHandler);
            document.removeEventListener("mouseup", mouseUpHandler);
        };

        // Attach the handler
        resizer.addEventListener("mousedown", mouseDownHandler);
    };

    // Query all resizers
    document.querySelectorAll(".resizer").forEach(function (ele) {
        resizable(ele);
    });
});

window.onload = function () {
    connection_status = document.getElementById("connection-status");
    teleop_controller_status = document.getElementById(
        "teleop-controller-status"
    );
    teleop_connected_status = document.getElementById(
        "teleop-connected-status"
    );
    teleop_is_ready_status = document.getElementById("teleop-is-ready-status");
    teleop_is_armed_status = document.getElementById("teleop-is-armed-status");

    teleop_controller_status.innerHTML = "No controller";
    teleop_connected_status.innerHTML = "No connection";
    teleop_is_ready_status.innerHTML = "Not ready";
    teleop_is_armed_status.innerHTML = "Not armed";

    connection_icon = initConnectionIcon("connection-icon");
    teleop_controller_icon = initConnectionIcon("teleop-controller-icon");
    teleop_connected_icon = initConnectionIcon("teleop-connected-icon");
    teleop_is_ready_icon = initConnectionIcon("teleop-is-ready-icon");
    teleop_is_armed_icon = initConnectionIcon("teleop-is-armed-icon");

    field_stable_status = document.getElementById("field-stable-status");
    field_stable_status.innerHTML = "Field not found";
    field_stable_icon = initConnectionIcon("field-stable-icon");

    document.getElementById("num-opponents").value = 1;
    updateOpponentConfigurationGrid();
    updateOpponentConfiguationOptions(["default"]);

    reconnectRosBridge();

    console.log("App loaded");
};
