// Configuration
const DEFAULT_ROSBRIDGE_IP = "10.105.1.168";
const ROSBRIDGE_PORT = 9090;
let rosbridgeIP = DEFAULT_ROSBRIDGE_IP;

let ros = initializeROSConnection(rosbridgeIP);

function initializeROSConnection(ip) {
    return new ROSLIB.Ros({
        url: `ws://${ip}:${ROSBRIDGE_PORT}`,
    });
}

function updateROSIP(input) {
    rosbridgeIP = input.value;
    ros.close();
    ros = initializeROSConnection(rosbridgeIP);
    attachROSEventHandlers(ros);
    console.log(`Updated ROSBridge IP to: ${rosbridgeIP}`);
}

function attachROSEventHandlers(rosInstance) {
    const statusEl = document.getElementById("status");
    
    rosInstance.on("connection", () => {
        console.log("Connected to rosbridge websocket server.");
        updateStatusUI(statusEl, "Connected", "connected");
    });

    rosInstance.on("error", (error) => {
        console.error("Error connecting to websocket server:", error);
        updateStatusUI(statusEl, "Connection Error", "error");
    });

    rosInstance.on("close", () => {
        console.log("Connection to websocket server close.");
        updateStatusUI(statusEl, "Connection Closed", "closed");
    });
}

function updateStatusUI(element, message, statusClass) {
    if (element) {
        element.textContent = message;
        element.className = statusClass;
    } else {
        console.warn("Status element not found!");
    }
}

attachROSEventHandlers(ros);

function createTopicListener(rosInstance, topicName, messageType, callback) {
    return new ROSLIB.Topic({
        ros: rosInstance,
        name: topicName,
        messageType: messageType,
    }).subscribe(callback);
}

const gpsTopicListener = createTopicListener(
    ros,
    "/gps/fix",
    "sensor_msgs/msg/NavSatFix",
    (message) => {
        handleGPSMessage(message);
    }
);

function handleGPSMessage(message) {
    const ul = document.getElementById("messages");
    if(!ul) {
        console.error("Messages element not found!");
    }

    const coords = `${message.latitude}; ${message.longitude}`;
    const newMessage = document.createElement("li");
    newMessage.appendChild(document.createTextNode(coords));
    ul.appendChild(newMessage);

    if (typeof L !== "undefined" && typeof map !== "undefined") {
        L.circle([message.latitude, message.longitude], {
            color: "red",
            fillColor: "#f03",
            fillOpacity: 0.5,
            radius: 0.2,
        }).addTo(map);
    } else {
        console.warn("Leaflet map is not defined.");
    }
}