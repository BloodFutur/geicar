// Create ros object to communicate over the Rosbridge connection
const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

const statusEl = document.getElementById("status");
// When the Rosbridge server connects, fill the span with id "status" with "successful"
ros.on("connection", () => {
    console.log('Connected to rosbridge websocket server.');
    statusEl.textContent = 'Connected';
    statusEl.className = 'connected';
});

// When the Rosbridge server experiences an error, fill the "status" span with the returned error
ros.on("error", (error) => {
    console.log('Error connecting to websocket server:', error);
    statusEl.textContent = 'Connection Error';
    statusEl.className = 'error';
});

// When the Rosbridge server shuts down, fill the "status" span with "closed"
ros.on("close", () => {
    console.log('Connection to websocket server closed.');
    statusEl.textContent = 'Connection Closed';
    statusEl.className = 'closed';
});

// It is to show that the website can communicate with ROS
// In the future it will be replaced by real topics
// Create a listener for /my_topic
const my_topic_listener = new ROSLIB.Topic({
    ros,
    name: "/my_topic",
    messageType: "std_msgs/String",
});

// When we receive a message on /my_topic, add its data as a list item to the "messages" ul
my_topic_listener.subscribe((message) => {
    const ul = document.getElementById("messages");
    const newMessage = document.createElement("li");
    newMessage.appendChild(document.createTextNode(message.data));
    ul.appendChild(newMessage);
});