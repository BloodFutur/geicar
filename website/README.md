# How to use this website ?

- Launch your ROS2 nodes
- Launch rosbridge_server (port 9090)
- Open the firewall on port 8080 (default http port) (use `ufw`)
- Execute `python3 -m http.server` in the `website` folder to start a server
- From your browser, go to `192.168.1.1/index.html`(ethernet) or `10.105.1.168/index.html`(eduroam)