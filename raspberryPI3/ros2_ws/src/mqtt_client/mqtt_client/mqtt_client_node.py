import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import CompressedImage
import json
import paho.mqtt.client as mqtt

# MQTT Settings
BROKER_ADDRESS = "srv665994.hstgr.cloud"
MQTT_PORT = 1883
MQTT_USERNAME = "geicar"
MQTT_PASSWORD = "geicar"


def todict(obj):        
    if isinstance(obj, dict):
        return dict((key.lstrip("_"), todict(val)) for key, val in obj.items())
    elif hasattr(obj, "_ast"):
        return todict(obj._ast())
    elif hasattr(obj, "__iter__") and not isinstance(obj, str):
        return [todict(v) for v in obj]
    elif hasattr(obj, '__dict__'):
        return todict(vars(obj))
    elif hasattr(obj, '__slots__'):
        return todict(dict((name, getattr(obj, name)) for name in getattr(obj, '__slots__')))
    return obj


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        # Send that the vehicle is connected
        status = {
            "status": "Connected"
        }
        client.publish("vehicle/status", json.dumps(status))
    else:
        print("Failed to connect, return code %d\n", rc)
 
 
def on_disconnect(client, userdata, rc):
    status = {
        "status": "Disconnected"
    }
    client.publish("vehicle/status", json.dumps(status))
            
class Ros2MqttClient(Node):
    def __init__(self):
        super().__init__('ros2_mqtt_client')
        self.sub_gps = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_listener_callback,
            10
        )
        self.sub_gps = self.create_subscription(
            CompressedImage,
            '/plate_detection/compressed',
            self.plate_detection_listener_cb,
            10
        )
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        
        self.mqtt_client.on_connect = on_connect
        self.mqtt_client.on_disconnect = on_disconnect
        
        self.mqtt_client.connect(BROKER_ADDRESS, MQTT_PORT)
        self.mqtt_client.loop_start()
       
    def gps_listener_callback(self, msg):
        self.get_logger().info("Publishing gps")
        json_message = todict(msg)
        self.mqtt_client.publish("gps", json.dumps(json_message))

    def plate_detection_listener_cb(self, msg):
        self.get_logger().info("Publishing image")
        json_message = todict(msg)
        self.mqtt_client.publish("plate_detection", json.dumps(json_message))

    def destroy_node(self):
        # Stop MQTT loop before destroying the node
        self.mqtt_client.loop_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    client = Ros2MqttClient()
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info('Shutting down mqtt client...')
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
