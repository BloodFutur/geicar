import rclpy
from rclpy.node import Node
from typing import Any, Dict, Optional
from dataclasses import dataclass
import json
import base64
import os
import time
from sensor_msgs.msg import NavSatFix, CompressedImage
from interfaces.msg import SystemCheck, GeneralData, JoystickOrder
import paho.mqtt.client as mqtt

@dataclass
class MqttConfig:
    broker_address: str = "srv665994.hstgr.cloud"
    mqtt_port: int = 443
    mqtt_username: str = "geicar"
    mqtt_password: str = "geicar"
    transport: str = "websockets"
    max_fps: int = 20


class MessageHandler:
    @staticmethod
    def to_dict(obj: Any) -> Dict:
        if isinstance(obj, dict):
            return dict((key.lstrip("_"), MessageHandler.todict(val)) for key, val in obj.items())
        elif hasattr(obj, "_ast"):
            return todict(obj._ast())
        elif hasattr(obj, "__iter__") and not isinstance(obj, str):
            return [todict(v) for v in obj]
        elif hasattr(obj, '__dict__'):
            return todict(vars(obj))
        elif hasattr(obj, '__slots__'):
            return todict(dict((name, getattr(obj, name)) for name in getattr(obj, '__slots__')))
        return obj


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
        client.publish("vehicle/status", json.dumps(status), retain=True)
    else:
        print("Failed to connect, return code %d\n", rc)
 
 
def on_disconnect(client, userdata, rc):
    status = {
        "status": "Disconnected"
    }
    print("Disconnected from MQTT Broker!")
    client.publish("vehicle/status", json.dumps(status), retain=True)
       
    
class Ros2MqttClient(Node):
    def __init__(self, config: Optional[MqttConfig] = None):
        super().__init__('ros2_mqtt_client')
        self.config = config or MqttConfig()
        self.setup_mqtt_client()
        self.setup_ros2_subscribers()
        self.last_publish_time = 0
        
    def setup_mqtt_client(self) -> None:
        self.mqtt_client = mqtt.Client(transport=self.config.transport)
        self.mqtt_client.username_pw_set(self.config.mqtt_username, self.config.mqtt_password)
        self.mqtt_client.tls_set()
        self.mqtt_client.on_connect = on_connect
        self.mqtt_client.on_disconnect = on_disconnect
        
        # try:
        self.mqtt_client.connect(self.config.broker_address, self.config.mqtt_port)
        self.mqtt_client.loop_start()
        # except Exception as e:
        #     self.get_logger().error(f"Error connecting to MQTT broker: {e}")
        #     raise
        
    def setup_ros2_subscribers(self) -> None:
        self.ros2_subscribers = {
            'gps_vehicle': ('gps/fix', NavSatFix, self.gps_listener_callback, "gps"),
            'plate_img': ('plate_detection/compressed', CompressedImage, self.plate_detection_listener_cb, "plate_detection"),
            'system_check_report': ('system_check', SystemCheck, self.system_check_listener_cb, "system_check"),
            'general_data': ('general_data', GeneralData, self.general_data_listener_cb, "general_data"),
            'mode': ('joystick_order', JoystickOrder, self.joystick_order_listener_cb, "joystick_order")
        }
        
        for _, (topic, msg_type, callback, _) in self.ros2_subscribers.items():
            self.create_subscription(msg_type, topic, callback, 10)
            
    def publish_message(self, topic: str, message: Any, retain: bool = True) -> None:
        try:
            json_msg = MessageHandler.to_dict(message)
            self.mqtt_client.publish(topic, json.dumps(json_msg), retain=retain)
            self.get_logger().debug(f"Published message to {topic}")
        except Exception as e:
            self.get_logger().error(f"Error publishing message to {topic}: {e}")
         
    def gps_listener_callback(self, msg):
        # Use ros2_subscribers to publish in mqtt_topic
        mqtt_topic = self.ros2_subscribers['gps'][3]
        self.publish_message(mqtt_topic, msg)

    def plate_detection_listener_cb(self, msg):
        current_time = time.time()
        if current_time - self.last_publish_time < 1.0 / self.max_fps:
            return  # Skip publishing if not enough time has passed
        
        self.last_publish_time = current_time
        transformed_json = MessageHandler.to_dict(msg)
        # Convert to base64 so that the web browser knows how to use this data
        transformed_json["data"] = base64.b64encode(msg.data).decode('utf-8')
        self.publish_message(self.ros2_subscribers['plate_img'][3], transformed_json)

    def system_check_listener_cb(self, msg):
        if msg.report:
            self.publish_message(self.ros2_subscribers['system_check_report'][3], msg)
    
    def general_data_listener_cb(self, msg):
        self.publish_message(self.ros2_subscribers['general_data'][3], msg)
        
    def joystick_order_listener_cb(self, msg):
        self.publish_message(self.ros2_subscribers['mode'][3], msg)

    def destroy_node(self):
        # Stop MQTT loop before destroying the node
        self.mqtt_client.loop_stop()
        super().destroy_node()


def get_uptime():
    with open('/proc/uptime', 'r') as f:
        uptime_seconds = float(f.readline().split()[0])
    uptime_string = time.strftime('%H:%M:%S', time.gmtime(uptime_seconds))
    return uptime_string
    

print(f"System uptime: {get_uptime()}")

def main(args=None):
    rclpy.init(args=args)
    config = MqttConfig()
    client = Ros2MqttClient(config)
    
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info('Shutting down mqtt client...')
    except Exception as e:
        client.get_logger().error(f'Error: {e}')
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
