#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String

import paho.mqtt.client as mqtt
from typing import Dict
import json

from ros2_mqtt.utils.launch_process import ROS2LaunchProc
from ros2_mqtt.utils.ina219 import INA219
import ros2_mqtt.utils.handlers as handlers


class RelayRos2Mqtt(Node):
    def __init__(self):
        super().__init__('relay_ros2_mqtt')
        
        # Parameters
        self.sleep_rate = 0.025
        self.omnicare_state, self.omnicare_battery = None, None
        self.rate = 10
        self.r = self.create_rate(self.rate)

        # MQTT Parameters from yaml file
        self.broker_address= self.declare_parameter("mqtt.broker_ip_address", '192.168.15.115').value
        self.MQTT_STATE_PUB_TOPIC = self.declare_parameter("mqtt.mqtt_state_pub_topic", '/esp32/navigation/state').value
        self.MQTT_BATTERY_PUB_TOPIC = self.declare_parameter("mqtt.mqtt_battery_pub_topic", '/esp32/status/battery').value
        self.MQTT_SUB_TOPIC = self.declare_parameter("mqtt.mqtt_sub_topic", '/esp32/omnicontroller').value

        # ROS Topics from yaml file
        self.ROS_STATE_SUB_TOPIC = self.declare_parameter("mqtt.state_sub_topic", '/omnicare/navigation/state').value
        self.ROS_BATTERY_PUB_TOPIC = self.declare_parameter("mqtt.battery_sub_topic", '/omnicare/status/battery').value

        # Initialize MQTT client
        self.mqttclient = mqtt.Client()
        self.mqttclient.connect(self.broker_address) 
        self.mqttclient.subscribe(self.MQTT_SUB_TOPIC)
        self.mqttclient.on_message = self.on_message
        self.mqttclient.loop_start()


        self.get_logger().info('relay_ros2_mqtt:: started...')
        self.get_logger().info(f'relay_ros2_mqtt:: broker_address = {self.broker_address}')
        self.get_logger().info(f'relay_ros2_mqtt:: MQTT_STATUS_PUB_TOPIC = {self.MQTT_STATE_PUB_TOPIC}')
        self.get_logger().info(f'relay_ros2_mqtt:: MQTT_BATTERY_PUB_TOPIC = {self.MQTT_BATTERY_PUB_TOPIC}')
        self.get_logger().info(f'relay_ros2_mqtt:: ROS_STATE_SUB_TOPIC = {self.ROS_STATE_SUB_TOPIC}')
        self.get_logger().info(f'relay_ros2_mqtt:: ROS_BATTERY_PUB_TOPIC = {self.ROS_BATTERY_PUB_TOPIC}')

        # Parameters of the controller
        self.button_a = self.declare_parameter("controller.button.a", "").value
        self.button_b = self.declare_parameter("controller.button.b", "").value
        self.button_x = self.declare_parameter("controller.button.x", "").value
        self.button_y = self.declare_parameter("controller.button.y", "").value
        self.button_start = self.declare_parameter("controller.button.start", "").value

        self.button_right = self.declare_parameter("controller.teleop.right", 0.5).value
        self.button_left = self.declare_parameter("controller.teleop.left", 0.5).value
        self.button_up = self.declare_parameter("controller.teleop.up", 0.5).value
        self.button_down = self.declare_parameter("controller.teleop.down", 0.5).value

        # Get all button parameters
        self.list_of_buttons_params = self.get_parameters_by_prefix("controller.button")
                
        # Iterate on the parameters and add their own ROS2LaunchProc instance
        self.launchers: Dict[str, ROS2LaunchProc] = {}
        for button, param in self.list_of_buttons_params.items():
            value = param.value

            # Separe pkg and launch file
            if value != "":
                type, pkg, executable = value.split(".", 2)

                self.get_logger().info(
                    f"controller.{button}: type={type}, pkg={pkg}, executable={executable}"
                )

                # Create and store on the dictionary
                proc = ROS2LaunchProc(type, pkg, executable)
            else:
                proc = ROS2LaunchProc("", "", "")

            self.launchers[button] = proc
            
        self.handlers = handlers.build_handlers(self.launchers, 
                                                self.create_publisher(Twist , 'cmd_vel', 10), 
                                                self,
                                                self.get_logger())


        read_voltage = self.declare_parameter("ina219.read_voltage", "").value # Read read_voltage parameter from yaml file
        self.INA219 = INA219()  # Initialize INA219 sensor

        # Pubs
        self.battery_pub = self.create_publisher(
            String, 
            self.ROS_BATTERY_PUB_TOPIC, 
            qos_profile_system_default)
        
        if read_voltage:
            self.timer = self.create_timer(1.0, self.publish_battery_status) # Timer that sends every 1.0s (1 Hz)

        # Subs
        self.create_subscription(
            String,
            self.ROS_STATE_SUB_TOPIC,
            self.status_callback,
            qos_profile=qos_profile_system_default)
        

        
    def status_callback(self, msg):
        self.get_logger().info(f"State msg: {msg.data}")
        self.omnicare_state = msg.data  # só salva a última
        self.mqttclient.publish(self.MQTT_STATE_PUB_TOPIC,self.omnicare_state,qos=0, retain=False)

    def _map_battery(self,voltage):
        if voltage >= 24.0: return 100.0
        elif voltage <= 18.0: return 0.0
        return ((voltage - 18.0) / (24.0 - 18.0)) * 100.0
    
    def publish_battery_status(self):
        msg = String()
        voltage = self.INA219.read_voltage()
        voltage_percentage = int(self._map_battery(voltage))
        
        msg.data = str(voltage_percentage)
        self.battery_pub.publish(msg)
        self.mqttclient.publish(self.MQTT_BATTERY_PUB_TOPIC,voltage_percentage,qos=0, retain=False)        
        self.get_logger().info(f"Battery status: {voltage_percentage}")

    # Callback when a message is received
    def on_message(self,client, userdata, msg):
        msg = msg.payload.decode()
        self.get_logger().info(f"Msg: {msg}")
        handler = self.handlers.get(msg, lambda: handlers.unknown(self.get_logger(), msg))()
        

def main(args=None):
    

    rclpy.init(args=args)
    try:
        relay_ros2_mqtt = RelayRos2Mqtt()
        rclpy.spin(relay_ros2_mqtt)
    except rclpy.exceptions.ROSInterruptException:
        pass

    relay_ros2_mqtt.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()