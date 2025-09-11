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
import ros2_mqtt.utils.handlers as handlers


class RelayRos2Mqtt(Node):
    def __init__(self):
        super().__init__('relay_ros2_mqtt')
        
        self.sleep_rate = 0.025
        self.omnicare_state, self.omnicare_battery = None, None
        self.rate = 10
        self.r = self.create_rate(self.rate)

        self.broker_address= self.declare_parameter("mqtt.broker_ip_address", '192.168.15.115').value
        self.MQTT_STATE_PUB_TOPIC = self.declare_parameter("mqtt.mqtt_state_pub_topic", '/esp32/navigation/state').value
        self.MQTT_BATTERY_PUB_TOPIC = self.declare_parameter("mqtt.mqtt_battery_pub_topic", '/esp32/status/battery').value
        self.MQTT_SUB_TOPIC = self.declare_parameter("mqtt.mqtt_sub_topic", '/esp32/omnicontroller').value
        self.ROS_STATE_SUB_TOPIC = self.declare_parameter("mqtt.state_sub_topic", '/omnicare/navigation/state').value
        self.ROS_BATTERY_SUB_TOPIC = self.declare_parameter("mqtt.battery_sub_topic", '/omnicare/status/battery').value


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
        self.get_logger().info(f'relay_ros2_mqtt:: ROS_BATTERY_SUB_TOPIC = {self.ROS_BATTERY_SUB_TOPIC}')

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


        
        # Subscriber to the ROS topics
        self.create_subscription(
            String,
            self.ROS_STATE_SUB_TOPIC,
            self.status_callback,
            qos_profile=qos_profile_system_default)
        
        self.create_subscription(
            String,
            self.ROS_BATTERY_SUB_TOPIC,
            self.battery_callback,
            qos_profile=qos_profile_system_default)



    
        # Timer que envia a cada 0.1s (2 Hz)
        # self.timer = self.create_timer(0.1, self.publish_to_mqtt)

        
    def status_callback(self, msg):
        self.get_logger().info(f"State msg: {msg.data}")
        self.omnicare_state = msg.data  # só salva a última
        self.mqttclient.publish(self.MQTT_STATE_PUB_TOPIC,self.omnicare_state,qos=0, retain=False)

    def battery_callback(self, msg):
        self.get_logger().info(f"Battery msg: {msg.data}")
        self.omnicare_battery = msg.data  # só salva a última
        self.mqttclient.publish(self.MQTT_BATTERY_PUB_TOPIC,self.omnicare_battery,qos=0, retain=False)
        
    # def publish_to_mqtt(self):
    #     if self.omnicare_battery is None:
    #         return
        
    #     self.get_logger().info(f"Publishing battery to MQTT: {self.omnicare_battery}")
    #     self.mqttclient.publish(self.MQTT_PUB_TOPIC,self.omnicare_battery,qos=0, retain=False)


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