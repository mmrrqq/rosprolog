#!/usr/bin/env python3

import rclpy
import json
import importlib

from rclpy.node import Node
from rclpy.client import Client
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rosprolog.srv import JSONWrapper
from rosprolog.msg import MessageJSON


class JSONNode(object):
    """
    A node that offers a service to interact
    with ROS via JSON encoded messages.
    """

    def __init__(self, node: Node):
        self.node = node
        self.update_positions_srv = node.create_service(
            JSONWrapper, "json_wrapper", self.json_wrapper_cb
        )
        self.subscription_publisher = node.create_publisher(
            MessageJSON, "/json/message", 100
        )
        self.node.get_logger().info("json_wrapper service is running")
        # handles to all imported ROS artifacts
        self.ros_modules = {}
        self.ros_services: dict[str, Client] = {}
        self.ros_pubisher = {}
        self.ros_subscriber = {}
        self.latching_qos = QoSProfile(
            depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

    def json_wrapper_cb(self, wrapper_request):
        request_data = json.loads(wrapper_request.json_data)
        if wrapper_request.mode == "service":
            return self.service(request_data)
        # elif wrapper_request.mode == 'action':
        #    return self.action(request_data)
        elif wrapper_request.mode == "publish":
            return self.publish(request_data)
        elif wrapper_request.mode == "subscribe":
            return self.subscribe(request_data)
        elif wrapper_request.mode == "unsubscribe":
            return self.unsubscribe(request_data)
        else:
            pass

    def subscribe(self, msg_data):
        # TODO: won't work at the moment when there are
        #       subscribers of the same topic
        msg_cls = self.get_msg_class(msg_data["msg_path"])
        self.get_subscriber(msg_data["topic_name"], msg_cls)
        return JSONWrapper.Response()

    def unsubscribe(self, msg_data):
        try:
            topic_name = msg_data["topic_name"]
            subscriber = self.ros_subscriber[topic_name]
            subscriber.unregister()
            del self.ros_subscriber[topic_name]
        except:
            pass
        return JSONWrapper.Response()

    def publish(self, msg_data):
        (msg, msg_cls) = self.decode_json_message(msg_data["msg_path"], msg_data)
        publisher = self.get_publisher(msg_data["topic_name"], msg_cls)
        publisher.publish(msg)
        return JSONWrapper.Response()

    def service(self, request_data):
        # TODO: also include a status field in response
        srv_module = self.get_service_module(request_data["service_path"])
        module_name = request_data["service_path"].split("/")[-1]
        srv_cls = getattr(srv_module, module_name)
        req_cls = getattr(srv_module, module_name + "Request")
        res_cls = getattr(srv_module, module_name + "Response")
        # instantiate the message
        request = req_cls()
        self.assign_slots(request, request_data)
        # get a service handle
        srv = self.get_service(request_data["service_name"], srv_cls)
        # send it
        response = JSONWrapper.Response()
        try:
            srv_response = srv.call_async(request)
            rclpy.spin_until_future_complete(self.node, srv_response)
            response.json_data = self.read_slots(res_cls, srv_response)
        except:
            response.json_data = ""
        # return the json encoded response
        return response

    # def action(self,request_data):
    #    # TODO implement
    #    return JSONWrapperResponse()

    #######################
    ######### Finiding ROS artifacts

    def get_ros_module(self, type_string, import_path):
        """
        dynamic import of ROS modules by type string, e.g. 'std_srvs/Trigger'.
        """
        try:
            module = self.ros_modules[type_string]
        except:
            module = importlib.import_module(import_path)
            self.ros_modules[type_string] = module
        return module

    def get_service_module(self, type_string):
        x = type_string.split("/")
        return self.get_ros_module(type_string, ".".join([x[0], "srv", "_" + x[1]]))

    def get_message_module(self, type_string):
        x = type_string.split("/")
        return self.get_ros_module(type_string, ".".join([x[0], "msg", "_" + x[1]]))

    def get_service(self, srv_name, srv_type) -> Client:
        try:
            srv = self.ros_services[srv_name]
        except:
            srv = self.node.create_client(srv_type, srv_name)
            while not srv.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info("service not available, waiting...")
            self.ros_services[srv_name] = srv
        return srv

    def get_publisher(self, topic_name, msg_class):
        try:
            publisher = self.ros_pubisher[topic_name]
        except:
            # NOTE: latching through latching_qos definition
            publisher = self.node.create_publisher(
                msg_class, topic_name, self.latching_qos
            )
            self.ros_pubisher[topic_name] = publisher
        return publisher

    def get_subscriber(self, topic_name, msg_class):
        try:
            subscriber = self.ros_subscriber[topic_name]
        except:

            def callback(msg):
                msg_json = self.read_slots(msg_class, msg)
                msg_ros = MessageJSON(topic_name=topic_name, json_data=msg_json)
                self.subscription_publisher.publish(msg_ros)

            subscriber = self.node.create_subscription(msg_class, topic_name, callback)
            self.ros_subscriber[topic_name] = subscriber
        return subscriber

    #######################
    ######### Type checking

    def is_primitive_type(self, type_path):
        # TODO: what about time?
        return type_path in [
            "bool",
            "float32",
            "float64",
            "int8",
            "int16",
            "int32",
            "int64",
            "uint8",
            "uint16",
            "uint32",
            "uint64",
            "string",
        ]

    def is_message_array_type(self, type_path):
        if not type_path.startswith("array("):
            return False
        array_type = type_path[6:-1]
        return not self.is_primitive_type(array_type)

    def is_primitive_array_type(self, type_path):
        if not type_path.startswith("array("):
            return False
        array_type = type_path[6:-1]
        return self.is_primitive_type(array_type)

    def is_message_type(self, type_path):
        if type_path.startswith("array("):
            return False
        return not self.is_primitive_type(type_path)

    def is_string_type(self, type_path):
        return type_path in ["string", "array(string)"]

    #######################
    ######### Decoding JSON into ROS messages

    def decode_json_value(self, type_path, value):
        if self.is_primitive_type(type_path):
            if self.is_string_type(type_path):
                return str(value.encode("utf-8"))
            else:
                return value
        elif self.is_primitive_array_type(type_path):
            if self.is_string_type(type_path):
                return list(map(lambda x: str(x.encode("utf-8")), value))
            else:
                return value
        elif self.is_message_array_type(type_path):
            element_type = type_path[6:-1]
            return self.decode_json_message_array(element_type, value)
        else:
            return self.decode_json_message(type_path, value)[0]

    def get_msg_class(self, type_path):
        msg_module = self.get_message_module(type_path)
        module_name = type_path.split("/")[-1]
        return getattr(msg_module, module_name)

    def decode_json_message(self, type_path, msg_json):
        msg_class = self.get_msg_class(type_path)
        msg = msg_class()
        self.assign_slots(msg, msg_json)
        return (msg, msg_class)

    def decode_json_message_array(self, type_path, array_json):
        out = []
        for x in array_json:
            out.append(self.decode_json_message(type_path, x)[0])
        return out

    def assign_slots(self, msg, json_data):
        for name in json_data:
            json_value = json_data[name]
            if not type(json_value) is list:
                continue
            [type_path, value] = json_data[name]
            if type_path in ["time"]:
                setattr(msg, name, rclpy.Time.from_sec(value))
            elif hasattr(msg, name):
                setattr(msg, name, self.decode_json_value(type_path, value))

    #######################
    ######### Encoding of ROS message into JSON

    def read_slots(self, response_cls, response_message):
        attributes = [
            attr
            for attr in dir(response_cls)
            if not callable(getattr(response_cls, attr)) and not attr.startswith("_")
        ]
        out = {}
        for a in attributes:
            out[a] = getattr(response_message, a)
        return json.dumps(out)


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("json_ros")
    json_node = JSONNode(node)
    rclpy.spin(node)
