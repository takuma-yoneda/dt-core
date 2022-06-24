#!/usr/bin/env python3


import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTParam, DTROS, NodeType
from duckietown_msgs.msg import BoolStamped, StopLineReading
from duckietown_msgs.srv import ChangePattern
from std_msgs.msg import String


class ObstacleDetectionNode(DTROS):
    """Subscribe to vehicle_detection and pedestrian_detection,
    and publish an 'or' of those booleans."""
    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(ObstacleDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self.bridge = CvBridge()
        self._interval = 1. / 20  # Seconds
        self._header = None
        self.topics =  [
            ('~vehicle_detection/detection', BoolStamped, self.cb_vehicle_detection),
            ('~pedestrian_detection/detection', BoolStamped, self.cb_pedestrian_detection)
        ]  # Vehicle and Pedestrian
        self.topics_state = {'vehicle': False, 'pedestrian': False}  # Updated by callbacks

        subscribers = []
        for topic, type, callback in self.topics:
            subscribers.append(
                rospy.Subscriber(topic, type, callback, queue_size=1)
            )
        self._subscribers = subscribers

        self.pub_stopped_flag = rospy.Publisher("~stopped", BoolStamped, queue_size=1)
        self.pub_virtual_stop_line = rospy.Publisher("~virtual_stop_line", StopLineReading, queue_size=1)

        # Publish in a fixed frequency with a ros timer
        # Initialize Timer
        import functools
        self._timer = rospy.Timer(
            rospy.Duration(self._interval),
            self.maybe_publish_stop_line
        )
        self.last_led_state = None
        self.changePattern = rospy.ServiceProxy("~set_pattern", ChangePattern)


    def cb_vehicle_detection(self, msg):
        # Message: bool
        self.topics_state['vehicle'] = msg.data
        self._header = msg.header

    def cb_pedestrian_detection(self, msg):
        self.topics_state['pedestrian'] = msg.data
        self._header = msg.header

    def maybe_publish_stop_line(self, event):
        self.logdebug(f'self.topics_state: {self.topics_state}')
        header = self._header
        if header is None:
            return

        if any(self.topics_state.values()):
            self.publish_stop_line_msg(
                header=header,
                detected=True,
                at=True,
                # x=distance_to_vehicle,
                x=0.  # HACK: Hardcoded
            )
            # Try turning on the hazard light
            try:
                self.trigger_led_hazard_light(
                    detection=False, stopped=True
                )
            except Exception:
                print('WARN: turning on LED hazard light failed...')  # TODO: replace it with ros-native log
                self.trigger_led_hazard_light(detection=False, stopped=True)
        else:

            # publish empty messages
            self.publish_stop_line_msg(header=header)



    def trigger_led_hazard_light(self, detection, stopped):
        """
        Publish a service message to trigger the hazard light at the back of the robot
        """
        msg = String()

        if stopped:
            msg.data = "OBSTACLE_STOPPED"
            if msg.data != self.last_led_state:
                self.changePattern(msg)
            self.last_led_state = msg.data
        elif detection:
            msg.data = "OBSTACLE_ALERT"
            if msg.data != self.last_led_state:
                self.changePattern(msg)
            self.last_led_state = msg.data
        else:
            if self.state == "LANE_FOLLOWING":
                msg.data = "CAR_DRIVING"
                if msg.data != self.last_led_state:
                    self.changePattern(msg)
                self.last_led_state = "CAR_DRIVING"
            elif self.state == "NORMAL_JOYSTICK_CONTROL":
                msg.data = "WHITE"
                if msg.data != self.last_led_state:
                    self.changePattern(msg)
                self.last_led_state = msg.data

    def publish_stop_line_msg(self, header, detected=False, at=False, x=0.0, y=0.0):
        """
        Makes and publishes a stop line message.

        Args:
            header: header of a ROS message, usually copied from an incoming message
            detected (`bool`): whether a vehicle has been detected
            at (`bool`): whether we are closer than the desired distance
            x (`float`): distance to the vehicle
            y (`float`): lateral offset from the vehicle (not used, always 0)
        """

        stop_line_msg = StopLineReading()
        stop_line_msg.header = header
        stop_line_msg.stop_line_detected = bool(detected)
        stop_line_msg.at_stop_line = bool(at)
        stop_line_msg.stop_line_point.x = int(x)
        stop_line_msg.stop_line_point.y = int(y)
        self.pub_virtual_stop_line.publish(stop_line_msg)

        """
        Remove once have the road anomaly watcher node
        """
        stopped_flag = BoolStamped()
        stopped_flag.header = header
        stopped_flag.data = bool(at)
        self.pub_stopped_flag.publish(stopped_flag)



if __name__ == "__main__":
    obstacle_detection_node = ObstacleDetectionNode(node_name="obstacle_detection_node")
    # vehicle_filter_node = VehicleFilterNode(node_name="vehicle_filter_node")
    rospy.spin()
