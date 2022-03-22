#!/usr/bin/env python3
import rospy

from visualization_msgs.msg import Marker
from visualization_msgs.msg import InteractiveMarkerControl
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.interactive_marker_server import InteractiveMarker
from interactive_markers.menu_handler import MenuHandler
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from functools import partial
from copy import deepcopy


def processFeedback(_pub: rospy.Publisher, _feedback):
    _pub.publish(_feedback.pose)


def button_callback(_marker: Marker):
    print(_marker)


if __name__ == "__main__":
    rospy.init_node("simple_marker")

    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer("simple_marker")

    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "world"
    int_marker.name = "my_marker"
    int_marker.description = "Simple 1-DOF Control"

    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.1
    box_marker.scale.y = 0.1
    box_marker.scale.z = 0.1
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append(box_marker)

    # add the control to the interactive marker
    int_marker.controls.append(box_control)

    # create a control which will move the box
    # this control does not contain any markers,
    # which will cause RViz to insert two arrows
    for orientation, label in zip([Quaternion(1, 0, 0, 1),
                                   Quaternion(0, 1, 0, 1),
                                   Quaternion(0, 0, 1, 1)], list('xzy')):

        rotate_control = InteractiveMarkerControl()
        rotate_control.orientation = orientation
        rotate_control.name = 'rotate_'+label
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(rotate_control)

        translate_control = InteractiveMarkerControl()
        translate_control.name = 'translate_'+label
        translate_control.orientation = orientation
        translate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(translate_control)

    menu_handler = MenuHandler()
    menu_handler.insert("First Entry", callback=button_callback)
    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True
    control.markers.append(box_marker)
    int_marker.controls.append(control)
    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    pub = rospy.Publisher('desired_pose', Pose, queue_size=10)

    server.insert(int_marker, partial(processFeedback, pub))
    menu_handler.apply(server, int_marker.name)

    # 'commit' changes and send to all clients
    server.applyChanges()

    rospy.spin()
