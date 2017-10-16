#!/usr/bin/env python

import copy

import rospy
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from rviz_paths.msg import Path, Paths
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import (InteractiveMarker,
                                    InteractiveMarkerControl,
                                    Marker)

from nav_msgs.msg import Path as NavPath

import numpy as np

from tf.transformations import quaternion_from_euler


def orientation(v):
    roll = np.arctan2(v[1], v[0])
    pitch = np.pi * 0.5 - np.arctan2(v[2], np.sqrt(v[0]**2 + v[1]**2))
    return quaternion_from_euler(0, pitch, roll, axes='szyz')


def cylinder_between(p1, p2, color_msg, width=0.1):
    cylinder = Marker()
    cylinder.type = Marker.CYLINDER
    cylinder.scale.x = cylinder.scale.y = width
    cylinder.color = color_msg
    cylinder.scale.z = np.linalg.norm(p1 - p2)
    m = (p1 + p2) * 0.5
    cylinder.pose.position.x = m[0]
    cylinder.pose.position.y = m[1]
    cylinder.pose.position.z = m[2]
    o = cylinder.pose.orientation
    o.x, o.y, o.z, o.w = orientation(p2 - p1)
    return cylinder


def sphere_at(p, color_msg, width=0.1):
    sphere = Marker()
    sphere.type = Marker.SPHERE
    sphere.scale.x = sphere.scale.y = sphere.scale.z = width
    sphere.color = color_msg
    sphere.pose.position.x = p[0]
    sphere.pose.position.y = p[1]
    sphere.pose.position.z = p[2]
    return sphere


def node(pose, delta_z):
    p = pose.pose.position
    return np.array([p.x, p.y, p.z + delta_z])


def create_marker(path_msg, color_msg, description, path_id, width=0.1, delta_z=0.1):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = path_msg.header.frame_id
    int_marker.name = str(path_id)
    int_marker.description = "Path {0}".format(path_id)
    # line_marker = Marker()
    # line_marker.type = Marker.LINE_STRIP
    # line_marker.scale.x = width
    # line_marker.color = color_msg
    # line_marker.points = [p.pose.position for p in path_msg.poses]
    # for point in line_marker.points:
    #     point.z += delta_z
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.interaction_mode = InteractiveMarkerControl.MENU
    # control.markers.append(line_marker)

    points = [node(pose, delta_z) for pose in path_msg.poses]
    for p1, p2 in zip(points[:-1], points[1:]):
        control.markers.append(cylinder_between(p1, p2, color_msg, width))

    for p in points:
        control.markers.append(sphere_at(p, color_msg, width))

    int_marker.controls.append(copy.deepcopy(control))

    menu_handler = MenuHandler()

    # put all the information in the main menu

    #d = menu_handler.insert("Description")
    for line in description:
        menu_handler.insert(line)#, parent=d)

    return menu_handler, int_marker


def ignore(msg):
    pass


def test_msg():
    msg = Path()
    msg.path.header.frame_id = 'base_link'
    msg.path.poses.append(PoseStamped())
    msg.path.poses.append(PoseStamped())
    msg.path.poses[1].pose.position.y = 1
    msg.color = ColorRGBA(1.0, 0.5, 0.0, 0.5)
    msg.description = ["A=1"]
    return msg


class RvizPathServer(object):
    def __init__(self):
        super(RvizPathServer, self).__init__()
        rospy.init_node("rviz_paths")
        self.server = InteractiveMarkerServer("paths")
        self.paths = {}
        self.delta_z = rospy.get_param('~offset', 0.15)
        self.width = rospy.get_param('~width', 0.15)
        self.pub = rospy.Publisher("selected_path", NavPath, queue_size=1)
        rospy.Subscriber("paths", Paths, self.updatePaths, queue_size=1)

        # self.add_marker(test_msg(), 0)
        # self.server.applyChanges()

        rospy.spin()

    def add_marker(self, msg, path_id):
        menu, marker = create_marker(path_msg=msg.path, color_msg=msg.color,
                                     description=msg.description, path_id=path_id,
                                     width=self.width,
                                     delta_z=self.delta_z)
        self.server.insert(marker, ignore)
        menu.insert("FOLLOW", callback=self.goto(path_id))
        menu.apply(self.server, marker.name)
        self.paths[path_id] = msg.path

    def goto(self, path_id):
        def f(msg):
            rospy.loginfo("Follow path %d", path_id)
            self.pub.publish(self.paths[path_id])
        return f

    def updatePaths(self, msg):

        self.server.clear()

        for i, m in enumerate(msg.paths):
            self.add_marker(m, i)
        self.server.applyChanges()


if __name__ == '__main__':
    RvizPathServer()
