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


def create_marker(path_msg, color_msg, description, path_id, width=0.1, delta_z=0.1):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = path_msg.header.frame_id
    int_marker.name = str(path_id)
    int_marker.description = "Path {0}".format(path_id)
    line_marker = Marker()
    line_marker.type = Marker.LINE_STRIP
    line_marker.scale.x = width
    line_marker.color = color_msg
    line_marker.points = [p.pose.position for p in path_msg.poses]
    for point in line_marker.points:
        point.z += delta_z
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.markers.append(line_marker)
    int_marker.controls.append(copy.deepcopy(control))

    menu_handler = MenuHandler()

    d = menu_handler.insert("Description")
    for line in description:
        menu_handler.insert(line, parent=d)

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
        rospy.Subscriber("paths", Paths, self.updatePaths, queue_size=1)
        self.pub = rospy.Publisher("selected_path", NavPath, queue_size=1)
        self.delta_z = 0.1

        # self.add_marker(test_msg(), 0)
        # self.server.applyChanges()

        rospy.spin()

    def add_marker(self, msg, path_id):
        print('Add path', path_id)
        menu, marker = create_marker(path_msg=msg.path, color_msg=msg.color,
                                     description=msg.description, path_id=path_id,
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
