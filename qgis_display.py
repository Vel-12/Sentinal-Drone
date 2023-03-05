#!/usr/bin/env python3

import rospy
from sentinel_drone.msg import Geolocation


def plotpoints(msg):

    canvas = iface.mapCanvas()

    pnt = QgsPointXY(msg.lat, msg.long)

    m = QgsVertexMarker(canvas)
    m.setCenter(pnt)
    m.setColor(QColor('Black'))
    m.setIconType(QgsVertexMarker.ICON_CIRCLE)
    m.setIconSize(12)
    m.setPenWidth(1)
    m.setFillColor(QColor(0, 200, 0))

rospy.init_node('qgisnode')
rospy.Subscriber('/geolocation',Geolocation,plotpoints)

if __name__ == '__main__':
    while not rospy.is_shutdown():
        rospy.spin()