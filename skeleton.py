#!/usr/bin/env python

PKG = 'boat_waypoints' # this package name
import roslib; roslib.load_manifest(PKG)

import rospy

from usc_asv_msgs.msg import Waypoint,Mission,Heading

from pyproj import Proj, Geod
import yaml
import time
import copy
import numpy as np

from usc_asv_msgs.srv import ControlSelect

def shift(l, n):
    """Circular shift a list"""
    return l[n:] + l[:n]

def pts_to_mission(pts, name, speed=1.5, radius=3.0, exit_radius=6.0, park_time=0, repeat=0, park_last=False, closest_first=False):
    """Creates a mission message from a list of lat-lon tuples."""
    m = Mission()
    m.name = name
    m.repeat = repeat
    m.park_last = park_last
    m.closest_first = closest_first
    i = 0
    for p in pts:
        i+=1
        wp = Waypoint()
        wp.latitude = p[0]
        wp.longitude = p[1]
        wp.speed = speed
        wp.park_time = park_time
        wp.radius = radius
        wp.exit_radius = exit_radius
        wp.name = 'WP #%d' % i
        m.waypoints.append(wp)

    return m

# Initialize ROS node
rospy.init_node('boat_waypoints', anonymous=True)

# Instruct ASV to execute waypoint control commands
rospy.loginfo("Selecting control path")
service_name = 'control_select'
rospy.wait_for_service(service_name)
control_select_service = rospy.ServiceProxy(service_name, ControlSelect)
control_select_service('waypoint')

# Set up ROS publisher for usc_asv_msgs/Mission type.
pub = rospy.Publisher('waypoint_cmd', Mission, queue_size=1) #, latch=True)

"""
Utility for geodetic calculations. Can do the following useful things:
g.fwd(lon, lat, heading, distance)
 Returns longitude, latitude of end point, e.g. the location of a point
 at given distance in a given heading from a start location.
g.inv(lon1, lat1, lon2, lat2)
 Returns the headings (forward, backward) and distance between two sets of
 lat-lons
Example:
 g.fwd(-118.289995, 34.020140,  10, 90) # 10m due east from RTH
 returns: (-118.28982579659416, 34.02093907183671, -169.9999053323755)
Documentation: http://pyproj.googlecode.com/svn/trunk/docs/pyproj.Geod-class.html
"""
g = Geod(ellps='clrk66')

"""
Utility for carthographic transformations. Can transform from a metric map projection
to lat, lon for exaple.
Example:
 p(-118.289995, 34.020140)
 returns: (380896.08575474937, 3765139.292554757)
 p(380896.08575474937, 3765139.292554757, inverse=True)
 returns: (-118.28999500000053, 34.020139999999785)
Documentation: http://pyproj.googlecode.com/svn/trunk/docs/pyproj.Proj-class.html
"""
# Sets up a projection to/from UTM coordinate system, zone 11.
p = Proj(proj='utm',zone=11,ellps='WGS84')

"""CODE GOES HERE"""
pts = [(34.020140,-118.289995),]

# Convert list of lat longs to a mission message
m = pts_to_mission(pts)

# Publish mission
pub.publish(m)

# Wait to make sure publishing was completed
rospy.loginfo("Waiting to ensure it is sent.")
rospy.sleep(2)
