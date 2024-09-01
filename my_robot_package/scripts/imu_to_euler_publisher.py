#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import tf.transformations as transformations

def imu_callback(data):
    # Convert quaternion to Euler angles
    quaternion = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    euler = transformations.euler_from_quaternion(quaternion)
    
    # Convert radians to degrees
    roll_deg = euler[0] * (180.0 / 3.141592653589793)
    pitch_deg = euler[1] * (180.0 / 3.141592653589793)
    yaw_deg = euler[2] * (180.0 / 3.141592653589793)

    # Create a Vector3 message with the Euler angles in degrees
    euler_msg = Vector3()
    euler_msg.x = roll_deg
    euler_msg.y = pitch_deg
    euler_msg.z = yaw_deg

    # Publish the Euler angles
    euler_pub.publish(euler_msg)

def imu_to_euler_publisher():
    global euler_pub
    
    

if __name__ == '__main__':
    rospy.init_node('imu_to_euler_publisher', anonymous=True)    
    rospy.Subscriber("/imu", Imu, imu_callback)
    euler_pub = rospy.Publisher("/imu/euler", Vector3, queue_size=10)
    try:
        imu_to_euler_publisher()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()