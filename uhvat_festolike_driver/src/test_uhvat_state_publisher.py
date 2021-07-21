import rospy

from sensor_msgs.msg import JointState


def main():
    rospy.init_node('test_publish')
    pub = rospy.Publisher('/uhvat/joint_states', JointState, queue_size=10)
    
    rate = rospy.Rate(500)
    while not rospy.is_shutdown():
        try:
            msg = JointState()
            msg.header.stamp.secs = rospy.get_rostime().secs
            msg.header.stamp.nsecs = rospy.get_rostime().nsecs
            msg.name.append('uhvat_joint_1')
            msg.name.append('uhvat_joint_2')
            msg.position.append(1)
            msg.position.append(-1)

            pub.publish(msg)
        except Exception as e:
            print(e)

        rate.sleep()


if __name__=="__main__":
    main()