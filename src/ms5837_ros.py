#!/usr/bin/env python

import ms5837_driver
import rospy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
from ms5837.msg import ms5837_data
from nav_msgs.msg import Odometry


# Choose seawater or freshwater depth calibration using ros param
# freshwater = 997 kg/m^3
# seawater = 1029 kg/m^3

class KalmanFilter:

    def __init__(self, max_diff=100):
        # initialize the filter with random values
        self.KG = 1  # Kalman gain
        self.est = 0  # last estimate
        self.e_error = 1  # Error of the filter estimate
        self.last_measurement = None
        self.max_diff = max_diff  # the maximum error between two sensor readings for value to be thrown out

    def update(self, measurement, m_error):
        if self.last_measurement is None:
            self.last_measurement = measurement

        if abs(measurement - self.last_measurement) > self.max_diff:
            rospy.logerr("Sensor value error: change in measurement too large for one time step")
        else:
            self.KG = self.e_error / (self.e_error + m_error)  # compute the Kalman gain
            self.est = self.est + self.KG(measurement - self.est)
            self.e_error = (1 - self.KG)(self.e_error)

        self.last_measurement = measurement
        return self.est, self.e_error

if __name__ == '__main__':
    last_value = None  # the last sensor value for computing the velocity
    last_time = time.time()  # time of last read for computing velocity
    try:
        # set up ros stuff
        rospy.init_node('ms5837_node')
        fluid_density = rospy.get_param('~fluid_density', '1000')
        publish_odom = rospy.get_param('~publish_odom', True)
        publish_pose = rospy.get_param('~publish_pose', False)
        use_kalman_filter = rospy.get_param('~use_kalman_filter', True)
        depth_variance = rospy.get_param('~depth_variance', 0.001)
        tf_frame = rospy.get_param("~tf_frame", "depth_sensor_link")

        pub = rospy.Publisher('rov/ms5837', ms5837_data, queue_size=1)
        rate = rospy.Rate(100)  # 100Hz data read
        sensor = ms5837_driver.MS5837_30BA()  # Default I2C bus is 1 (Raspberry Pi 3)
        # sensor = ms5837.MS5837_02BA()

        sensor.setFluidDensity(int(fluid_density))

        # sensor.init must run immediately after installation of ms5837 object
        sensor.init()

        odom_pub = None
        pose_pub = None
        filter = None
        if publish_odom:
            odom_pub = rospy.Publisher("/rov/depth_odom", Odometry, queue_size=1)
        if publish_pose:
            pose_pub = rospy.Publisher("/rov/depth_pose", PoseWithCovarianceStamped, queue_size=1)
        if use_kalman_filter:
            filter = KalmanFilter(100)

        while not rospy.is_shutdown():
            msg = ms5837_data()

            sensor.read(oversampling=2)  # maximum read rate of ~90Hz

            if use_kalman_filter:
                depth, variance = filter.update(sensor.depth(), depth_variance)
            else:
                depth = sensor.depth()
                variance = depth_variance

            msg.tempC = sensor.temperature(ms5837_driver.UNITS_Centigrade)
            msg.tempF = sensor.temperature(ms5837_driver.UNITS_Farenheit)
            msg.depth = sensor.depth()
            # msg.altitudeM = sensor.altitude() # causes error in driver

            # update message headers
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'depth_data'

            pub.publish(msg)

            if publish_odom:
                msg = Odometry()
                msg.header.frame_id = tf_frame
                msg.header.stamp = rospy.Time.now()
                msg.pose.pose.position.z = float(depth)
                time_now = time.time()
                if last_value is not None:
                    msg.twist.twist.linear.z = float((depth - last_value) / (time_now - last_time))
                last_value = depth
                last_time = time_now
                msg.pose.covariance[14] = variance
                msg.twist.covariance[14] = variance
                odom_pub.publish(msg)

            if publish_pose:
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = tf_frame
                msg.header.stamp = rospy.Time.now()
                msg.pose.pose.position.z = float(depth)
                msg.pose.covariance[14] = variance
                pose_pub.publish(msg)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
