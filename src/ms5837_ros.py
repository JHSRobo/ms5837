#!/usr/bin/env python

import ms5837_driver

import rospy
from ms5837.msg import ms5837_data

# Choose seawater or freshwater depth calibration using ros param
# freshwater = 997 kg/m^3
# seawater = 1029 kg/m^3
fluidDensity = rospy.get_param('~fluidDensity', '1000')


def publisher():
	# set up ros stuff
	pub = rospy.Publisher('rov/ms5837', ms5837_data, queue_size=1)
	rospy.init_node('ms5837')
	rate = rospy.Rate(50)  # 50Hz data read

	sensor = ms5837_driver.MS5837_30BA()  # Default I2C bus is 1 (Raspberry Pi 3)
	# sensor = ms5837.MS5837_02BA()
	sensor.setFluidDensity(int(fluidDensity))


	# sensor.init must run immediately after installation of ms5837 object
	sensor.init()

	while not rospy.is_shutdown():
		msg = ms5837_data()

		try:
			sensor.read(oversampling=4) #maximum read rate of ~90Hz

			msg.tempC = sensor.temperature(ms5837_driver.UNITS_Centigrade)
			msg.tempF = sensor.temperature(ms5837_driver.UNITS_Farenheit)
			msg.depth = sensor.depth()
			# msg.altitudeM = sensor.altitude() # causes error in driver

			# update message headers
			msg.header.stamp = rospy.Time.now()
			msg.header.frame_id = 'depth_data'

			pub.publish(msg)

		except Exception as e:
			rospy.logdebug("Depth Sensor read failed! %s", e)

		rate.sleep()


if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
