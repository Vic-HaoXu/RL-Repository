import math
import rospy
import time
import tf
from excavator_ros_driver.msg import RTKData
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16

num = 0

x0 = 0
y0 = 0
yaw0 = 0

yaw01 = 0

x_axis = []
y_axis = []

crane_pose = 0.0

cal_status = 0

swing_flag = False

def RTK_converter(Bdeg, Ldeg):

	# time_0 = time.time()

	scale = 1
	B = Bdeg / 180.0 * math.pi
	L = Ldeg / 180.0 * math.pi
	a = 6378137
	b = 6356752.3142
	e1 = math.sqrt((math.pow(a, 2)-math.pow(b, 2)) / math.pow(a, 2))
	e2 = math.sqrt((math.pow(a, 2)-math.pow(b, 2)) / math.pow(b, 2))

	prjno = math.floor((L/math.pi*180.0 - 1.5) / 3) + 1
	L0 = prjno * 3/180.0 * math.pi

	ita2 = e2*e2*math.cos(B)*math.cos(B)
	T = math.tan(B) * math.tan(B)
	l = L - L0
	W = math.sqrt(1-e1*e1*math.sin(B)*math.sin(B))
	N = a/W

	m0 = a * (1 - e1*e1)
	m2 = 3 * (e1 * e1 * m0) / 2.0
	m4 = 5 * (e1 * e1 * m2) / 4.0
	m6 = 7 * (e1 * e1 * m4) / 6.0
	m8 = 9 * (e1 * e1 * m6) / 8.0

	a0 = m0 + m2/2.0 + 3*m4/8.0 + 5*m6/16.0 + 35*m8/128.0
	a2 = m2/2.0 + m4/2.0 + 15*m6/32.0 + 7*m8/16.0
	a4 = m4/8.0 + 3*m6/16.0 + 7*m8/32.0
	a6 = m6/32.0 + m8/16.0

	X = a0*B - math.sin(B)*math.cos(B)*((a2-a4+a6) + (2*a4-(16*a6/3))*math.sin(B)*math.sin(B) + 16*a6*math.pow(math.sin(B), 4)/3.0)

	x = X + N*math.sin(B)*math.cos(B)*l*l/2.0 + N*math.sin(B)*math.cos(B)*math.cos(B)*math.cos(B)*(5-T+9*ita2+4*ita2*ita2)*l*l*l*l/24.0 + N*math.sin(B)*math.cos(B)*math.cos(B)*math.cos(B)*math.cos(B)*math.cos(B)*(61-58*T+T*T)*l*l*l*l*l*l/720.0

	y = N*math.cos(B)*l + N*math.cos(B)*math.cos(B)*math.cos(B)*(1-T+ita2)*l*l*l/6.0 + N*math.cos(B)*math.cos(B)*math.cos(B)*math.cos(B)*math.cos(B)*(5-18*T+T*T+14*ita2-58*ita2*T)*l*l*l*l*l/120.0

	x1 = x*scale
	y1 = y*scale
	x2 = x1 + 0
	y2 = y1 + 500000

	## compute time consumed
	# time.sleep(1)
	# time_1 = time.time()
	# print('Time elapsed: %f' % (time_1-time_0))
	# print(x2, y2)

	return x2, y2

# RTK_converter(24.597318170000000, 117.9883029900000)

def convertDeg(fpDegA):
	
	while(fpDegA >= 180 or fpDegA < -180):
		if (fpDegA >= 180.0):
			fpDegA -= 360.0
		elif (fpDegA < -180.0):
			fpDegA += 360.0

	return fpDegA

def joint_CB(msg):

	global crane_pose
	global swing_flag

	swing_flag = True

	crane_pose = msg.position[0] * 180 / math.pi

def cal_CB(msg):

	global cal_status

	cal_status = msg.data

def rtk_CB(msg):

	global num

	global x0
	global y0
	global yaw0

	global yaw01

	global x_axis
	global y_axis

	global crane_pose

	global cal_status
	global swing_flag

	longtitude = msg.longitude
	latitude = msg.latitude
	yaw = msg.yaw

	x, y = RTK_converter(longtitude, latitude)

	print('raw data: ', x, y, yaw)

	# x_axis = [math.cos(yaw0), math.sin(yaw0)]
	# y_axis = [-math.sin(yaw0), math.cos(yaw0)]

	# x1 = x * x_axis[0] + y * x_axis[1]
	# y1 = x * y_axis[0] + y * y_axis[1]

	# print(x, y, yaw)

	print("crane_pose: {}".format(crane_pose))

	if num == 0 and swing_flag == True:
		print("\nInitializing global coordinate...")

		x0 = x
		y0 = y
		yaw01 = yaw + crane_pose
		yaw0_l = yaw01 / 180.0 * math.pi
		x_axis = [math.cos(yaw0_l), math.sin(yaw0_l)]
		y_axis = [-math.sin(yaw0_l), math.cos(yaw0_l)]

	if cal_status == 1:
		print("\nReceive new target! Re-calibration using encoder data...")

		yaw0 = yaw01 - crane_pose
		# yaw0_l = yaw0 / 180.0 * math.pi
		# x_axis = [math.cos(yaw0_l), math.sin(yaw0_l)]
		# y_axis = [-math.sin(yaw0_l), math.cos(yaw0_l)]

		cal_status = 0

	x = x - x0
	y = y - y0
	yaw = convertDeg(yaw - yaw0)

	# yaw = yaw + crane_pose

	x1 = x * x_axis[0] + y * x_axis[1]
	y1 = x * y_axis[0] + y * y_axis[1]

	print(x1, y1, yaw)
	# print(x_axis, y_axis)

	br = tf.TransformBroadcaster()
	br.sendTransform((x1, y1, 0), 
					 tf.transformations.quaternion_from_euler(0, 0, yaw * math.pi / 180.0),
					 rospy.Time.now(),
					 'base_link',
					 'world')
	
	num+=1

if __name__ == '__main__':

	rospy.init_node("RTK_Converter")

	rospy.Subscriber('rtk_data', RTKData, rtk_CB, queue_size=10)

	rospy.Subscriber('/excavator_joint_states', JointState, joint_CB, queue_size=10)

	rospy.Subscriber('/crane_calibration', UInt16, cal_CB, queue_size=1)

	rospy.spin()