import math
import random
import os
import time

PI = 3.1415

# num_leaders = 4

time_step = 32
# control_type = ""
# local_gain = 2 #0.5
# global_gain = 20
collision_distance = 0.015
BASE_PATH = '/Users/shamoilkhomosi/Desktop/Sheffield/FYP/simulation/webots/'

def run_sim():
	foldername = control_type
	os.system("webots --mode=fast --no-rendering --stderr --stdout --minimize  --log-performance=/Users/shamoilkhomosi/Desktop/Sheffield/FYP/simulation/webots/worlds/perflog"+foldername+".txt e-puck2.wbt  &> log.txt &")
	print("Running Webots....\n")
	time.sleep(300) # 180
	os.system("exit")
	kill_statement = "osascript -e '" + 'quit app "Webots"' + "'"
	os.system(kill_statement) # last resort, quit all webots instances
	print("All done....\n")


def create_world():
	cluster_size = round(num_agents/num_leaders) # this should be an odd number always!
	leader_index = math.ceil(cluster_size/2)

	TARGET_Y = []
	TARGET_X = []
	DIRECTION = []
	for i in range(num_agents):
		# target_y = round(1.5 - (math.cos(2*PI*(i)/num_agents)), 2)
		target_y = 3.2*(math.sin(2*PI*((i)*(num_agents+1)/(num_agents))/num_agents)/2)
		# target_y = 0
		target_x = round((i+0)*0.1, 1) #+3
		direction = round((360*i/num_agents), 2)

		TARGET_Y.append(target_y)
		TARGET_X.append(target_x)
		DIRECTION.append(direction)
	print(TARGET_Y)
	print(TARGET_X)
	print(DIRECTION)

	with open('e-puck2.wbt', 'w') as f:
		f.write('#VRML_SIM R2021a utf8\nWorldInfo {\n  info [\n    "The model of the e-puck 2 robot"\n  ]\n  title "e-puck simulation"\n  basicTimeStep '+str(time_step)+'\n  coordinateSystem "NUE"\n  contactProperties [    ContactProperties {\n      coulombFriction [\n        0.9\n      ]\n      rollingFriction 0.65 0.65 0.65\n    }\n  ]\n}\nViewpoint {\n orientation -0.771530439165453 -0.5608861161017472 -0.30024580963847025 1.243726186873522\n  position -0.43858165831619056 4.97982469409408 2.4622756868701403\n}\nTexturedBackground {\n}\nTexturedBackgroundLight {\n}\nRectangleArena {\n  floorSize 100 10\n  floorAppearance FormedConcrete {\n  }\n}\n')
		for m in range(1,num_leaders+1):
			for n in range(1,cluster_size+1):
				i = (m-1)*cluster_size + n

				f.write('E-puck {\n')
				# f.write('  translation ' + str(round((i-1)*0.1, 1)) + ' 0 ' + str(3.2*(math.sin(2*PI*((i-1)*(num_agents+1)/(num_agents))/num_agents)/2)) + '\n')
				f.write('  translation ' + str(round((i-1)*0.1, 1)) + ' 0 ' + '0' + '\n')

				if (i - leader_index)%cluster_size == 0:
					my_index = m
					f.write('  name "e-puck-' + str(my_index) + '"\n')
					agent_type = "leader"
				else:
					my_leader = m
					my_index = n
					if i == 1:
						boundary = '-lb'
					elif i == num_agents:
						boundary = '-rb'
					else:
						boundary = ''
					f.write('  name "e-puck-' + str(my_leader) + '-' + str(my_index) + boundary + '"\n')
					agent_type = "follower"
				f.write('  controller "' + agent_type + '_controller"\n')

				f.write('  version "2"\n')

				segment = math.floor((i-1)/cluster_size)+1
				if agent_type == "leader":
					f.write('  emitter_channel ' + str(segment) + '\n')
					f.write('  receiver_channel ' + str(segment) + '\n')
				else:
					segment = segment*100 + n
					f.write('  emitter_channel ' + str(segment) + '\n')
					f.write('  receiver_channel ' + str(segment) + '\n')
					# f.write('  synchronization FALSE\n')
				f.write('  turretSlot [\n    GPS {\n    }\n    Compass {\n    }\n  ]\n')
				f.write('}\n')

				f.write('SolidBox {\n')
				f.write('  translation ' + str(TARGET_X[i-1]) + ' -0.04 ' + str(-TARGET_Y[i-1]) + '\n')
				f.write('  size 0.08 0.08 0.08\n')
				f.write('  appearance PBRAppearance {\n')
				f.write('    baseColor 0.517449 0.807218 0.999069\n')
				f.write('    roughness 0.5\n')
				f.write('    metalness 0\n')
				f.write('  }\n')
				f.write('}\n')



	with open('../controllers/parameters.h', 'w') as f:
		f.write('#define MIN(X, Y) ((X) > (Y) ? (Y) : (X))\n')
		f.write('#define MAX(X, Y) ((X) > (Y) ? (X) : (Y))\n')
		f.write('\n')
		f.write('#define '+control_type+'\n')
		f.write('#define LOCAL_GAIN '+str(local_gain)+'\n')
		f.write('#define GLOBAL_GAIN '+str(global_gain)+'\n')
		f.write('\n')
		f.write('// num of leaders\n')
		f.write('#define NUM_CLUSTERS '+str(num_leaders)+'\n')
		f.write('\n')
		f.write('// num of agents per cluster\n')
		f.write('#define CLUSTER_SIZE '+str(int(num_agents/num_leaders))+'.0\n')
		f.write('#define COLLISION_MIN_DISTANCE '+str(collision_distance)+'\n')
		f.write('const char* FNAME = "' + BASE_PATH + 'worlds/sim_data_'+control_type+'.csv";\n')


		f.write('#define TARGET_Y (double[]){')
		for i in range(num_agents):
			f.write(str(TARGET_Y[i]))
			if i == num_agents-1:
				f.write('}\n')
			else:
				f.write(', ')
		f.write('#define TARGET_X (double[]){')
		for i in range(num_agents):
			f.write(str(TARGET_X[i]))
			if i == num_agents-1:
				f.write('}\n')
			else:
				f.write(', ')
		f.write('#define DIRECTION (double[]){')
		for i in range(num_agents):
			f.write(str(DIRECTION[i]))
			if i == num_agents-1:
				f.write('}\n\n')
			else:
				f.write(', ')

		f.write('double get_bearing_in_rad(WbDeviceTag tag) {\n')
		f.write('  const double *north = wb_compass_get_values(tag);\n')
		f.write('  double rad = atan2(north[2], north[0]);\n')
		f.write('  return rad;\n')
		f.write('}\n')

		f.write('void CalcDistanceToDestination(double* return_val, const double destinationCoordinate_y, const double destinationCoordinate_x, WbDeviceTag gps, WbDeviceTag compass) {\n')
		f.write('  // double currentCoordinate = cartesianConvertVec3fToCartesianVec2f(wb_gps_get_values(gps));\n')
		f.write('  const double *gpsRead = wb_gps_get_values(gps);\n')
		f.write('  const double currentCoordinate_y = -gpsRead[2];\n')
		f.write('  const double currentCoordinate_x = gpsRead[0];\n')
		f.write('  \n')
		f.write('  const double dist_y = -currentCoordinate_y+destinationCoordinate_y;\n')
		f.write('  const double dist_x = -currentCoordinate_x+destinationCoordinate_x;\n')
		f.write('  \n')
		f.write('  const double currentBearing = get_bearing_in_rad(compass);\n')
		f.write('  const double destinationBearing = atan2(dist_y, dist_x)  - 1.5708;\n')
		f.write('  \n')
		# f.write(r'  printf("%lf %lf %lf %lf %lf %lf\n", currentCoordinate_y, currentCoordinate_x, currentBearing, destinationCoordinate_y, destinationCoordinate_x, destinationBearing);')
		# f.write('  \n')
		f.write('  #if defined(EST_2D_ALL)\n')
		f.write('  return_val[0] = (double) sqrt( dist_y*dist_y + dist_x*dist_x );\n')
		f.write('  #else\n')
		f.write('  return_val[0] = dist_y;\n')
		f.write('  #endif\n')
		f.write('  return_val[1] = (double) destinationBearing - currentBearing;\n')
		f.write('}\n')

	os.system("export WEBOTS_HOME=/Applications/Webots.app")
	os.system("export WEBOTS_HOME_PATH=$WEBOTS_HOME/Contents")
	os.system('export PATH="/Applications/Webots.app/Contents/MacOS:$PATH"')
	os.chdir(BASE_PATH+'controllers/follower_controller')
	os.system("make")
	os.chdir(BASE_PATH+'controllers/leader_controller')
	os.system("make")
	os.chdir(BASE_PATH+'worlds/')
	print("Build complete....\n")

def save_sim_data():
	foldername = str(num_agents) + "a_" + str(num_leaders) + "l_" + str(local_gain) + "l_" + str(global_gain) + "g_/" 
	try:
		os.makedirs(BASE_PATH + 'worlds/' + foldername)
		filenames = os.listdir(BASE_PATH + 'worlds/')
		csv_list =  [ filename for filename in filenames if (filename.endswith( ".csv" ) or filename.endswith( ".txt" ) ) ]
		print(csv_list)
		for csv_file in csv_list:
			os.replace(BASE_PATH + 'worlds/' + csv_file, BASE_PATH + 'worlds/' + foldername + csv_file)
	except FileExistsError:
		pass


# num_leaders = 5
# local_gain = 0.5
# global_gain = 10
# control_type = "EST_QUAD"
# create_world()
# run_sim()

num_agents = 40
num_leaders_list = [5]
# global_gain_list = [10, 20, 30]
# local_gain_list = [0.5, 1, 2]

# num_leaders_list = [2, 6, 14]
local_gain_list = [1]
global_gain_list = [20]
control_types = ["EST_IDEAL", "EST_CONSTANT", "EST_LINEAR", "EST_QUAD"]
if os.path.exists(BASE_PATH+'worlds/log.txt'):
	os.remove(BASE_PATH+'worlds/log.txt')

for num_leaders in num_leaders_list:
	for global_gain in global_gain_list:
		for local_gain in local_gain_list:
			for control_type in control_types:
				print("!!!!!!!!!" + control_type + "!!!!!!!!!")
				create_world();
			# 	run_sim();
			# save_sim_data()
