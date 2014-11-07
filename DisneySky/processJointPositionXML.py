import xml.etree.ElementTree as ET
import numpy as np 
import os, os.path
#from pylab import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D

#fullpath = "./data/3/test003.center.RE.dat"
fullpath = "./data/record002.in.dat"

#data_dir = "/data"
#subdirs = [x[0] for x in os.walk(data_dir)]

# joint names ~ scale factor, zero pos value 
joint_names = ["BF","BS","BT","P","TT","TS","TF","HT","HN","HTLT","LAF","LAO","LAS","LE","LWT","LWIO","LWFB","RAF","RAO","RAS","RE","RWT","RWIO","RWFB", "LSS", "RSS"]
scale_factor = [0.0313, 0.0637, 0.2326, 0.1663, 0.1589, 0.1875, 0.1741, 0.7785, 0.1761, 0.1540, 0.4252, 0.2772, 0.6583, 0.4068, 0.7450, 0.1837, 0.2049, 0.4222, 0.2875, 0.6819, 0.4102, 0.6686, 0.2342, 0.2025, 0.01073, 0.01212]
scale_factor = np.array([1.0e-03 * x for x in scale_factor])
zero_pos_value = [0, 0, 0, 3.7034, 2.4718, 1.8675, 3.7643, 1.9011, 0.0050, 0.9640, 3.9711, 4.0400, 0.3015, 3.9925, 2.8590, 3.5314, 3.8325, 3.5314, 3.8325, 4.0494, 3.9999, 2.9418, 3.7445, 0.5751, 1.8880, 2.9453, 0.0005651, 0.0059828]
zero_pos_value[:] = np.array([1.0e+03 * x for x in zero_pos_value])


def getJointDict(): 
	print  "getJointDict"
	
	tree = ET.parse('skyfigure_prm.xml')
	root = tree.getroot()
	
	relattitude = []
	relposition = []
	name = []
	count = 0
	joint_dict_prm = dict()

	joint_dict_util = {joint_names[i]: [scale_factor[i], zero_pos_value[i]] for i in xrange(len(joint_names))}
	print joint_dict_util["BF"][0], joint_dict_util["BF"][1]

	for joint in root.iter('Joint'):
		if "relposition" in joint.attrib and "relattitude" in joint.attrib:
			# relative position
			relpositionList = joint.get('relposition')
			# relative rotation matrix: R(1,1) R(1,2) R(1,3) R(2,1) ... R(3,3)
			nineValuesList = joint.get('relattitude')
			# get a dictionary containing position and rotation matrix 
			joint_dict_prm[joint.get('name')] = [np.array(nineValuesList.split(" ")).reshape(3,3), np.array(relpositionList.split(" "))]
		
		print count
		count = count + 1
		print 

	print "size", len(joint_dict_util), np.shape(relattitude), len(name) # 26 , 32, 32 
	return joint_dict_util, joint_dict_prm

def readJointPosData(joint_dict_util): 
	time_stamp_list = []
	left_arm_dict = dict()
	RAF_rel_pos_list= []
	RAO_rel_pos_list = []
	RAS_rel_pos_list = []
	RE_rel_pos_list = []

	# for root, _, files in subdirs:
 #    	for f in files:
 #    		fullpath = os.path.join(root, f)
	for line in open(fullpath, 'r'):
		item = line.rstrip() # strip off newline and any other trailing whitespace
		itemList = item.split("\t")
	 	if itemList[0] == "t_us":
	 		for i, item in enumerate(itemList):
	 			if item == "RAF":
	 				left_arm_dict["RAF"] = i
	 			if item == "RAO":
	 				left_arm_dict["RAO"] = i
	 			if item == "RAS":
	 				left_arm_dict["RAS"] = i 
	 			if item == "RE":
	 				left_arm_dict["RE"] = i
		else:
			# calculate relative postion 
			itemList = item.split("\t")
			time_stamp_list.append(itemList[0])

			RAF_rel_pos = float(joint_dict_util["RAF"][0])*(float(itemList[left_arm_dict["RAF"]]) - float(joint_dict_util["RAF"][1])) # scale*(pot_reading - zero) 
			RAO_rel_pos = float(joint_dict_util["RAO"][0])*(float(itemList[left_arm_dict["RAO"]]) - float(joint_dict_util["RAO"][1])) # scale*(pot_reading - zero) 
			RAS_rel_pos = float(joint_dict_util["RAS"][0])*(float(itemList[left_arm_dict["RAS"]]) - float(joint_dict_util["RAS"][1])) # scale*(pot_reading - zero) 
			RE_rel_pos = float(joint_dict_util["RE"][0])*(float(itemList[left_arm_dict["RE"]]) - float(joint_dict_util["RE"][1])) # scale*(pot_reading - zero) 

			RAF_rel_pos_list.append(RAF_rel_pos)
			RAO_rel_pos_list.append(RAO_rel_pos)
			RAS_rel_pos_list.append(RAS_rel_pos)
			RE_rel_pos_list.append(RE_rel_pos)
			big_list = [time_stamp_list, RAF_rel_pos_list, RAO_rel_pos_list, RAS_rel_pos_list, RE_rel_pos_list]

	return big_list

def getTraj(big_list, joint_dict_prm):
	H_RAF_ini = np.zeros([4,4])
	H_RAS_ini = np.zeros([4,4])
	H_RAO_ini = np.zeros([4,4])
	H_RE_ini = np.zeros([4,4])
	H_endeffector_list = []
	pos_list = []

	# get the H matrix of the parent joint with respect to the data_joint 
	H_RAF_ini[0:3,0:3] = joint_dict_prm["RAF"][0] 
	H_RAF_ini[:,3] = np.append(joint_dict_prm["RAF"][1], 1)

	H_RAS_ini[0:3,0:3] = joint_dict_prm["RAS"][0] 
	H_RAS_ini[:,3] = np.append(joint_dict_prm["RAS"][1], 1)

	H_RAO_ini[0:3,0:3] = joint_dict_prm["RAO"][0] 
	H_RAO_ini[:,3] = np.append(joint_dict_prm["RAO"][1], 1)
	
	H_RE_ini[0:3,0:3] = joint_dict_prm["RE"][0] 
	H_RE_ini[:,3] = np.append(joint_dict_prm["RE"][1], 1)

	print H_RAF_ini

	for i in xrange(len(big_list[1])): 
		RAF = big_list[1][i]
		RAO = big_list[2][i]
		RAS = big_list[3][i]
		RE = big_list[4][i]

		H_RAF_rel = np.array([[np.cos(RAF), np.sin(RAF), 0, 0],[-np.sin(RAF), np.cos(RAF), 0, 0],[0,0,1,0],[0,0,0,1]])
		H_RAS_rel = np.array([[np.cos(RAS), np.sin(RAS), 0, 0],[-np.sin(RAS), np.cos(RAS), 0, 0],[0,0,1,0],[0,0,0,1]])
		H_RAO_rel = np.array([[np.cos(RAO), np.sin(RAO), 0, 0],[-np.sin(RAO), np.cos(RAO), 0, 0],[0,0,1,0],[0,0,0,1]])
		H_RE_rel = np.array([[np.cos(RE), np.sin(RE), 0, 0],[-np.sin(RE), np.cos(RE), 0, 0],[0,0,1,0],[0,0,0,1]])

		H_RE = np.dot(H_RE_rel, H_RE_ini) 
		H_RAS = np.dot(H_RAS_rel, H_RAS_ini) 
		H_RAO = np.dot(H_RAO_rel, H_RAO_ini)
		H_RAF = np.dot(H_RAF_rel, H_RAF_ini)

		H_endeffector = np.dot(H_RE, np.dot(H_RAS, np.dot(H_RAO, H_RAF)))


		pos_list.append(H_endeffector[0:3, 3])
		H_endeffector_list.append(H_endeffector)

	# draw trajectory 
	pos_list = np.array(pos_list)
	time_stamp_list = big_list[0]

	fig = plt.figure(figsize=(14,6))
	ax = Axes3D(fig)
	
	ax.set_xlabel('x')
	ax.set_ylabel('y')
	ax.set_zlabel('z')

	ax.plot(pos_list[:,0],pos_list[:,1],pos_list[:,2])
	plt.show()

# def init():
#     pendulum1.set_data([], [])
#     pendulum2.set_data([], [])

# def update(n): 
#     # n = frame counter
#     # calculate the positions of the pendulums
#     x1 = + L * sin(x[n, 0])
#     y1 = - L * cos(x[n, 0])
#     x2 = x1 + L * sin(x[n, 1])
#     y2 = y1 - L * cos(x[n, 1])
    
#     # update the line data
#     pendulum1.set_data([0 ,x1], [0 ,y1])
#     pendulum2.set_data([x1,x2], [y1,y2])

# anim = animation.FuncAnimation(fig, update, init_func=init, frames=len(t), blit=True)


def main(): 
	[joint_dict_util, joint_dict_prm] = getJointDict()
	big_list = readJointPosData(joint_dict_util)
	getTraj(big_list, joint_dict_prm)

if __name__ == "__main__":
	main()

