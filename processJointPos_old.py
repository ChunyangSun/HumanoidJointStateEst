import xml.etree.ElementTree as ET
import numpy as np 
import os, os.path

data_dir = "/data"
subdirs = [x[0] for x in os.walk(data_dir)]

joint_names = ["BF","BS","BT","P","TT","TS","TF","HT","HN","HTLT","LAF","LAO","LAS","LE","LWT","LWIO","LWFB","RAF","RAO","RAS","RE","RWT","RWIO","RWFB", "LSS", "RSS"]
scale_factor = [0.0313, 0.0637, 0.2326, 0.1663, 0.1589, 0.1875, 0.1741, 0.7785, 0.1761, 0.1540, 0.4252, 0.2772, 0.6583, 0.4068, 0.7450, 0.1837, 0.2049, 0.4222, 0.2875, 0.6819, 0.4102, 0.6686, 0.2342, 0.2025, 0.01073, 0.01212]
scale_factor = np.array([1.0e-03 * x for x in scale_factor])
zero_pos_value = [0, 0, 0, 3.7034, 2.4718, 1.8675, 3.7643, 1.9011, 0.0050, 0.9640, 3.9711, 4.0400, 0.3015, 3.9925, 2.8590, 3.5314, 3.8325, 3.5314, 3.8325, 4.0494, 3.9999, 2.9418, 3.7445, 0.5751, 1.8880, 2.9453, 0.0005651, 0.0059828]
zero_pos_value[:] = np.array([1.0e+03 * x for x in zero_pos_value])


def getJointDict(): 
	joint_dict = {joint_names[i]: [scale_factor[i], zero_pos_value[i]] for i in xrange(len(joint_names))}
		#joint_dict[joint_names(i)] = [scale_factor[i], zero_pos_value[i]]
	print joint_dict["BF"][0], joint_dict["BF"][1]


	tree = ET.parse('skyfigure_prm.xml')
	root = tree.getroot()

	print  "hello lets get started"
	relattitude = []
	name = []
	count = 0
	for joint in root.iter('Joint'):
		print count

		if "relattitude" in joint.attrib:
			nineValuesList = joint.get('relattitude')
			nineValuesSplit = nineValuesList.split(" ")
			print nineValuesSplit
			print relattitude

			relattitude.append(nineValuesSplit)
			name.append(joint.get('name'))

		count = count + 1
		print 

	relattitude = np.array(relattitude)

	print "size", np.shape(relattitude), len(name)

def readJointPosData(): 
	for root, _, files in subdirs[0]:
	    for f in files:
	    	print fullpath 
	        fullpath = os.path.join(root, f)
	        for line in open(fullpath, 'r'):
	        	print line
	        	break
	        	item = line.rstrip() # strip off newline and any other trailing whitespace
	        	print item
def main(): 
	data_name = "BF"
	# relative rotation matrix: R(1,1) R(1,2) R(1,3) R(2,1) ... R(3,3)
	for name in joint_dict.keys():
		if name == data_name:
			rel_pos = joint_dict[name][0]*(pot_reading - joint_dict[name][1])


if __name__ == "__main__":
	main()

