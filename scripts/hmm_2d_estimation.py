#!/usr/bin/python
import roslib
import rospy
from matplotlib.pyplot import *
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PointStamped
import sys
sys.path.append('/home/valeria/hmm')
from hmm.continuous.GMHMM import GMHMM
#from hmm.discrete.DiscreteHMM import DiscreteHMM
from std_srvs.srv import Empty
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
import numpy as np
import math as m
import tf

train_hmm = False
path_estimate = False
print_once = True
r_point = PointStamped()
prev_size_dec_seq = 0
counter_sample = 0
sample_size = 200
train_observation_seq = np.array([])
plot_sequence = np.array([])
plot_path = np.array([])
path = np.array([])
decoding_seq = np.array([])
theta_k = Float32()
theta_A = np.array([])
# ------- HMM Definition ----------
N = 4
M = 6
D = 2
print("---- HMM Initial parameters ----")
pi = np.array([0.6, 0.2, 0.2])
print("Initial pi: ", pi)
A = np.array([[0.6, 0.2, 0.2] , [0.2, 0.6, 0.2], [0.2, 0.2, 0.6]])
print("Initial A: ", A)
w = np.ones((N,M),dtype=np.double)*0.5
print("w", w)
means = np.ones((N,M,D),dtype=np.double)*0.5
print("means", means)
#print(np.size(means))
covars = [[ np.matrix(np.eye(D,D)) for j in xrange(M)] for i in xrange(N)]

# w[0][0] = 0.5
# w[0][1] = 0.5
# w[1][0] = 0.5
# w[1][1] = 0.5  


# btmp = np.random.random_sample((N, M))
# row_sums = btmp.sum(axis=1)
# B = btmp / row_sums[:, np.newaxis]
# print("Initial B: ", B)

print("HMM node started... Call a service")
#print("--- Check rows sum up to 1 ---")

HMM_model = GMHMM(N,M,D,A,means,covars,w,pi,init_type='user',verbose=True)


def get_training_set_srv(req):
	print("Start training requested")
	print("Run a bag...")
	global train_hmm
	train_hmm = True
	return

def start_training_srv(req):
	global train_hmm
	train_hmm = False
	"Stopped collecting training data..."
	if train_observation_seq.size:
		print("   Training will begin with a sequence of length ", train_observation_seq.size)
		HMM_model.train(train_observation_seq,10, epsilon=0.00001)	
		print("Training is done:")
		print("Pi: ", HMM_model.pi)
		print("A: ", HMM_model.A)
		print("weights: ", HMM_model.w)
		print("means: ", HMM_model.means)
		print("covars: ", HMM_model.covars)
		
	else:
		print("Training sequence is an empty array!")
	return

def start_viterbi_srv(req):
	global path_estimate, print_once
	print("Starting path estimation")
	print("Run a bag...")
	path_estimate = True
	print_once = True
	return

def stop_viterbi_srv(req):
	global path_estimate, decoding_seq, path, plot_sequence, plot_path
	print("Stopped path estimation")
	print("Path", path)
	path_estimate = False
	plot_sequence = np.copy(decoding_seq)
	decoding_seq = np.delete(decoding_seq,np.s_[:],0)
	plot_path = np.copy(path)
	path = np.delete(path,np.s_[:],0)
	print("Size(decoding_seq)", decoding_seq.size)
	print("Size(path)", path.size)
	return

def reset_train_seq_srv(req):
	global train_observation_seq
	print("Deleting previous observations")
	train_observation_seq = np.delete(train_observation_seq,np.s_[:],0)
	print("size(obs_seq)", train_observation_seq.size)
	return


def estimator_callback(msg):
	global train_observation_seq, decoding_seq, theta_A, print_once, path, r_point, counter_sample
	r_point = msg 
	point_t = np.array([[r_point.point.x, r_point.point.z]])
	#theta_A = np.append(theta_A, theta_k)
	if train_hmm == True:
		if print_once:
			print("Creating observation sequence")
			train_observation_seq = point_t
			counter_sample += 1
			#print(train_observation_seq)
			print_once = False
		else:
			if counter_sample == sample_size:
				train_observation_seq = np.append(train_observation_seq, point_t, 0)
				counter_sample = 1
			else:
				counter_sample += 1
			#print("train_observation_seq", train_observation_seq)
		#print("obs_seq:", train_observation_seq)
	if path_estimate == True:
		if print_once:
			print("Estimating path")
			decoding_seq = point_t
			counter_sample = 1
			print_once = False
		else:
			if counter_sample == sample_size:
				decoding_seq = np.append(decoding_seq, point_t, 0)
				counter_sample = 1
			else:
				counter_sample += 1

		#path = HMM_model.decode(decoding_seq)
		#print("Path: ", path)
	return

def compute_path(event):
	global decoding_seq, prev_size_dec_seq, path_estimate, path
	if path_estimate == True:
		size_dec_seq = decoding_seq.size
		#print("size_dec_seq", size_dec_seq)
		if size_dec_seq > prev_size_dec_seq:
			path = HMM_model.decode(decoding_seq)
			#print("path", path)
		prev_size_dec_seq = size_dec_seq
		#print("prev_size_dec_seq", prev_size_dec_seq)


def hmm_track_agents():
	rospy.init_node('hmm_track_agents', anonymous=True)
	#rospy.Subscriber("/KF/theta", Float32, estimator_callback)
	rospy.Subscriber("/KF/point_estimation", PointStamped, estimator_callback)
	start_set = rospy.Service('get_training_set', Empty, get_training_set_srv)
	stop_set = rospy.Service('start_training_set', Empty, start_training_srv)
	get_path = rospy.Service('get_path', Empty, start_viterbi_srv)
	stop_path = rospy.Service('stop_path', Empty, stop_viterbi_srv)
	get_path = rospy.Service('reset_training_seq', Empty, reset_train_seq_srv)
	rospy.Timer(rospy.Duration(1), compute_path)
	rospy.spin()
	return

if __name__ == '__main__':

	hmm_track_agents()

	try:
		rate = rospy.Rate(10.0)
		while not rospy.is_shutdown():
			rate.sleep()

	except KeyboardInterrupt:
		pass
	raw_input("Press enter to plot theta or ctrl+Z if not")
	sampled_X_axis = np.linspace(0,(np.size(plot_sequence)-1),np.size(plot_sequence))
	print("Path: ", plot_path)
	#print("np.size(path)", np.size(path))
	X_path = np.linspace(0,(np.size(plot_path)-1),np.size(plot_path))
	figure(0)
	plot(sampled_X_axis, plot_sequence, 'g'), title('theta'), ylabel('[rad]')
	figure(1)
	plot(X_path, plot_path), title('Estimated Path'), ylabel('States')
	show()
