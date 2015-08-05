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

set_params = True
train_hmm = False
path_estimate = False
print_once = True
r_point = PointStamped()
prev_size_dec_seq = 0
counter_sample = 0
sample_size =200
train_observation_seq = np.array([])
plot_sequence = np.array([])
plot_path = np.array([])
path = np.array([])
decoding_seq = np.array([])
theta_k = Float32()
theta_A = np.array([])
# ------- HMM Definition ----------
N = 4
M = 1
D = 2

# ::: N = 4 :::
print("---- HMM Initial parameters ----")
# pi = np.array([0.7, 0.1, 0.1, 0.1])
# print("Initial pi: ", pi)
# A = np.array([[0.5, 0.3, 0.1, 0.1] , [0.1, 0.5, 0.3, 0.1], [0.1, 0.1, 0.5, 0.3], [0.1, 0.2, 0.2, 0.5]])
# #A = np.array([[0.5, 0.3, 0.199, 0.001] , [0.01, 0.5, 0.3, 0.2], [0.001, 0.099, 0.5, 0.4], [0.001, 0.001, 0.198, 0.8]])
# print("Initial A: ", A)
# w = np.ones((N,M),dtype=np.double)*1/M
# print("w", w)
# means = np.ones((N,M,D),dtype=np.double)*0.5
# print("means", means)
# #print(np.size(means))
# covars = [[ np.matrix(np.eye(D,D)) for j in xrange(M)] for i in xrange(N)]
# print("covars", covars)

# ::: No training params :::
pi = np.array([1.,  0.,  0.,  0.])
print("Initial pi: ", pi)
A = np.array([[ 0.9,  0.0998,   0.0001, 0.0001],
       [  0.0001,   0.9,  0.0998, 0.0001],
       [  0.0001,   0.0001,   0.9, 0.0998],
       [  0.0001,   0.0001,   0.0001,
          0.9997]])
#A = np.array([[0.5, 0.3, 0.199, 0.001] , [0.01, 0.5, 0.3, 0.2], [0.001, 0.099, 0.5, 0.4], [0.001, 0.001, 0.198, 0.8]])
print("Initial A: ", A)
w = np.ones((N,M),dtype=np.double)
print("w", w)
means = np.array([[[-0.016,  0.5275]],

       [[-0.15,  0.363]],

       [[-0.24 ,  0.3166]],

       [[-0.056,  0.2866]]])
print("means", means)
#print(np.size(means))
covars = [[np.matrix([[  0.0006,  0.0],
        [ 0.0,  0.001768]])], [np.matrix([[ 0.0025  , 0.0],
        [ 0.0,   0.0032]])], [np.matrix([[ 0.0025, 0.0],
        [0.0,  0.0101]])], [np.matrix([[ 0.005 ,  0.0],
        [ 0.0,  0.0176]])]]
print("covars", covars)


#print("shape(covars)", np.size(covars))
# # ::: N = 3 :::
# print("---- HMM Initial parameters ----")
# pi = np.array([0.6, 0.2, 0.2])
# print("Initial pi: ", pi)
# A = np.array([[0.6, 0.3, 0.1] , [0.2, 0.6, 0.2], [0.1, 0.3, 0.6]])
# print("Initial A: ", A)
# w = np.ones((N,M),dtype=np.double)*0.5
# print("w", w)
# means = np.ones((N,M,D),dtype=np.double)*0.5
# print("means", means)
# #print(np.size(means))
# covars = [[ np.matrix(np.eye(D,D)) for j in xrange(M)] for i in xrange(N)]
# print("covars", covars)

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
		HMM_model.train(train_observation_seq,100, epsilon=0.00001)	
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
	#rospy.Subscriber("/filtered_torque", PointStamped, estimator_callback)
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
	X_axis_train = np.linspace(0,(train_observation_seq.shape[0]-1),train_observation_seq.shape[0])
	X_axis_decode = np.linspace(0,(plot_sequence.shape[0]-1),plot_sequence.shape[0])
	zero_vector_train = np.zeros(len(X_axis_train))
	zero_vector_decode = np.zeros(len(X_axis_decode))
	print("Path: ", plot_path)
	#print("np.size(path)", np.size(path))
	X_path = np.linspace(0,(np.size(plot_path)-1),np.size(plot_path))
	if set_params== False:
		figure(0)
		subplot(211)
		plot(X_axis_train, train_observation_seq[:,0], 'r'), title('training_seq r_x'), ylabel('[m]')
		plot(X_axis_train,zero_vector_train, '--k')
		subplot(212)
		plot(X_axis_train, train_observation_seq[:,1], 'b'), title('training_seq r_z'), ylabel('[m]')
		plot(X_axis_train,zero_vector_train, '--k')

		figure(1)
		subplot(211)
		plot(X_axis_decode, plot_sequence[:,0], 'r'), title('decoding_seq r_x'), ylabel('[m]')
		plot(X_axis_decode,zero_vector_decode, '--k')
		subplot(212)
		plot(X_axis_decode, plot_sequence[:,1], 'b'), title('decoding_seq r_z'), ylabel('[m]')
		plot(X_axis_decode,zero_vector_decode, '--k')
		
	figure(2), ylim((-1, 4))
	plot(X_path, plot_path), title('Estimated Path'), ylabel('States')
	show()
