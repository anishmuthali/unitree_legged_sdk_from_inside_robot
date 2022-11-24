import numpy as np
import pdb
import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib
import csv
import pdb

def test():



	"""
	# Figure out the initialization of comm.h
	# Where are we running the heavy computations? 
		Laptop: we can visualize; improved workflow
		Robot: workflow worse. We need to stream all the data back to the laptop
	# Visulation: the raisim visual articulated system isn't affected y gravity, so doesn't care if the base is fixed or foating.
		the fastest is to code uo the visulation inn c++
	# Coompensate for the delay: how much of the delay we see is due to the bad PD vs. the roundtrip delay?
	
	For now send data from laptop for improved workflow.


	Explore other quanitites, like q_raw, and foot sensors
	"""










	file_path = "2022_11_22_16_43_76"; # from laptop
	# file_path = "2022_11_22_16_46_34"; # inside the robot

	# joints_names = ["FR_0","FR_1","FR_2","FL_0","FL_1","FL_2","RR_0","RR_1","RR_2","RL_0","RL_1","RL_2"]
	joints_names = ["Front Right - Hip Lateral","Front Right - Hip Forward","Front Right - Knee",
					"Front Left - Hip Lateral","Front Left - Hip Forward","Front Left - Knee",
					"Rear Right - Hip Lateral","Rear Right - Hip Forward","Rear Right - Knee",
					"Rear Right - Hip Lateral","Rear Right - Hip Forward","Rear Right - Knee"]

	file_names = ["q_curr","q_des","dq_curr","u_des","u_est"]
	# file_ind = [0,1,2,3,4]
	data = np.zeros((len(file_names),3001,13))
	for ff in range(data.shape[0]):

		file_path_full = "{0:s}/data_robot_{1:s}.csv".format(file_path,file_names[ff])
		with open(file_path_full, newline='') as csvfile:
			spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
			ii = 0
			for row in spamreader:
				jj = 0
				if ii == 0:
					ii += 1
					continue
				for col in row:
					if jj == 12:
						if col[-1] == ";": 
							print("double check that this is correct .... col = col[0:-1] !!!!!!!!!!!!!!!!!!!")
							pdb.set_trace()
							col = col[0:-1]
					data[ff,ii,jj] = np.float64(col)
					jj += 1
				ii += 1


	# pdb.set_trace()

	data = data[:,10::,:]

	Njoints = 3
	k_cut = 0 # Cut the first time steps
	hdl_fig, hdl_splots = plt.subplots(Njoints,1,figsize=(13,9),sharex=True)
	hdl_fig.suptitle("Position - Right Leg")
	for jj in range(Njoints):
		time_stamp = data[0,k_cut::,0] # It's the same for all, q_des, q_curr, etc.
		hdl_splots[jj].plot(time_stamp,data[1,k_cut::,jj+1],label=file_names[1]) # desired
		# hdl_splots[jj].plot(time_stamp[5::],data[0,5::,jj+1],label=file_names[0]) # current (delay compensated)
		hdl_splots[jj].plot(time_stamp,data[0,:,jj+1],label=file_names[0]) # current
		hdl_splots[jj].set_title(joints_names[jj])
		hdl_splots[jj].set_ylabel("angle [rad]")
		hdl_splots[jj].legend()
	hdl_splots[Njoints-1].set_xlabel("time [sec]")


	hdl_fig, hdl_splots = plt.subplots(Njoints,1,figsize=(13,9),sharex=True)
	hdl_fig.suptitle("Velocity - Right Leg")
	for jj in range(Njoints):
		time_stamp = data[0,k_cut::,0] # It's the same for all, q_des, q_curr, etc.
		hdl_splots[jj].plot(time_stamp,data[2,k_cut::,jj+1],label=file_names[2])
		hdl_splots[jj].set_title(joints_names[jj])
		hdl_splots[jj].set_ylabel("angle [rad/s]")
		hdl_splots[jj].legend()
	hdl_splots[Njoints-1].set_xlabel("time [sec]")


	hdl_fig, hdl_splots = plt.subplots(Njoints,1,figsize=(13,9),sharex=True)
	hdl_fig.suptitle("Torque - Right Leg")
	for jj in range(Njoints):
		time_stamp = data[0,k_cut::,0] # It's the same for all, q_des, q_curr, etc.
		hdl_splots[jj].plot(time_stamp,data[3,k_cut::,jj+1],label=file_names[3]) # u des
		hdl_splots[jj].plot(time_stamp,data[4,k_cut::,jj+1],label=file_names[4]) # u est
		hdl_splots[jj].set_title(joints_names[jj])
		hdl_splots[jj].set_ylabel("torque [Nm]")
		hdl_splots[jj].legend()
	hdl_splots[Njoints-1].set_xlabel("time [sec]")


	plt.show(block=True)




if __name__ == "__main__":

	test()