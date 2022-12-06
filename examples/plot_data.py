import numpy as np
import pdb
import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib
import csv
import pdb

from unitree_legged_sdk_python_tools.utils.data_parsing import read_cvs_file

def main():

	# folder_name = "2022_11_22_16_43_76"; # from laptop
	# folder_name = "2022_11_22_16_46_34"; # inside the robot


	# folder_name = "2022_01_13_18_30_55"

	folder_name = "2022_01_13_19_28_46"

	path2data = "./"

	data, file_names, joints_names = read_cvs_file(path2data,folder_name)

	print(file_names)

	# pdb.set_trace()

	ind_is_zero = data[0,:,0] == 0.0 # Check when the timestamp is zero and remove those values (the program finished before the data logging did...)
	data = data[:,~ind_is_zero,:]

	# data = data[:,0:5500,:] # 2022_01_13_19_24_16

	Njoints = 3
	k_cut = 0 # Cut the first time steps
	hdl_fig, hdl_splots = plt.subplots(Njoints,1,figsize=(13,9),sharex=True)
	hdl_fig.suptitle("Position - Right Leg")
	ind_plot = 0
	for jj in range(3,Njoints+3):
		time_stamp = data[0,k_cut::,0] # It's the same for all, q_des, q_curr, etc.
		hdl_splots[ind_plot].plot(time_stamp,data[7,k_cut::,jj+1],label=file_names[7]) # desired
		# hdl_splots[jj].plot(time_stamp[5::],data[0,5::,jj+1],label=file_names[0]) # current (delay compensated)
		hdl_splots[ind_plot].plot(time_stamp,data[0,:,jj+1],label=file_names[0]) # current
		# hdl_splots[jj].plot(time_stamp,data[2,:,jj+1],label=file_names[2]) # raw
		hdl_splots[ind_plot].set_title(joints_names[jj])
		hdl_splots[ind_plot].set_ylabel("angle [rad]")
		hdl_splots[ind_plot].legend()
		ind_plot += 1
	hdl_splots[Njoints-1].set_xlabel("time [sec]")


	hdl_fig, hdl_splots = plt.subplots(Njoints,1,figsize=(13,9),sharex=True)
	hdl_fig.suptitle("Velocity - Right Leg")
	k_cut = 10
	for jj in range(Njoints):
		time_stamp = data[0,k_cut::,0] # It's the same for all, q_des, q_curr, etc.
		# time_stamp_time_step = np.diff(time_stamp,prepend=0.0)

		joint_vel_vec = data[1,:,jj+1]

		joint_pos_vec = data[0,:,jj+1]
		joint_vel_vec_num_diff = np.diff(joint_pos_vec,prepend=0.0) / 0.002

		hdl_splots[jj].plot(time_stamp,joint_vel_vec[k_cut::],label=file_names[1])
		hdl_splots[jj].plot(time_stamp,joint_vel_vec_num_diff[k_cut::],label="dq_curr (num diff)")
		hdl_splots[jj].set_title(joints_names[jj])
		hdl_splots[jj].set_ylabel("angular velocity [rad/s]")
		hdl_splots[jj].legend()
	hdl_splots[Njoints-1].set_xlabel("time [sec]")


	hdl_fig, hdl_splots = plt.subplots(Njoints,1,figsize=(13,9),sharex=True)
	hdl_fig.suptitle("Torque - Right Leg")
	for jj in range(Njoints):
		time_stamp = data[0,k_cut::,0] # It's the same for all, q_des, q_curr, etc.
		hdl_splots[jj].plot(time_stamp,data[6,k_cut::,jj+1],label=file_names[6]) # u des
		hdl_splots[jj].plot(time_stamp,data[9,k_cut::,jj+1],label=file_names[9]) # u est
		hdl_splots[jj].set_title(joints_names[jj])
		hdl_splots[jj].set_ylabel("torque [Nm]")
		hdl_splots[jj].legend()
	hdl_splots[Njoints-1].set_xlabel("time [sec]")


	plt.show(block=True)




if __name__ == "__main__":

	main()