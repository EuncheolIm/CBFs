import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def lmake_xy( dir_file):
        xy = np.genfromtxt(dir_file, delimiter='', dtype = np.float64)
        # print(xy.shape)
        return xy
# data2 = lmake_xy("/home/kist/euncheol/Dual-arm/data/Sim_data/panda_qddot.txt")

data = lmake_xy("/home/kist/euncheol/Dual-arm/data/Sim_data/panda_qddot.txt")
data2 = lmake_xy("/home/kist/euncheol/Dual-arm/data/Sim_data/comp_clik.txt")
print("d : ", data.shape)


mode = 2
if (mode == 1):
        current_q = data[3000:,:7] *180/np.pi
        reference_q = data[3000:,7:14]*180/np.pi
        des = data[3000:,14:]*180/np.pi
        # _q = data[:,12:]*180/np.pi

        # _q2 = data2[:,12:]*180/np.pi
        ii = 0
        plt.figure(figsize=(20,10))
        for i in range(7):
                if (i == 1 or i == 3 or i == 5 ):
                        ii +=1
                if (i == 1 or i == 3 or i == 5 ):
                        plt.subplot(3,1,ii)
                        plt.title("joint {}".format(i+1))
                        plt.plot(des[:,i], label = "des")
                        plt.plot(reference_q[:,i], label = "reference")
                        plt.plot(current_q[:,i], ':', label = "current angle")
                        # plt.plot(_q[:,i], label = "current angle")
                        # plt.plot(_q2[:,i], label = "constraint")
                        plt.ylabel(" angle (deg)")
                
                        plt.xlabel(" time (ms) ")
                        plt.subplots_adjust(
                        wspace=0.05, # the width of the padding between subplots 
                        hspace=0.5) #
                        plt.legend()
        plt.show()
elif(mode == 2):
        pos = ["x","y","z", "roll","pitch","yaw"]

        reference_pos = data[:,:6] 
        current_pos = data[:,6:12] 
        reference_pos[:,3:] = reference_pos[:,3:]* 180/np.pi
        current_pos[:,3:] = current_pos[:,3:]* 180/np.pi
        
        _E_up = 120
        _E_low = 1
        
        _E_t = data[13]
        _E_up = 120*np.ones_like(_E_t)
        _E_low = np.ones_like(_E_t)
        
        # clikpos = data2[:,:6] 
        # clikpos[:,3:] = clikpos[:,3:]* 180/np.pi
        # reference_pos = data[:,6:12] 

        # error = reference_pos - current_pos

        plt.figure(figsize=(20,10))
        for i in range(6):
                plt.subplot(2,3,i+1)
                if (i<3):
                        plt.title("position {}".format([pos[i]]))
                        plt.ylabel(" distance (m)")
                else:
                        plt.title("orientation {}".format([pos[i]]))
                        plt.ylabel(" angle (deg)")
                plt.plot(reference_pos[:,i], label = "reference")
                plt.plot(current_pos[:,i],  label = "current")
                # plt.plot(clikpos[:,i],  label = "CLIK")
                
                # plt.plot(current_pos1[:,i],  label = "CLIK")
                # plt.plot(error[:,i], ':',  label = "error")
                plt.xlabel(" time (ms) ")
                plt.subplots_adjust(
                wspace=0.25, # the width of the padding between subplots 
                hspace=0.3) #
                
                
                
                plt.legend()
        plt.show()
        
        plt.figure(figsize=(20,10))
        plt.plot(_E_up, label = "E_up")
        plt.plot(_E_t, label = "energy")
        plt.plot(_E_low, label = "E_low")
        plt.legend()
        plt.show()
elif(mode == 3):

        ref = data[:,:6] 
        pos = data[:,6:12] 

        ref_x = ref[:,0]
        ref_y = ref[:,1]
        ref_z = ref[:,2]

        x = pos[:,0]
        y = pos[:,1]
        z = pos[:,2]
        
        ref = data[:,:6] 
        pos = data[:,6:12] 


        fig = plt.figure(figsize=(16,12))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x,y,z, label = " ref ")
        ax.plot(ref_x,ref_y,ref_z ,label = " pos ")
        # ax.plot(x1,y1,z1, label = "constraint y<0.05 ")
        # ax.plot(pred_x,pred_y,pred_z , 'bo', label = " desired ")
        ax.set_xlabel("x-axis (m)")
        ax.set_ylabel("y-axis (m)")
        ax.set_zlabel("z-axis (m)")

        plt.legend()
        plt.show()
        
elif(mode == 4):

        Fx = data[:,19:] 
        # Tx = data[:,21:] 


        plt.figure(figsize=(20,10))
        for i in range(6):
                plt.subplot(2,3,i+1)
                pos = ["Fx","Fy","Fz", "Tau x","Tau y","Tau z"]
                if (i<3):
                        plt.title("transition {}".format([pos[i]]))
                        plt.ylabel(" force ")
                else:
                        plt.title("rotation {}".format([pos[i]]))
                        plt.ylabel(" tau ")
                plt.plot(Fx[:,i], label = " force ")

                plt.xlabel(" time (ms) ")
                plt.subplots_adjust(
                wspace=0.25, # the width of the padding between subplots 
                hspace=0.3) #
                
                
                
                plt.legend()
        plt.show()

else:
        print(np.tanh(1))