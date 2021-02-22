from os import path, listdir
import os
import numpy as np
from scipy.spatial.transform import Rotation as R

class rigid_body:
    rigid_ID = 0
    tracked = True
    x = 0
    y = 0
    z = 0
    qx = 0
    qy = 0
    qz = 0
    qw = 0

def compute_coord(obj_list):
    r1 = obj_list[0]   # lu frame corner
    r2 = obj_list[1]   # lu door corner
    r3 = obj_list[2]   #camera
    
    r1_array = np.array([[r1.x], [r1.y], [r1.z], [1]])
    r3_array = np.array([[r3.x], [r3.y], [r3.z]])
    K = np.array([[602.25927734375, 0.0, 321.3750915527344], [0.0, 603.0400390625, 240.51527404785156], [0.0, 0.0, 1.0]])  #intrinsic
    T = R.from_quat([r3.qx, r3.qy, r3.qz, r3.qw]).as_matrix() #extrinsic rotation
    T = np.hstack((T,r3_array)) #extrinsic translation
    T = np.append(T, [[0,0,0,1]], axis=0)
    P = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])
    
    r1_transform = np.matmul(np.matmul(np.matmul(K, P), T), r1_array)

    print("r1_transform:"+str(r1_transform))


def compute_once(align_path, exp_time):
    #print("exp_time::"+str(exp_time))
    global init_time
    global obj_list
    global pre_time

    with open(path.join(align_path,"Mocap_t1.txt"),'r') as infile:
        r1 = rigid_body()
        lines = infile.readlines()
        for line in lines:
            if (line.split(" Time: ")[0] == ">> System"):
                time = int(line.split(" Time: ")[1].split("\n")[0])
                time_duration = round(float(time - init_time)/1e9,2)
                #print("time_duration:"+str(time_duration))
                if not ((time_duration >= exp_time) and (pre_time < exp_time)):
                    continue

                pre_time = time_duration

            if (line.split(" of RigidBodies: ")[0] == "#"):
                num_rigid = int(line.split(" of RigidBodies: ")[1].split("\n")[0])
            global pre_count
            global count
            if (pre_count == count):
                if (line.split("igidBody ID: ")[0] == 'R'):
                    r1.rigid_ID = int(line.split("igidBody ID: ")[1])
                if (line.split("Tracked : ")[0] == ""):
                    if(line.split("Tracked : ")[1].split("\n")[0] == "true"):
                        r1.tracked = True
                    else:
                        continue
                if (line.split(": ")[0] == 'X'):
                    r1.x = float(line.split(": ")[1].split(" - ")[0])
                    r1.y = float(line.split(": ")[2].split(" - ")[0])
                    r1.z = float(line.split(": ")[3].split("\n")[0])
                if (line.split(": ")[0] == 'qX'):
                    r1.qx = float(line.split(": ")[1].split(" - ")[0])
                    r1.qy = float(line.split(": ")[2].split(" - ")[0])
                    r1.qz = float(line.split(": ")[3].split(" - ")[0])
                    r1.qw = float(line.split(": ")[4].split("\n")[0])
                    count = count +1
                    obj_list.append(r1)
                if (line.split(" END DATA ")[0] == "<<"):
                    compute_coord(obj_list)
                    obj_list = []
            else:
                r1 = rigid_body()
                pre_count = pre_count +1


def process_file_time(file):
    time_array = file.split("_")[2].split(".")
    time = float(time_array[0]) + float(time_array[1])/100
    return time

def filter():
    desktop = os.path.expanduser("~/Desktop")
    align_path = desktop+"/align_file"
    image_path = align_path +"/save_file_1"
    

    for f in listdir(image_path):
        exp_time = process_file_time(f)
        compute_once(align_path,exp_time)
        

                
if __name__ == '__main__':
    pre_count = 0
    count = 0
    pre_time = 0
    init_time = 174755266054028
    time = None
    num_rigid = 0
    obj_list = []
    filter()

