from os import path, listdir
import os
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from numpy.linalg import inv

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

def compute_coord(obj_list,align_path,f):
    r1 = obj_list[0]   # lu frame corner
    r2 = obj_list[1]   # lu door corner
    r3 = obj_list[2]   #camera
    image_path = align_path +"/save_file_1"
    
    r1_array = np.array([[r1.x], [r1.y], [r1.z], [1]])
    r2_array = np.array([[r2.x], [r2.y], [r2.z], [1]])
    #print("r1_array:::"+str(r1_array))
    r3_array = np.array([[r3.x], [r3.y], [r3.z]])
#####
    lfd_array = np.array([[-0.54067284], [-0.0111583145], [-0.59584385], [1]])
    rfu_array = np.array([[-0.47726107], [-2.0054173], [-1.5429099], [1]])
    rfd_array = np.array([[-0.52493185], [-0.0029689868], [-1.4633833], [1]])
#####
    K = np.array([[602.25927734375, 0.0, 321.3750915527344], [0.0, 480.0400390625, 240.51527404785156], [0.0, 0.0, 1.0]])  #intrinsic
    P = [[1,0,0,0],[0,1,0,0],[0,0,1,0]]

    T = R.from_quat([r3.qx, r3.qy, r3.qz, r3.qw]).as_matrix() #extrinsic rotation
    T = np.hstack((T,r3_array)) #extrinsic translation
    T = np.append(T, [[0,0,0,1]], axis=0)
    T = inv(T)
    T_compensate = R.from_quat([0.7071, 0 ,0.7071 ,0]).as_matrix() #extrinsic rotation
    #T_compensate = R.from_quat([ 0.2027, 0 ,0.7863 ,-0.5836]).as_matrix() #extrinsic rotation

    compensate_array = np.array([[0.2], [-0.4], [-0.8]])
    #print(compensate_array)

    T_compensate = np.hstack((T_compensate, compensate_array)) #extrinsic translation
    T_compensate = np.append(T_compensate, [[0,0,0,1]], axis=0)
    print("T_compensate_r1:::"+str(P @ T_compensate @ T @ r1_array))
    print("T_compensate_r2:::"+str(P @ T_compensate @ T @ r2_array))

    r1_transform = K @ P @ T_compensate @ T @ r1_array
    print("r1_transform_before:::"+str(r1_transform))
    #r1_transform = np.matmul(np.matmul(np.matmul(K, P), T), r1_array)
    r1_transform = r1_transform/r1_transform[2]
    print("r1_transform:::"+str(r1_transform))

    r2_transform = K @ P @ T_compensate @ T @ r2_array
    print("r2_transform_before:::"+str(r2_transform))
    r2_transform = r2_transform/r2_transform[2]
    print("r2_transform:::"+str(r2_transform))

    lfd_transform = K @ P @ T_compensate @ T @ lfd_array
    lfd_transform = lfd_transform/lfd_transform[2]

    rfu_transform = K @ P @ T_compensate @ T @ rfu_array
    rfu_transform = rfu_transform/rfu_transform[2]

    rfd_transform = K @ P @ T_compensate @ T @ rfd_array
    rfd_transform = rfd_transform/rfd_transform[2]

   # print("x in camera:"+str(r1_transform))
    #print("image"+str(f))
    image = cv2.imread(path.join(image_path,f))
    cv2.rectangle(image, (640- int(r2_transform[0])-10, 480- int(r2_transform[1])-10, 20, 20), (0,255,0), -1)
    cv2.rectangle(image, (640- int(r1_transform[0])-10, 480- int(r1_transform[1])-10, 20, 20), (255,0,0), -1)
    cv2.rectangle(image, (640- int(lfd_transform[0])-10, 480- int(lfd_transform[1])-10, 20, 20), (0,255,0), -1)
    cv2.rectangle(image, (640- int(rfu_transform[0])-10, 480- int(rfu_transform[1])-10, 20, 20), (0,255,0), -1)
    cv2.rectangle(image, (640- int(rfd_transform[0])-10, 480- int(rfd_transform[1])-10, 20, 20), (0,255,0), -1)


    cv2.imshow("image", image)
    cv2.waitKey(3000) & 0xFF

    #print("r1_transform:"+str(r1_transform))


def compute_once(align_path, exp_time, f):
    #print("exp_time::"+str(exp_time))
    global init_time
    global obj_list
    global pre_time
    global continue_flag

    with open(path.join(align_path,"Mocap_t1.txt"),'r') as infile:
        r1 = rigid_body()
        lines = infile.readlines()
        for line in lines:
            if ((line.split(" Time: ")[0] == ">> System") and (round(float(int(line.split(" Time: ")[1].split("\n")[0]) - init_time)/1e9,2))>0) and ((round(float(int(line.split(" Time: ")[1].split("\n")[0]) - init_time)/1e9,2) > exp_time) and (pre_time <= exp_time)):
                #print("exp_time::::::"+str(exp_time))
                #print("pre_time::::::"+str(pre_time))
                #print("time_duration::::::::::"+str((round(float(int(line.split(" Time: ")[1].split("\n")[0]) - init_time)/1e9,2))))
                pre_time = round(float(int(line.split(" Time: ")[1].split("\n")[0]) - init_time)/1e9,2)
                continue_flag = 1
                #print("only_once")
            
                #print("f::::"+str(f))

            elif (line.split(" Time: ")[0] == ">> System"):
                pre_time = round(float(int(line.split(" Time: ")[1].split("\n")[0]) - init_time)/1e9,2)

                #print("hahahaha"+line)
            if continue_flag == 1:
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
                        compute_coord(obj_list,align_path,f)
                        continue_flag = 0
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
        #print("file read!!")
        exp_time = process_file_time(f)
        compute_once(align_path,exp_time,f)
        

                
if __name__ == '__main__':
    pre_count = 0
    count = 0
    pre_time = 0
    init_time = 174755184484862
    time = None
    num_rigid = 0
    obj_list = []
    continue_flag =0
    filter()

