from os import path
import os

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
    
def cal_start(time, pre_dis_12, pre_time, pre_speed, obj_list):
    if (pre_dis_12 == 0):
        obj1 = obj_list[0]
        obj2 = obj_list[1]
        pre_dis_12 = (obj1.x - obj2.x)**2 + (obj1.y - obj2.y)**2 + (obj1.z - obj2.z)**2
        pre_speed = 0.1
    else:
        obj1 = obj_list[0]
        obj2 = obj_list[1]
        cur_dis_12 = (obj1.x - obj2.x)**2 + (obj1.y - obj2.y)**2 + (obj1.z - obj2.z)**2
        delta_t = (time - pre_time)* 1e-9
        speed = abs(cur_dis_12-pre_dis_12) / delta_t
        if(abs(time - 174768666054028.0) < 1e8):
            print(speed)
        pre_dis_12 = cur_dis_12
        pre_time = time
    return pre_dis_12, pre_time, pre_speed


def filter():
    desktop = os.path.expanduser("~/Desktop")
    align_path = desktop+"/align_file"
    time = None
    pre_time = None
    pre_speed = 0
    num_rigid = 0
    pre_count = 0
    pre_dis_12 = 0
    count = 0
    obj_list = []
    r1 = rigid_body()
    with open(path.join(align_path,"Mocap_t1.txt"),'r') as infile:
        lines = infile.readlines()
        for line in lines:
            if (line.split(" Time: ")[0] == ">> System"):
                time = long(line.split(" Time: ")[1].split("\n")[0])
                if pre_time == None:
                    pre_time = time 
            if (line.split(" of RigidBodies: ")[0] == "#"):
                num_rigid = int(line.split(" of RigidBodies: ")[1].split("\n")[0])
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
                    pre_dis_12, pre_time, pre_speed = cal_start(time, pre_dis_12, pre_time, pre_speed, obj_list)
                    obj_list = []
            else:
                r1 = rigid_body()
                pre_count = pre_count +1
                
                
if __name__ == '__main__':
    filter()

