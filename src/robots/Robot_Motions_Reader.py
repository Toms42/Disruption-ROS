import json


import time
import random
import math

class Robot:

    def load_from_Json(self,name):
        with open(name, "r") as f:
            data = json.loads(f.read())
            self.name = data["name"]
            self.transitions = data["transitions"]
            self.poses = data["poses"]
            self.curr_primitive = data["curr_primitive"]
            self.curr_pose = data["curr_pose"]

    def __init__(self, name,motor_index):
        self.motor_index = motor_index
        self.load_from_Json(name)
        self.excitement = 0


    def light_on(self,light_on = False):
        if light_on:
            self.excitement += 10
        else:
            self.excitement -= .5
        self.excitement = max(-10,min(10,self.excitement))

    def get_time_to_sleep(self,time):
        excitement =  (math.atan(self.excitement) + math.pi/2) 
        return max(min(float(time)/excitement,10),.5)

    def get_desire(self):
        return -self.excitement


    def get_next_pose(self):
        #randomly select pose from the current primitive
        print(self.curr_primitive)
        pose = self.curr_pose
        while(self.curr_pose == pose):
            pose = random.choice(self.poses[self.curr_primitive])
        self.curr_pose = pose

        #randomly select the current primitive based on prob
        item = self.transitions[self.curr_primitive]
        
        self.curr_primitive = random.choices(list(item.keys()), weights=item.values(), k=1)[0]
        
        print(pose)
        print("")
        return pose




'''
class YourThreadName(QThread):

    def __init__(self):
        QThread.__init__(self)

    def __del__(self):
        self.wait()

    def run(self):
        w = winch.QtSerialWinch()
        print(w.available_ports())
        w.set_port("ttyACM0")

        w.open()
       # w.motor_enable(True);
        time.sleep(5)
        print(w.read_input())
        for i in range(1,4):
            print(w.read_input())
            w.set_target([0], [i])
            print(w.status_message())
            time.sleep(.5)

def main():
    r = Robot()
    s = r.toJSON()
    print(s)
    print("--------------------")
    r2 = Robot()
    r2.load_from_Json(s)
    print(r2.toJSON())
 

    myThread = YourThreadName()
    myThread.start()

if __name__ == "__main__":
    main()
'''