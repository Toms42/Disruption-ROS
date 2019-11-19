import json
from rcp import winch

import time
from PyQt5.QtCore import QThread

class Robot:
    def __init__(self):
        self.name = "snake";
        self.transitions = {"Slither":{"Rear":.2,"Rattle":.2,"Slither":.6},
            "Rear":{"Rear":.5,"Rattle":.5},
            "Rattle":{"Rattle":.6,"Slither":.4}}
        self.poses = {"Slither": [[0,0,0,0],[1,1,1,1]],
            "Rear": [[2,2,2,2]],
            "Rattle": [[3,3,3,3]]}
        self.curr_primitive = "Slither";
        self.curr_pose = [0,0,0,0]
        self.speed = 1;

    def toJSON(self):
        return json.dumps(self,default=lambda o: o.__dict__,
            sort_keys=True,indent=4)

    def load_from_Json(self,string):
        data = json.loads(string)
        self.name = data["name"]
        self.transitions = data["transitions"]
        self.poses = data["poses"]
        self.curr_primitive = data["curr_primitive"]
        self.curr_pose = data["curr_pose"]

    def update(self,light_state,enable):
        if not enable: #stop motor
            return





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



