import json


import time
import random

class Robot:
    def __init__(self):
        self.name = "snake";
        self.transitions = {"Slither":{"Rear":0,"Rattle":0,"Slither":1},
            "Rear":{"Rear":0,"Rattle":1},
            "Rattle":{"Rattle":.8,"Slither":.2}}
        self.poses = {"Slither": [([0,0,0,0],2,3,.5),([1500,400,-800,500],2,5,.3), ([-800,-800,-800,800],2,10,.2)]}
        self.curr_primitive = "Slither"
        self.curr_pose = [0,0,0,0]
        self.speed = 1;
        self.motor_index = 1

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

def main():
    r = Robot()
    s = r.toJSON()
    #print(s)
    #print("--------------------")
    r2 = Robot()
    r2.load_from_Json(s)
    #print(r2.toJSON())

    with open("snake_config.json", "w") as f:
        f.write(r.toJSON())


if __name__ == "__main__":
    main()