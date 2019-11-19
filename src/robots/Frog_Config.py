import json


import time
import random

class Robot:
    def __init__(self):
        self.name = "snake";
        self.transitions = {"Neutral":{"Neutral":1,"Active":0},
            "Active":{"Neutral":1,"Active":0}}
        #self.poses = {"Neutral": [([0,0,0,0],2,.5,1),([600,-1200,700,0],2,.5,1)],
           # "Active": [([600,100,800,0],3,10,.6),([-600,-800,0,0],2,10,.2)]}
        self.poses = {"Neutral": [([0,0,0,0],2,1,.5),([400,400,-400,0],2,5,.2),([-500,1200,0,0],2,10,.1)]}
        self.curr_primitive = "Neutral"
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

def main():
    r = Robot()
    s = r.toJSON()
    #print(s)
    #print("--------------------")
    r2 = Robot()
    r2.load_from_Json(s)
    #print(r2.toJSON())

    with open("frog_config.json", "w") as f:
        f.write(r.toJSON())


if __name__ == "__main__":
    main()