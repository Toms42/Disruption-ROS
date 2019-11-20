import json


import time
import random

class Robot:
    def __init__(self):
        self.name = "snake";
        self.transitions = {"Neutral":{"Neutral":1}}
        self.poses = {"Neutral": [([0,0,0,0],1,1.3,.6),
                                ([0,200,0,-200],1,1.3,.5),
                                ([400,0,-400,0],1,1.3,.6)]}
                                #([-193,0,0,193],1,.5,1),
                                #([0,775,-755,0],1,.5,1),
                                #([194,775,-775, -194],1,.5,1),
                                #([480,775,-775,-480],1,.5,1),
                                #([96,-484,484,-96],1,.5,1),([387,-387,387,-387],1,.5,1)]}
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

    with open("bird_config.json", "w") as f:
        f.write(r.toJSON())



if __name__ == "__main__":
    main()