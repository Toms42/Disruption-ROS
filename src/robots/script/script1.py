"""Demonstration script class for show1.py showing a self-contained process
for sequencing events over time.  The inputs and outputs are deliberately
constrained to message queues to preclude synchronization problems and maintain
compatibility with network communication.
"""

################################################################
# Written in 2019 by Garth Zeglin <garthz@cmu.edu>

# To the extent possible under law, the author has dedicated all copyright
# and related and neighboring rights to this software to the public domain
# worldwide. This software is distributed without any warranty.

# You should have received a copy of the CC0 Public Domain Dedication along with this software.
# If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.

################################################################

import time, queue
import numpy
import rcp.script
import Robot_Motions_Reader
import os.path


################################################################
class Script(rcp.script.Script):

    def __init__(self):
        super().__init__()
        return

    def script_task(self):
        """Entry point for the script to run on a background thread."""
        self.write("Script thread waking up.")

        # top-level event loop to wait for a play or reset command
        while True:
            try:
                command = self.input.get()
                if command[0] == 'console':
                    self.write("Script thread received user command: %s" % command[1])
                    if command[1] == 'reset':
                        self.send_reset_cue()

                    elif command[1] == 'play':
                        self.sequence()

                elif command[0] == 'script':
                    self.write("Script thread received network command: %s" % command[1])
                    if command[1] == 'start':
                        self.sequence()

                elif command[0] == 'status':
                    self.write("Script thread received status: %s" % repr(command))

            except rcp.script.ScriptStopException:
                self.write("Script stopped and idle.")

            except rcp.script.ScriptTimeoutException:
                self.write("Warning: script step timed out.  Script idle.")


    def send_show(self, *args):
        self.output.put(('show',) + args)
        self.write('Issuing show message: ' + repr(args))
        return

    def send_cue(self, *args):
        self.output.put(('cue',) + args)
        self.write('Issuing cue: ' + repr(args))
        return

    def send_pose(self, name):
        self.send_cue('pose',name)

    def send_reset_cue(self):
        self.write("Sending reset cue.")
        self.send_cue('gains', 0.5, 1.0)
        self.send_cue('pose', 'reset')
        self.send_cue('random', False)
        self.send_cue('tempo', 60.0)
        self.send_cue('magnitude', 1.0)
        return


    def sequence(self):
        """Demonstration sequence.  This could be decomposed further into subroutines."""
        snake = Robot_Motions_Reader.Robot("snake_config.json", 1)
        frog = Robot_Motions_Reader.Robot("frog_config.json",0)

        bird = Robot_Motions_Reader.Robot("bird_config.json",2)

        self.write("Script starting.")
        self.send_cue('gains', 0.5, 1.0)
        for index in range(3):
            self.output.put(('raw',index, [0,0,0,0])) #Get these robots hard
        sleep_dict = {"Snake" : 0,"Frog" : 0, "Bird": 0}
        robo_dict = {"Snake":snake,"Frog":frog, "Bird":bird}
        actually_start = False

        while True:
            if os.path.isfile('communicate.dat'):
                with open('communicate.dat', 'r') as f:
                    lines = f.readlines()
                    if len(lines) == 0:
                        break;
                    words = lines[-1].split(':')
                    #print(words)
                    robot_name = words[0] #TODO USE THIS VAR
                    actually_start = words[1] == "True"
                    print("we have read " + robot_name + " " + str(actually_start))
                    if actually_start:
                        with open('desires.dat', 'w+') as f:
                            for key in robo_dict.keys():
                                f.write(key + ":" + str(robo_dict[key].get_desire()) + ":\n")
                                #print(key + ":" + str(robo_dict[key].get_desire()))
                                if(robot_name == key):
                                    robo_dict[key].light_on(True)
                                else:
                                    robo_dict[key].light_on(False)


            if actually_start:
                min_sleep = min(sleep_dict.values()) 
                self.sleep(min_sleep)
                for key in sleep_dict:
                    value = sleep_dict[key]
                    if value == 0:
                        robot = robo_dict[key]
                        (pos,time,gain,damping) = robot.get_next_pose()

                        rand_damp = min(1, max(.1, numpy.random.normal(damping, .2, None)))
                        self.send_cue('gains', gain, rand_damp)
                        self.output.put(('raw', robot.motor_index, pos))
                        sleep_dict[key] = robot.get_time_to_sleep(time)

                    else:
                        sleep_dict[key] = value-min_sleep
            print('------------------------------------------------------------------')
            print('------------------------------------------------------------------')

            #print(pos)
            
            #self.send_pose('reset')
            #self.sleep(1.0)

            #self.send_pose('lead1')
            #self.sleep(1.0)

            #self.send_show('done')
            #self.write("Script done.")
        return

################################################################
