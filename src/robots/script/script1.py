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


    def Tom_update(self):
        return False,False

    def sequence(self):
        """Demonstration sequence.  This could be decomposed further into subroutines."""
        snake = Robot_Motions_Reader.Robot("snake_config.json", 1)
        frog = Robot_Motions_Reader.Robot("frog_config.json",0)

        bird = Robot_Motions_Reader.Robot("bird_config.json",2)

        self.write("Script starting.")
        self.send_cue('gains', 0.5, 1.0)
        for index in range(3):
            self.output.put(('raw',index, [0,0,0,0])) #Get these robots hard
        sleep_dict = {"Snake" : 100000,"Frog" : 100000, "Bird": 0}
        robo_dict = {"Snake":snake,"Frog":frog, "Bird":bird}
        actually_start = False;

        while True:
            actually_start,light_on = self.Tom_update()
            if light_on:
                pass
           # self.sleep(time)
         #   self.send_cue('gains', gain, damping)
           # self.output.put(('raw', 0, pos))
            if actually_start:
                min_sleep = min(sleep_dict.values()) 
                #res = [key for key in sleep_dict if sleep_dict[key] == temp] 
                print(min_sleep)
                self.sleep(min_sleep)
                for key in sleep_dict:
                    value = sleep_dict[key]
                    if value == 0:
                        robot = robo_dict[key]
                        (pos,time,gain,damping) = robot.get_next_pose()

                        rand_damp = min(1, max(.1, numpy.random.normal(damping, .2, None)))
                        self.send_cue('gains', gain, rand_damp)
                        self.output.put(('raw', robot.motor_index, pos))
                        sleep_dict[key] = time

                    else:
                        sleep_dict[key] = value-min_sleep

            #print(pos)
            
            #self.send_pose('reset')
            #self.sleep(1.0)

            #self.send_pose('lead1')
            #self.sleep(1.0)

            #self.send_show('done')
            #self.write("Script done.")
        return

################################################################
