#!/usr/bin/env python

import rospy
import random
from Tkinter import *
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn, SetPen
from geometry_msgs.msg import Twist, Vector3

class Buttons:
    ON = 0
    OFF = 1

    def __init__(self):
        self.started = False
        rospy.wait_for_service('kill')
        rospy.wait_for_service('spawn')
        self.kill_service = rospy.ServiceProxy('kill', Kill)
        self.spawn_service = rospy.ServiceProxy('spawn', Spawn)

        # configuring button widget:
        # https://stackoverflow.com/questions/7591294/how-to-create-a-self-resizing-grid-of-buttons-in-tkinter
        widget = Tk()
        self.widget = widget
        frame = Frame(widget)
        Grid.rowconfigure(widget, 16, weight = 1)
        Grid.columnconfigure(widget, 3, weight = 1)
        frame.grid(row = 16, column = 3) #, sticky = N + S + E + W)
        grid = Frame(frame)
        grid.grid(sticky = N + S + E + W, column = 3, row = 16, columnspan = 3)
        Grid.rowconfigure(frame, 16, weight = 1)
        Grid.columnconfigure(frame, 3, weight = 1)

        self.robot_buttons = [None for i in range(32)] # holds buttons for toggling each robot
        self.x = [None for i in range(32)] # holds x coordinate turtle is currently supposed to be at
        self.y = [None for i in range(32)] # holds y coordinate turtle is currently supposed to be at
        self.state = [None for i in range(32)] # holds whether robots are ON or OFF the field
        self.name = ['robot' + str(i) for i in range(32)] # holds name of turtle for board removal purposes
        self.vel_pub = [rospy.Publisher(self.name[i] +'/cmd_vel', Twist, queue_size = 10) for i in range(32)]
        self.twist = [Twist(linear = Vector3(0, 0, 0), angular = Vector3(0, 0, 0)) for i in range(32)]

        def make_toggler(idx):
            return lambda : self.toggle(idx)

        # make individual robot buttons, without spawning the robots:
        for i in range(16):
            self.robot_buttons[i] = Button(frame, text = str(i), command = make_toggler(i))
            self.robot_buttons[i].grid(column = 1, row = i, sticky = N + S + E + W)
            # somehow add picture to button
        for i in range(16, 32):
            self.robot_buttons[i] = Button(frame, text = str(i), command = make_toggler(i))
            self.robot_buttons[i].grid(column = 2, row = i - 16, sticky = N + S + E + W)
            # somehow add picture to button

        self.newgame_button = Button(frame, text = 'new game', command = self.newgame)
        self.newgame_button.grid(column = 0, row = 0, sticky = N + S + E + W)

        self.stop_button = Button(frame, text = 'stop', command = self.stop)
        self.stop_button.grid(column = 0, row = 1, sticky = N + S + E + W)

        self.halt_button = Button(frame, text = 'halt', command = self.halt)
        self.halt_button.grid(column = 0, row = 2, sticky = N + S + E + W)

        self.normal_start_button = Button(frame, text = 'normal start', command = self.normal_start)
        self.normal_start_button.grid(column = 0, row = 3, sticky = N + S + E + W)

        self.kickoff_button = Button(frame, text = 'kick-off', command = self.kickoff)
        self.kickoff_button.grid(column = 0, row = 4, sticky = N + S + E + W)

        self.direct_freekick_button = Button(frame, text = 'direct free-kick', command = self.direct_freekick)
        self.direct_freekick_button.grid(column = 0, row = 5, sticky = N + S + E + W)

        self.indirect_freekick_button = Button(frame, text = 'indirect free-kick', command = self.indirect_freekick)
        self.indirect_freekick_button.grid(column = 0, row = 6, sticky = N + S + E + W)

        self.force_start_button = Button(frame, text = 'force start', command = self.force_start)
        self.force_start_button.grid(column = 0, row = 7, sticky = N + S + E + W)

        self.penalty_kick_button = Button(frame, text = 'penalty_kick', command = self.penalty_kick)
        self.penalty_kick_button.grid(column = 0, row = 8, sticky = N + S + E + W)

        self.autoball_button = Button(frame, text = 'auto ball placement', command = self.autoball)
        self.autoball_button.grid(column = 0, row = 9, sticky = N + S + E + W)

        self.shootout_button = Button(frame, text = 'shoot-out', command = self.shootout)
        self.shootout_button.grid(column = 0, row = 10, sticky = N + S + E + W)

        self.random_play_button = Button(frame, text = 'random play', command = self.random_play)
        self.random_play_button.grid(column = 0, row = 11, sticky = N + S + E + W)

        for c in range(3):
            Grid.columnconfigure(frame, c, weight = 1)
        for r in range(17):
            Grid.rowconfigure(frame, r, weight = 1)

        # for velocity publishing:
        for i in range(32):
            def make_publisher(i):
                return lambda x : self.vel_pub[i].publish(self.twist[i])
            _ = rospy.Subscriber(self.name[i]+'/pose', Pose, make_publisher(i))

        # place all robots in starting positions:
        self.newgame() # will set self.started to True

    def newgame(self):
        # first remove all remaining robots from field:
        for i in range(6):
            # blue team
            if self.state[i] == ON:
                self.toggle(i)
            self.state[i] = OFF
            self.x[i] = 4
            self.y[i] = 3*i + 1

        for i in range(16, 22):
            # yellow team
            if self.state[i] == ON:
                self.toggle(i)
            self.state[i] = OFF
            self.x[i] = 20
            self.y[i] = 3*(i - 16) + 1

        # then add all robots back onto field in starting positions:
        for i in range(6):
            # blue team
            self.toggle(i)

        for i in range(16, 22):
            # yellow team
            self.toggle(i)

        self.started = True

    def toggle(self, robot_idx):
        if self.state[robot_idx] == OFF:
            # somehow "press"/hold down button (color it green or something)
            _ = self.spawn_service(x = self.x[robot_idx],
                                   y = self.y[robot_idx],
                                   theta = 0,
                                   idx = robot_idx,
                                   name = self.name[robot_idx])
            self.state[robot_idx] = ON
            rospy.wait_for_service(self.name[robot_idx] + '/set_pen')
            _ = rospy.ServiceProxy(self.name[robot_idx] + '/set_pen', SetPen)(off = 1)
        elif self.state[robot_idx] == ON:
            # somehow "unpress" button
            _ = self.kill_service(self.name[robot_idx])
            self.state[robot_idx] = OFF
        else:
            # if state is still None, means we are initializing buttons
            pass

    def stop(self):
        pass

    def halt(self):
        # all robots must stop moving within 2 seconds
        # this implementation will need to be changed in real life or robots will tip over
        for i in range(32):
            self.twist[i] = Twist(linear = Vector3(0, 0, 0), angular = Vector3(0, 0, 0))

    def normal_start(self):
        pass

    def kickoff(self):
        pass

    def direct_freekick(self):
        pass

    def indirect_freekick(self):
        pass

    def force_start(self):
        pass

    def penalty_kick(self):
        pass

    def autoball(self):
        pass

    def shootout(self):
        pass

    def random_play(self):
        for i in range(32):
            self.twist[i] = Twist(linear = Vector3(random.randint(1, 10), random.randint(1, 10), random.randint(1, 10)),
                                  angular = Vector3(random.randint(1, 10), random.randint(1, 10), random.randint(1, 10)))


if __name__ == '__main__':
    node = rospy.init_node('buttons', anonymous = True)
    widget = Buttons()
    while not rospy.is_shutdown():
        try:
            widget.widget.mainloop()
        except rospy.ROSInterruptException:
            widget.widget.destroy()
            rospy.loginfo('node terminated')
