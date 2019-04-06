#!/usr/bin/env python

import rospy
from Tkinter import *
from simulator.srv import *

class Buttons:
    ON = 0
    OFF = 1

    def __init__(self):
        self.started = False
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

        self.robot_buttons = {i:None for i in range(32)} # holds buttons for toggling each robot
        self.x = {i:None for i in range(32)} # holds x coordinate turtle is currently supposed to be at
        self.y = {i:None for i in range(32)} # holds y coordinate turtle is currently supposed to be at
        self.state = {i:None for i in range(32)} # holds whether robots are ON or OFF the field
        self.name = {i:None for i in range(32)} # holds name of turtle for board removal purposes

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

        # stop
        # halt
        # normal start
        # kick-off
        # direct free-kick
        # indirect free-kick
        # force start
        # penalty kick
        # auto ball placement
        # shoot-out

        for c in range(3):
            Grid.columnconfigure(frame, c, weight = 1)
        for r in range(17):
            Grid.rowconfigure(frame, r, weight = 1)

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
            self.name[robot_idx] = self.spawn_service(x = self.x[robot_idx],
                                   y = self.y[robot_idx],
                                   theta = 0,
                                   idx = robot_idx).name
            self.state[robot_idx] = ON
        elif self.state[robot_idx] == ON:
            # somehow "unpress" button
            _ = self.kill_service(self.name[robot_idx])
            self.state[robot_idx] = OFF
        else:
            # if state is still None, means we are initializing buttons
            pass

if __name__ == '__main__':
    node = rospy.init_node('buttons', anonymous = True)
    widget = Buttons()
    while not rospy.is_shutdown():
        try:
            widget.widget.mainloop()
        except rospy.ROSInterruptException:
            widget.widget.destroy()
            rospy.loginfo('node terminated')
