#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped

class collisionObjects():
    def __init__(self):
        """Store all pose() and size data in two dictionaries."""
        # store all the data in these 2 dictionaries
        self.obj_pose_dict = {}
        self.obj_size_dict = {}

        # fridge ref frame relative to sawyer (centroid of the "fridge bottom panel" / fridge_b)
        self.x_fridge = 0.74
        self.y_fridge = -0.92
        self.z_fridge = -0.17

        #table ref frame relative to sawyer (centroid of "table"/ p )
        self.x_table = 0.4
        self.y_table = -1.0
        self.z_table = -0.211

        #table size
        self.t_length = 1.22
        self.t_width = 0.76
        self.t_thickness = 0.022

        #fridge size
        self.f_length = 0.39
        self.f_width = 0.47
        self.f_height = 0.46
        self.f_thickness = 0.03
        self.f_d_thickness = 0.04
        self.f_b_thickness = 0.19
        self.f_t_thickness = 0.25

        #handle dimensions
        self.f_h_left_of_center = 0.145
        self.f_h_forward_of_door = 0.05
        self.f_h_height_from_bottom_of_fridge_door = 0.22
        self.h_height = 0.1
        self.h_radius = 0.015
        # t_height, from base of the sawyer to the top of the table
        # t_height = -0.2

        # f_b_height: from top of table to bot of fridge
        # f_b_height = 0.015

        # table scene
        self.p = PoseStamped()
        # self.p.header.frame_id = robot.get_planning_frame()
        self.p.pose.position.x = self.x_table
        self.p.pose.position.y = self.y_table
        self.p.pose.position.z = self.z_table
        self.p_size = (self.t_length, self.t_width, self.t_thickness)
        self.add_to_dict("table",self.p,self.p_size)

        # fridge scene

        #fridge bottom panel
        self.f_b = PoseStamped()
        # self.f_b.header.frame_id = robot.get_planning_frame()
        self.f_b.pose.position.x = self.x_fridge
        self.f_b.pose.position.y = self.y_fridge
        self.f_b.pose.position.z = self.z_fridge
        self.f_b_size = (self.f_length, self.f_width, self.f_thickness)
        self.add_to_dict("fridge_back",self.f_b,self.f_b_size)

        #fridge top panel
        self.f_t = PoseStamped()
        # self.f_t.header.frame_id = robot.get_planning_frame()
        self.f_t.pose.position.x = self.x_fridge
        self.f_t.pose.position.y = self.y_fridge
        self.f_t.pose.position.z = self.z_fridge - self.f_thickness/2 + self.f_height - self.f_t_thickness/2
        self.f_t_size = (self.f_length, self.f_width, self.f_thickness)
        self.add_to_dict("fridge_top",self.f_t,self.f_t_size)

        #fridge_left
        self.f_l = PoseStamped()
        # self.f_l.header.frame_id = robot.get_planning_frame()
        self.f_l.pose.position.x = self.x_fridge
        self.f_l.pose.position.y = self.y_fridge - self.f_thickness/2 + self.f_width/2
        self.f_l.pose.position.z = self.z_fridge - self.f_thickness/2 + self.f_height/2
        self.f_l_size = (self.f_length, self.f_thickness, self.f_height)
        self.add_to_dict("fridge_left",self.f_l,self.f_l_size)

        #fridge_right
        self.f_r = PoseStamped()
        # self.f_l.header.frame_id = robot.get_planning_frame()
        self.f_r.pose.position.x = self.x_fridge
        self.f_r.pose.position.y = self.y_fridge + self.f_thickness/2 - self.f_width/2
        self.f_r.pose.position.z = self.z_fridge - self.f_thickness/2 + self.f_height/2
        self.f_r_size = (self.f_length, self.f_thickness, self.f_height)
        self.add_to_dict("fridge_right",self.f_r,self.f_r_size)

        #fridge_back
        self.f_b = PoseStamped()
        # self.f_b.header.frame_id = robot.get_planning_frame()
        self.f_b.pose.position.x = self.x_fridge - self.f_b_thickness/2 + self.f_length/2
        self.f_b.pose.position.y = self.y_fridge
        self.f_b.pose.position.z = self.z_fridge - self.f_thickness/2 + self.f_height/2
        self.f_b_size = (self.f_b_thickness, self.f_width, self.f_height)
        self.add_to_dict("fridge_bottom",self.f_b,self.f_b_size)

        #fridge_door (closed position)
        self.f_d = PoseStamped()
        # self.f_d.header.frame_id = robot.get_planning_frame()
        self.f_d.pose.position.x = self.x_fridge - self.f_d_thickness/2
        self.f_d.pose.position.y = self.y_fridge
        self.f_d.pose.position.z = self.z_fridge - self.f_thickness/2 + self.f_height/2
        self.f_d_size = (self.f_d_thickness, self.f_width, self.f_height)
        self.add_to_dict("fridge_door_closed",self.f_d,self.f_d_size)

        #handle
        self.f_h = PoseStamped()
        # self.f_h.header.frame_id = robot.get_planning_frame()
        self.f_h.pose.position.x = self.f_d.pose.position.x - self.f_h_forward_of_door
        self.f_h.pose.position.y = self.y_fridge - self.f_h_left_of_center
        self.f_h.pose.position.z = self.z_fridge - self.f_thickness/2 + self.f_h_height_from_bottom_of_fridge_door
        self.f_h_size = (self.h_height, self.h_radius)
        self.add_to_dict("fridge_handle",self.f_h,self.f_h_size)

        #TODO: add two bottle info - just size info
        #bottle_tall

        #bottle_can

    def add_to_dict(self,name,p,p_size):
        self.obj_pose_dict[name] = p
        self.obj_size_dict[name] = p_size
