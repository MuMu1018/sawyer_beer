# fridge ref frame relative to sawyer (centroid of the "fridge bottom panel" / fridge_b)
x_fridge = 0.74
y_fridge = -0.92
z_fridge = -0.17

#table ref frame relative to sawyer (centroid of "table"/ p )
x_table = 0.4
y_table = -1.0
z_table = -0.211

#table size
t_length = 1.22
t_width = 0.76
t_thickness = 0.022

#fridge size
f_length = 0.39
f_width = 0.47
f_height = 0.46
f_thickness = 0.03
f_d_thickness = 0.04
f_b_thickness = 0.19
f_t_thickness = 0.25

#handle dimensions
f_h_left_of_center = 0.145
f_h_forward_of_door = 0.05
f_h_height_from_bottom_of_fridge_door = 0.22
h_height = 0.1
h_radius = 0.015
# t_height, from base of the sawyer to the top of the table
# t_height = -0.2

# f_b_height: from top of table to bot of fridge
# f_b_height = 0.015

# table scene
p = PoseStamped()
p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = x_table
p.pose.position.y = y_table
p.pose.position.z = z_table
p_size = (t_length, t_width, t_thickness)

# fridge scene

#fridge bottom panel
f_b = PoseStamped()
f_b.header.frame_id = robot.get_planning_frame()
f_b.pose.position.x = x_fridge
f_b.pose.position.y = y_fridge
f_b.pose.position.z = z_fridge
f_b_size = (f_length, f_width, f_thickness)

#fridge top panel
f_t = PoseStamped()
f_t.header.frame_id = robot.get_planning_frame()
f_t.pose.position.x = x_fridge
f_t.pose.position.y = y_fridge
f_t.pose.position.z = z_fridge - f_thickness/2 + f_height - f_t_thickness/2
f_t_size = (f_length, f_width, f_thickness)

#fridge_left
f_l = PoseStamped()
f_l.header.frame_id = robot.get_planning_frame()
f_l.pose.position.x = x_fridge
f_l.pose.position.y = y_fridge - f_thickness/2 + f_width/2
f_l.pose.position.z = z_fridge - f_thickness/2 + f_height/2
f_l_size = (f_length, f_thickness, f_height)

#fridge_right
f_r = PoseStamped()
f_l.header.frame_id = robot.get_planning_frame()
f_r.pose.position.x = x_fridge
f_r.pose.position.y = y_fridge + f_thickness/2 - f_width/2
f_r.pose.position.z = z_fridge - f_thickness/2 + f_height/2
f_r_size = (f_length, f_thickness, f_height)

#fridge_back
f_b = PoseStamped()
f_b.header.frame_id = robot.get_planning_frame()
f_b.pose.position.x = x_fridge - f_b_thickness/2 + f_length/2
f_b.pose.position.y = y_fridge
f_b.pose.position.z = z_fridge - f_thickness/2 + f_height/2
f_b_size = (f_b_thickness, f_width, f_height)

#fridge_door (closed position)
f_d = PoseStamped()
f_d.header.frame_id = robot.get_planning_frame()
f_d.pose.position.x = x_fridge - f_d_thickness/2
f_d.pose.position.y = y_fridge
f_d.pose.position.z = z_fridge - f_thickness/2 + f_height/2
f_d_size = (f_d_thickness, f_width, f_height)

#handle
f_h = PoseStamped()
f_h.header.frame_id = robot.get_planning_frame()
f_h.pose.position.x = f_d.pose.position.x - f_h_forward_of_door
f_h.pose.position.y = y_fridge - f_h_left_of_center
f_h.pose.position.z = z_fridge - f_thickness/2 +f_h_height_from_bottom_of_fridge_door
f_h_size = (h_height, h_radius)
