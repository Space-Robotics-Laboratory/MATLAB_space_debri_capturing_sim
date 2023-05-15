# This is a blender python script to apply the motion of dualArm capturing simulation
# csv data calculated by MatLab uchida_program.
#
# 2023.3 Akiyoshi Uchida
#
# HOW TO USE
# Set csv dat file path in line 88.
# Set matlab simulation dt in line 90.
# Set blender animation frame rate in line 91. Be carefull to your machine max frame rate.
# Set target size in line 97, reset_target function. 
#
# IMPORTANT NOTE
# It's required to arange the header name in line 117 as the same order of CSV file.
#
#
#
#
#

import bpy
import csv
import math
import numpy as np

### functions
# relocate target to vis 2d calculated matlab position in 3d blender world
def relocate_target(target):
    # set target 'Z' same as the one of robo
    target.location = [0, 0, 0.055269]
    # activate Target
    bpy.context.view_layer.objects.active = target
    # redefine target frame
    bpy.ops.object.transform_apply(location=True)
    bpy.context.view_layer.update()

def reset_target(target_size):
    # remove target if it's already existnig
    exist_target = bpy.data.objects.get('Target')
    if exist_target is not None:
        bpy.data.objects.remove(exist_target, do_unlink=True)
    
    # remake target object
    bpy.ops.mesh.primitive_cube_add(size=target_size)
    cube_obj = bpy.context.object
    cube_obj.name = 'Target'
    return

# apply 3d vec location
def apply_location(obj, row, header_index, matching_indexes, frame_number):
    # get colmun number of first value of position vector
    col = matching_indexes[header_index]
    # all members of vector to float
    location = [float(i) for i in row[col:col+3]]
    # insert the location
    obj.location = location
    #insert the frame for location into timeline
    obj.keyframe_insert(data_path="location", frame=frame_number) 

# apply 3d vec orientation
def apply_orientation(obj, row, header_index, matching_indexes, frame_number):
    # get colmun number of first value of [roll, pitch, yaw] vector
    col = matching_indexes[header_index]
    # all members of vector to float
    rot_euler = [float(i) for i in row[col:col+3]]
#    rot_euler = np.rad2deg(rot_data)
    # insert the location
    obj.rotation_mode = 'XYZ'
    obj.rotation_euler = rot_euler
    #insert the frame for location into timeline
    obj.keyframe_insert(data_path="rotation_euler", frame=frame_number)  

# apply 8d  vec joint angle
def apply_angle(obj_list, row, header_index, matching_indexes, frame_number):
    # get colmun number of first value of [roll, pitch, yaw] vector
    col = matching_indexes[header_index]    
    angle = [float(i) for i in row[col:col+8]]
#    angle = np.rad2deg(angle_data)
    i = 0
    for obj in obj_list:
        obj.rotation_mode = 'XYZ'
        obj.rotation_euler = (0, 0, angle[i])
        obj.keyframe_insert(data_path="rotation_euler", frame=frame_number)
        i += 1


### main
# insert the file path of csv file generated by MATLAB in single inverted commas
csvfilename = "/Users/akiyoshi/develop/srl/github/MATLAB_space_debri_capturing_sim/two-dimensional/uchida_program/dat/2023-0330/blend/8_9000-dat/savedDat.csv"
# set animation frame rate
dt_simu = 0.001
frameRate_anime = 50
freq_anime = 1/frameRate_anime
freq_roop = 1/ (dt_simu * frameRate_anime)
bpy.context.scene.render.fps = frameRate_anime

# reset target 
reset_target(0.16)
T = bpy.data.objects["Target"]
relocate_target(T)

#Define the objects and bones in the model
armLeft = bpy.data.objects['ArmLeft'].pose.bones
armRight = bpy.data.objects['ArmRight'].pose.bones
linksLeft  = [ armLeft[link]  for link in armLeft.keys() ] 
linksRight = [ armRight[link] for link in armRight.keys() ] 
links = linksLeft + linksRight

R = bpy.data.objects["Robo"]
fn = 0
count = 0

with open(csvfilename, newline='', encoding='utf-8-sig') as csvfile:
    csvreader = csv.reader(csvfile, delimiter=',')
    headers = next(csvreader)
    # make header index
    # important: it's required to sort them as the same order of csv header
    search_headers = ["roboR0_1", "roboQ0_1", "jointAng_1", "targR0_1", "targQ0_1"]
    matching_indexes = [i for i, header in enumerate(headers) if header in search_headers]
    if headers != None:
        for row in csvreader:
            # write anime if time is devided by frequency
            if count % freq_roop == 0:
                # apply move on robot
                apply_location(R, row, 0, matching_indexes, fn)
                apply_orientation(R, row, 1, matching_indexes, fn)
                apply_angle(links, row, 2, matching_indexes, fn)
                # apply move on target
                apply_location(T, row, 3, matching_indexes, fn)
                apply_orientation(T, row, 4, matching_indexes, fn)
                fn += 1
            count += 1
bpy.context.scene.frame_end = fn-1
csvfile.close()