# Space Debris Captureing Robot simulations
## Two Dimensional

![image2](/docs/media/simulationmodel.jpg)

## Description of the imprtant folders and files
```
|--two-dimensional
    |--[UNUSED]olr_references
    |--blender/DualArm Robot
        |--origin       : conserved original blender file
        |--dualArm.py   : python script run in blender
        |--dualArmRobo_moved.blend  : blender file of dyn result
        |--dualArmRobo.blend        : blender file of initial robot state
    |--dat
    |--docs
    |--lib
    |--parameters   : parameter setting
    |--src
        |--analysis : analysis using graph
        |--contact  : cont detection and calc force
        |--control  
            |--Pathway/Pathway.m    : A class to handle robo arm distination
            |--Contoroller.m        : A class to calc input motor torque
        |--datsave  
        |--estimation
        |--robot    : robot difinition and calc dyn
        |--target   : target difinition and calc dyn
        |--visualization
```


# Simulation Structure
In "main_sim" under "src", the function of the code can be devided into 4 big parts. 
- Saved Data Update Part
- Contact Caliculation and Estimation Part
- Control Part
- Dynamics Caliculation Part

## Contorol Part
In this simulation, the robot is contolled by input motor torque, thus it's requred to caliculate the desired torque in order to control robot as you want. 

### Controller Class
Inside the "control/Contoroller.m" file, the contoroller class is defined, and you can calculate input torque here as 'obj.tau'. 

You can use a function named 'calc_tau_from_ee_vel' to calculate torque by desired velocity with gerenaralized Jacobian.

You can use Pathway class to set the desired robot hands positions. 

### Pathway Class
Pathway class can handle robot hands target position with goal time. Pathway method, which is under Pathway class, contain DOF 3 desired position ( x, y, theta) and the time to end translation. You can add next goal to the pathway, so robot is able to move its hands in succession.

This class also has a function to calculate desired velocity by pathway, using defined velocity model. 

# Parameters Often Changed
## Control Parameters
```
controlParam.controlMode = 'MULTIPLE'; 
controlParam.velocityMode = 'str_tru';           
```
* controlMode : Now, 'DIRECT' and 'MULTIPLE' are available. 'DIRECT' is two hands direct caging, and 'MULTIPLE' is repeated one hand impedance contact, followed by two hands direct caging.

* velocityMode : Currently, 'str_str', 'str_tru' are available. ('str_fbk' is available too, but not useful) the former represents trajectory, the latter does speed model. For example, str_str model is straight trajectory with constant velocity, str_tru means straight trajectry with triangle speed-time graph. (constant acceleration)
