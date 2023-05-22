
* [UNUSED]old_references :
Files with InC at the end of the name are codes for impedance control. The calculation of torque control in the main code was changed.
    * abeprograms :

        * functions :
            contains code for functions used to calculate equations of motion and code for SpaceDyn
            In particular, the following two codes are often used because they are related to the setup of the dual-arm space robot model.
        * DualArm_FourTips_LP_v3.m
        * DualArm_FourTips_SV_v3.m

    * Shikaku :
        * main_DualArm_TargetShikaku_FourTips_v07.m :
        main code for capture simulation
        
        * target_LP_SV :
        contains codes used to set the target parameters
        
        * desired_hand_velocity_calculation :
        contains codes to calculate the desired end-effector's velocity of the dual-arm space robot
        
        * contact_physics :
        contains codes for making contact decisions and calculating contact forces
        
        * figure_and_video_creation :
        contains codes to output a figure for each parameter and codes to create a capture simulation movie


## Parameters that change a lot of times
* ts_W_i :
Angular velocity of the target in inertial coordinate system

* The second line of the vector zeta :
Simulation time

* kw_1 :
Coefficient of rigidity

* cw_1 :
Viscosity coefficient (damping coefficient)
