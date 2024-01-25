function [tl, tr] = calc_armTipsTorque(ORI_eL, ORI_eR, ExtWrench, param)

tipPos = [-param.robot.endEffector_h * cos(param.robot.endEffector_gamma), param.robot.endEffector_h * cos(param.robot.endEffector_gamma);
           param.robot.endEffector_h * sin(param.robot.endEffector_gamma), param.robot.endEffector_h * sin(param.robot.endEffector_gamma);
           0                                                             , 0                                                            ];
tl = cross(ORI_eL * tipPos(:, 1), ExtWrench(1:3)) + cross(ORI_eL * tipPos(:, 2), ExtWrench(1:3, 3)) ...
   + ExtWrench(4:6, 2) + ExtWrench(4:6, 3);
tr = cross(ORI_eR * tipPos(:, 1), ExtWrench(1:3, 4)) + cross(ORI_eR * tipPos(:, 2), ExtWrench(1:3, 5)) ...
   + ExtWrench(4:6, 4) + ExtWrench(4:6, 5);
end