% 手先位置，端リンク姿勢・速度からArmの二つの先端球速度を求める, 3次元
% 
% 2023.1 uchida akiyoshi
% 
% Tips : [x1 x2 x3
%         y1 y2 y3
%         z1 z2 z3]

function VEL_es = calc_armTipsVel(VEL_e, ORI_e, ww_e, param)
    R = param.robot.radius_endEffector;
    L = param.robot.length_endEffector;
    th = pi/3;
    Tip1 = VEL_e + cross(ww_e, ORI_e * [-R*sin(th), L, -R*cos(th)]'); 
    Tip2 = VEL_e + cross(ww_e, ORI_e * [         0, L,          R]');
    Tip3 = VEL_e + cross(ww_e, ORI_e * [ R*sin(th), L, -R*cos(th)]');
    VEL_es = [Tip1, Tip2, Tip3];
end