% 手先位置，端リンク姿勢からArmの二つの先端球位置を求める, 3次元
% 
% 2023.1 uchida akiyoshi
% 
% 手先の初期値は末端フレーム表現(baseフレームと一致してるからあまり気にしなくて良い)
% Tips : [x1 x2 x3
%         y1 y2 y3
%         z1 z2 z3]

function POS_es = calc_armTipsPos(POS_e, ORI_e, param)
    R = param.robot.radius_endEffector;
    L = param.robot.length_endEffector;
    th = pi/3;
    Tip1 = POS_e + ORI_e * [-R*sin(th), L, -R*cos(th)]'; 
    Tip2 = POS_e + ORI_e * [         0, L,          R]';
    Tip3 = POS_e + ORI_e * [ R*sin(th), L, -R*cos(th)]';
    POS_es = [Tip1, Tip2, Tip3];
    
end