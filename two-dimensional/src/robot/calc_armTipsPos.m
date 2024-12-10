% 手先位置，端リンク姿勢からArmの二つの先端球位置を求める, 3次元
% 
% 2023.1 uchida akiyoshi
% 
% 初期値が左側の手先がtip1, 右がtip2
% Tips : [x1 x2
%         y1 y2
%         z1 z2]

function POS_es = calc_armTipsPos(POS_e, ORI_e, param)
    LdH = param.robot.endEffector_h;
    LdGamma = param.robot.endEffector_gamma;
    Tip1 = POS_e + ORI_e * LdH * sin(LdGamma) * [-1 0 0]';
    Tip2 = POS_e + ORI_e * LdH * sin(LdGamma) * [ 1 0 0]';
    POS_es = [Tip1, Tip2];
end