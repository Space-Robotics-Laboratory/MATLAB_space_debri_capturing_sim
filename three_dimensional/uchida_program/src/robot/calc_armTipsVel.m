% 手先位置，端リンク姿勢・速度からArmの二つの先端球速度を求める, 3次元
% 
% 2023.1 uchida akiyoshi
% 
% 初期値が左側の手先がtip1, 右がtip2
% Tips : [x1 x2
%         y1 y2
%         z1 z2]

function VEL_es = calc_armTipsVel(VEL_e, ORI_e, W_e, Param)
    % ce1 = Param.LdH * sin(Param.LdGamma) * [-1 0 0]';
    % ce2 = Param.LdH * sin(Param.LdGamma) * [ 1 0 0]';
    % ce3 = 
    % Tip1 = VEL_e + cross(W_e, ORI_e * ce1);
    % Tip2 = VEL_e + cross(W_e, ORI_e * ce2);
    VEL_es = zeros(3,3);
end