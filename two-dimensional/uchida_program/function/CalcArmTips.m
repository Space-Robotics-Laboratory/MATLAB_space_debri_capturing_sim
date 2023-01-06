% 手先位置，端リンク姿勢からArmの二つの先端球位置を求める, 3次元
% Tips : [x1 x2
%         y1 y2
%         z1 z2]

function POS_es = CalcArmTips(POS_e, ORI_e, Param)
    Tip1 = POS_e + ORI_e * Param.LdH * sin(Param.LdGamma) * [ 1 0 0]';
    Tip2 = POS_e + ORI_e * Param.LdH * sin(Param.LdGamma) * [-1 0 0]';
    POS_es = [Tip1, Tip2];
end