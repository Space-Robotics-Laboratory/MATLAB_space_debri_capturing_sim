% フレーム変換
% input  : p_s(x;y;th), v_s(x;y;th)
% output : p_b(x;y;th), v_b(x;y;th)

function [pb, vb] = tf_s2b(ps, vs, robo)
p0 = robo.SV.R0;
th0 = robo.SV.Q0;
v0 = robo.SV.v0;
w0 = robo.SV.w0;

ws = vs(3) - w0(3);
r = [ps(1:2,1);0] - p0;

vb0 = cross(w0, r);
vb_ = vs - v0 - vb0;

vb = [vb_(1:2, 1); ws];
pb = [r(1:2, 1); th0(3)];

end