% 双腕ロボ一般化慣性行列H*，一般化速度非線形項C*計算
%
% 2023.1 uchida akiyoshi
%

function[H_asuta, C_asuta] = calc_asuta_2arm(LP, SV, num_eL, num_eR)

% Calculate inertia matrices, HH
HH = calc_hh(LP, SV);
CC = calc_cc(LP, SV);

% Find joint connection from the end-link to the 0-th link
jointsL = j_num(LP, num_eL);
jointsR = j_num(LP, num_eR);

Hb = HH(1:6, 1:6);
Hbm_L = HH(1:6, 6 + jointsL);
Hbm_R = HH(1:6, 6 + jointsR);
Hm_L = HH(6 + jointsL, 6 + jointsL);
Hm_R = HH(6 + jointsR, 6 + jointsR);

Cb = CC(1:6, 1);
Cm_L = CC(6 + jointsL, 1);
Cm_R = CC(6 + jointsR, 1);

% 一般化慣性行列H*，一般化速度非線形項C*計算
H_asuta = [Hm_L - Hbm_L'*(Hb\Hbm_L),         -Hbm_L'*(Hb\Hbm_R)
                 -Hbm_R'*(Hb\Hbm_L),   Hm_R - Hbm_R'*(Hb\Hbm_R);];

C_asuta = [Cm_L - Hbm_L'*(Hb\Cb);
           Cm_R - Hbm_R'*(Hb\Cb)];
