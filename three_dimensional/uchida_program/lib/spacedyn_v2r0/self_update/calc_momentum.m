% 運動量計算関数
% 
% 2023.3 akiyoshi uchida
%

function PL = calc_momentum(LP, SV)

HH = calc_hh(LP, SV);

PL = HH(1:6, 1:6) * [SV.v0; SV.w0] + HH(1:6, 7:6+LP.num_q) * SV.qd;
end