% 速度非線形項計算
% 逆運動学的手法を用いる
% 2023.1　uchida akiyoshi, 関節数3のみであったものを，num＿qに拡張

function CC = calc_cc(LP, SV)

num_q = LP.num_q;
SV_tmp = SV;                    % 置換する.SVが更新されてしまうため
SV_tmp.vd0 = zeros(3,1);        % ベースの加速度
SV_tmp.wd0 = zeros(3,1);        % ベースの角加速度
SV_tmp.qdd = zeros(num_q,1);    % 関節角加速度

SV_tmp.Fe = zeros(3,num_q);     % 端点へかかる外力
SV_tmp.Te = zeros(3,num_q);     % 端点へかかるトルク

CC = r_ne(LP, SV_tmp);      % r_neはspacedynの関数
