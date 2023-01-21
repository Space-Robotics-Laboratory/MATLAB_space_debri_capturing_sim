function CC = calc_cc(LP, SV)

SV_tmp = SV;                % 置換する.SVが更新されてしまうため
SV_tmp.vd0 = zeros(3,1);    % ベースの加速度
SV_tmp.wd0 = zeros(3,1);    % ベースの角加速度
SV_tmp.qdd = zeros(3,1);    % 関節角加速度

SV_tmp.Fe = zeros(3,3);     % 端点へかかる外力
SV_tmp.Te = zeros(3,3);     % 端点へかかるトルク

CC = r_ne(LP, SV_tmp);      % r_neはspacedynの関数
