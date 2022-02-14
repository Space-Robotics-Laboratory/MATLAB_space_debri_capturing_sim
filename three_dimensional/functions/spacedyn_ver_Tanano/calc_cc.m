function CC=calc_cc(LP,SV)

SV_temp=SV;
n=LP.num_q;
SV_temp.vd0 = [0 0 0]'; % ベースの加速度
SV_temp.wd0 = [0 0 0]'; % ベースの角加速度
SV_temp.qdd = zeros(n,1); % 関節角加速度
SV_temp.Fe = zeros(3,n); % 端点へかかる外力
SV_temp.Te = zeros(3,n); % 端点へかかるトルク

CC=r_ne(LP,SV_temp);
