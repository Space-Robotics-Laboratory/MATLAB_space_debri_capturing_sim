function CC=calc_cc(LP,SV)

SV_temp=SV;
n=LP.num_q;
SV_temp.vd0 = [0 0 0]'; % �x�[�X�̉����x
SV_temp.wd0 = [0 0 0]'; % �x�[�X�̊p�����x
SV_temp.qdd = zeros(n,1); % �֐ߊp�����x
SV_temp.Fe = zeros(3,n); % �[�_�ւ�����O��
SV_temp.Te = zeros(3,n); % �[�_�ւ�����g���N

CC=r_ne(LP,SV_temp);
