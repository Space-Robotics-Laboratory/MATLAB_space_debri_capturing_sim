function CC = calc_cc(LP, SV)

SV_tmp = SV;                % �u������.SV���X�V����Ă��܂�����
SV_tmp.vd0 = zeros(3,1);    % �x�[�X�̉����x
SV_tmp.wd0 = zeros(3,1);    % �x�[�X�̊p�����x
SV_tmp.qdd = zeros(3,1);    % �֐ߊp�����x

SV_tmp.Fe = zeros(3,3);     % �[�_�ւ�����O��
SV_tmp.Te = zeros(3,3);     % �[�_�ւ�����g���N

CC = r_ne(LP, SV_tmp);      % r_ne��spacedyn�̊֐�
