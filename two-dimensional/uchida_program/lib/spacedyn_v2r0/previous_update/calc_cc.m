% ���x����`���v�Z
% �t�^���w�I��@��p����
% 2023.1�@uchida akiyoshi, �֐ߐ�3�݂̂ł��������̂��Cnum�Qq�Ɋg��

function CC = calc_cc(LP, SV)

num_q = LP.num_q;
SV_tmp = SV;                    % �u������.SV���X�V����Ă��܂�����
SV_tmp.vd0 = zeros(3,1);        % �x�[�X�̉����x
SV_tmp.wd0 = zeros(3,1);        % �x�[�X�̊p�����x
SV_tmp.qdd = zeros(num_q,1);    % �֐ߊp�����x

SV_tmp.Fe = zeros(3,num_q);     % �[�_�ւ�����O��
SV_tmp.Te = zeros(3,num_q);     % �[�_�ւ�����g���N

CC = r_ne(LP, SV_tmp);      % r_ne��spacedyn�̊֐�
