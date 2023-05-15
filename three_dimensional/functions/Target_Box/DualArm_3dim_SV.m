function SV = DualArm_3dim_SV( LP ) % ~ �� void �ƈꏏ

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%% �ϐ��̏����� %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

n = LP.num_q;
SV.q = zeros(n,1);   % �֐ߊp
SV.qd  = zeros(n,1); % �֐ߊp���x
SV.qdd = zeros(n,1); % �֐ߊp�����x

SV.v0  = zeros(3,1); % �x�[�X�̕��i���x
SV.w0  = zeros(3,1); % �x�[�X�̊p���x
SV.vd0 = zeros(3,1); % �x�[�X�̉����x
SV.wd0 = zeros(3,1); % �x�[�X�̊p�����x

SV.vv = zeros(3,n); % �e�����N�̏d�S���x
SV.ww = zeros(3,n); % �e�����N�̏d�S�p���x
SV.vd = zeros(3,n); % �e�����N�̏d�S�����x
SV.wd = zeros(3,n); % �e�����N�̏d�S�p�����x

SV.R0 = zeros(3,1); % �x�[�X�̈ʒu
SV.Q0 = zeros(3,1); % �x�[�X�̊p�x (�I�C���[�p�\��)
SV.A0 = eye(3);     % �x�[�X�̊p�x (�����]���s��\��)  eye�͒P�ʍs��

SV.Fe = zeros(3,n); % �[�_�ւ�����O��
SV.Te = zeros(3,n); % �[�_�ւ�����g���N
SV.F0 = zeros(3,1); % �x�[�X�d�S�ւ�����O��
SV.T0 = zeros(3,1); % �x�[�X�d�S�ւ�����g���N

SV.tau = zeros(n,1); % �e�֐߂ւ����鎲�g���N


%%% EOF