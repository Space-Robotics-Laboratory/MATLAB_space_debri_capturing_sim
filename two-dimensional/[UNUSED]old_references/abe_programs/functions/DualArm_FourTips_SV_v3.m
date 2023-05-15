function SV_d = DualArm_FourTips_SV_v3( ~ ) % ~ �� void �ƈꏏ

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%% �ϐ��̏����� %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

n = 8;
SV_d.q = zeros(n,1);   % �֐ߊp
SV_d.qd  = zeros(n,1); % �֐ߊp���x
SV_d.qdd = zeros(n,1); % �֐ߊp�����x

SV_d.v0  = zeros(3,1); % �x�[�X�̕��i���x
SV_d.w0  = zeros(3,1); % �x�[�X�̊p���x
SV_d.vd0 = zeros(3,1); % �x�[�X�̉����x
SV_d.wd0 = zeros(3,1); % �x�[�X�̊p�����x

SV_d.vv = zeros(3,n); % �e�����N�̏d�S���x
SV_d.ww = zeros(3,n); % �e�����N�̏d�S�p���x
SV_d.vd = zeros(3,n); % �e�����N�̏d�S�����x
SV_d.wd = zeros(3,n); % �e�����N�̏d�S�p�����x

SV_d.R0 = zeros(3,1); % �x�[�X�̈ʒu
SV_d.Q0 = zeros(3,1); % �x�[�X�̊p�x (�I�C���[�p�\��)
SV_d.A0 = eye(3);     % �x�[�X�̊p�x (�����]���s��\��)  eye�͒P�ʍs��

SV_d.Fe = zeros(3,n); % �[�_�ւ�����O��
SV_d.Te = zeros(3,n); % �[�_�ւ�����g���N
SV_d.F0 = zeros(3,1); % �x�[�X�d�S�ւ�����O��
SV_d.T0 = zeros(3,1); % �x�[�X�d�S�ւ�����g���N

SV_d.tau = zeros(n,1); % �e�֐߂ւ����鎲�g���N


%%% EOF