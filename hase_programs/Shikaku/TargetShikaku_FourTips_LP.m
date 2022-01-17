function LP = TargetShikaku_FourTips_LP()   % ���^�^�[�Q�b�g

LP.BB = [];   % BB(i) = �������Ō������Ă��郊���N�ԍ�

LP.num_q = length(LP.BB);

%�x�[�X��(�����N0)�ɘA�����Ă��郊���N��1�C����ȊO��0
LP.S0 = zeros(1,LP.num_q);
LP.S0 = [];

% �Ίp�����͂��ׂ�-1�C�e�񂪍����炻�ꂼ�ꃊ���N1,2,3��\���Ă���C�������Ō������Ă��郊���N�ԍ��ɑΉ�����s��1������
LP.SS = zeros(LP.num_q,LP.num_q);
LP.SS = [];
      
% ���[�����N�Ȃ�� 1
LP.SE = zeros(1,LP.num_q);
LP.SE = [];   % ��]�֐߂Ȃ�� 'R', �����֐߂Ȃ�� 'P'
LP.J_type = zeros(1,LP.num_q);
LP.J_type = [];

LP.m = zeros(1,LP.num_q);
LP.m = [];

% �x�[�X�̎��� [kg]
LP.m0 = 3.700 * 1;

LP.mass = sum(LP.m) + LP.m0;

% �����̃����N���W�n����]������ ��̓����N�C�s��x,y,z����\��
LP.Qi = zeros(3,LP.num_q);

% ���[�����N�ƒ[�_�̉�]�֌W�C���[���������Ȃ������N��0������
LP.Qe = zeros(3,LP.num_q);

% �x�[�X�d�S����֐�i�ւ̈ʒu�x�N�g��
% ���ژA�����Ă��Ȃ������N�͂��ׂ�0
% �x�[�X�̍��W�n����݂�����
LP.c0 = zeros(3,LP.num_q);

% �e�����N�d�S����֐߂ւ̈ʒu�x�N�g��
LP.cc = zeros(3,LP.num_q,LP.num_q);

LP.ce = zeros(3,LP.num_q);

% �x�[�X�̊����e���\��
tt = 3 * 1;
LP.inertia0 = [ 1e9 0   0;
                0   1e9 0;
                0   0   0.01163261858 * tt ];

LP.inertia = zeros(3,3*LP.num_q);
