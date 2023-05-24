function LP = TargetChohou_FourTips_LP()   % �Z�^�^�[�Q�b�g

% LP.BB = [0];
% LP.SS = [1];
% LP.S0 = [1];
% LP.SE = [1];
% LP.J_type = ['P'];
% LP.m = [mi];
LP.BB = [];% BB(i) = �������Ō������Ă��郊���N�ԍ�

LP.num_q = length(LP.BB);

%�x�[�X��(�����N0)�ɘA�����Ă��郊���N��1
%����ȊO��0
LP.S0 = zeros(1,LP.num_q);
LP.S0 = [];

% �Ίp�����͂��ׂ�-1
% �e�񂪍����炻�ꂼ�ꃊ���N�P�C�Q�C�R��\���Ă���
% �������Ō������Ă��郊���N�ԍ��ɑΉ�����s�ɂP�����
LP.SS = zeros(LP.num_q,LP.num_q);
LP.SS = [];
      
% ���[�����N�Ȃ�� 1
LP.SE = zeros(1,LP.num_q);
LP.SE = [];
% ��]�֐߂Ȃ�� 'R'
% �����֐߂Ȃ�� 'P'
LP.J_type = zeros(1,LP.num_q);
LP.J_type = [];

LP.m = zeros(1,LP.num_q);
LP.m = [];


% �x�[�X�̎��� [kg]
LP.m0 = 3.700 * 2;
% LP.m0 = 15.015;
%LP.m0 = 14.925;%����͕��Z�~�����ɂČv��������C�ł��Ƃ肠�������Qt�ɂ��킹��
% LP.m0 = mi;

LP.mass = sum(LP.m) + LP.m0;

% LP.Qi = [0 90 0]' * pi / 180;
% LP.Qe = [0 0 0]' * pi / 180;
% LP.c0 = [0.5 0 0]';
% LP.cc(:,1,1) = [0 0 -0.3]';
% LP.ce = [0 0 0.3]';
% �����̃����N���W�n����]������ ��̓����N�C�s��x��y��z����\��
LP.Qi = zeros(3,LP.num_q);

% ���[�����N�ƒ[�_�̉�]�֌W
% ���[���������Ȃ������N��0������
LP.Qe = zeros(3,LP.num_q);

%%%%% �x�[�X�d�S����֐�i�ւ̈ʒu�x�N�g�� %%%%%
% ���ژA�����Ă��Ȃ������N�͂��ׂ� 0
% �x�[�X�̍��W�n����݂�����
LP.c0 = zeros(3,LP.num_q);

%%%%% �e�����N�d�S����֐߂ւ̈ʒu�x�N�g�� %%%%%
LP.cc = zeros(3,LP.num_q,LP.num_q);

LP.ce = zeros(3,LP.num_q);

t = 1.5;
% �x�[�X�̊����e���\��
LP.inertia0 = [ 1e9 0   0;
                0   1e9 0;
                0   0   0.01074964594 * t ];

LP.inertia = zeros(3,3*LP.num_q);
