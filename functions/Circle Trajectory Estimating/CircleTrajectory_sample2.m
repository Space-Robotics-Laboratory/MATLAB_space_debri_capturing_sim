%%%%%% �~�̋O�Ղ��璆�S���߂� �T���v�� %%%%%%

clc
clear all
close all

global d_time
d_time = 0.001;

circle = linspace( 0, 2 * pi, 50 );   % 0�`2�΂̋�Ԃ̓��Ԋu�̓_��enn�܂ލs�x�N�g����Ԃ�

%%% ���m���
en = [ 0 0 ]';
en_tmp = zeros(2,1);
theta1 = -pi / 6;
theta2 =  pi / 6;
theta3 = -pi / 2;
theta4 = -pi / 16;
r1 = 0.09;
r2 = 0.06;
r3 = 0.05;
r4 = 0.1;
env = [ 1 2 ]';
enw = 10;

%%% ���m���
ten1 = en + r1 * [ cos( theta1 ) sin( theta1 ) ]';
ten2 = en + r2 * [ cos( theta2 ) sin( theta2 ) ]';
ten3 = en + r3 * [ cos( theta3 ) sin( theta3 ) ]';
ten4 = en + r4 * [ cos( theta4 ) sin( theta4 ) ]';
ten1_tmp = zeros(2,1);
ten2_tmp = zeros(2,1);
ten3_tmp = zeros(2,1);
ten4_tmp = zeros(2,1);

% ten1_tmptmp = zeros(2,1);
% ten2_tmptmp = zeros(2,1);
% ten3_tmptmp = zeros(2,1);
% ten4_tmptmp = zeros(2,1);


% �\�����S�O��
en12_est = zeros(2,1);
en34_est = zeros(2,1);
en13_est = zeros(2,1);
en24_est = zeros(2,1);
en_est = zeros(2,1);
en_est_tmp = zeros(2,1);
en_error = 0;
normal_kiseki_est = zeros(2,1); % �Ƃ肠��������
normal_kiseki_est_tmp = zeros(2,1);
r1_est = 0;
r2_est = 0;
r3_est = 0;
r4_est = 0;
theta1_est = 0;
theta2_est = 0;
theta3_est = 0;
theta4_est = 0;
env_est = zeros(2,1);
enw_est = 0;
ten1_est = zeros(2,1);
ten2_est = zeros(2,1);
ten3_est = zeros(2,1);
ten4_est = zeros(2,1);
ten1_dash_est = zeros(2,1);
ten2_dash_est = zeros(2,1);
ten3_dash_est = zeros(2,1);
ten4_dash_est = zeros(2,1);

% r1_est = sqrt( ( ten1(1,1) - normal_kiseki_est(1,1) )^2 + ( ten1(2,1) - normal_kiseki_est(2,1) )^2 );
% r2_est = sqrt( ( ten2(1,1) - normal_kiseki_est(1,1) )^2 + ( ten2(2,1) - normal_kiseki_est(2,1) )^2 );
% r3_est = sqrt( ( ten3(1,1) - normal_kiseki_est(1,1) )^2 + ( ten3(2,1) - normal_kiseki_est(2,1) )^2 );
% r4_est = sqrt( ( ten4(1,1) - normal_kiseki_est(1,1) )^2 + ( ten4(2,1) - normal_kiseki_est(2,1) )^2 );
% theta1_est = atan( ( ten1(2,1) - normal_kiseki_est(2,1) ) / ( ten1(1,1) - normal_kiseki_est(1,1) ) );
% theta2_est = atan( ( ten2(2,1) - normal_kiseki_est(2,1) ) / ( ten2(1,1) - normal_kiseki_est(1,1) ) );
% theta3_est = atan( ( ten3(2,1) - normal_kiseki_est(2,1) ) / ( ten3(1,1) - normal_kiseki_est(1,1) ) );
% theta4_est = atan( ( ten4(2,1) - normal_kiseki_est(2,1) ) / ( ten4(1,1) - normal_kiseki_est(1,1) ) );
% error_ten1 = zeros(2,1);
% error_ten2 = zeros(2,1);
% error_ten3 = zeros(2,1);
% error_ten4 = zeros(2,1);
% error_theta1 = 0;
% error_theta2 = 0;
% error_theta3 = 0;
% error_theta4 = 0;
% error_v = zeros(2,1);
% error_w = 0;


% �V�~�����[�V�����X�^�[�g
for time = 0:d_time:0.1

% time


if time > d_time * 10

figure(1)
left =   400;
bottom = 30;
width =  750;
height = 750;
position = [ left bottom width height ];
set( gcf, 'Position', position, 'Color', 'white', 'Resize', 'off', 'Renderer', 'OpenGL' );

% plot( est_en12(1,1), est_en12(2,1), 'g-o', 'LineWidth', 1 );
% plot( est_en34(1,1), est_en34(2,1), 'g-o', 'LineWidth', 1 );
plot( en_est(1,1), en_est(2,1), 'g-o', 'MarkerFaceColor', 'g', 'LineWidth', 5 );
plot( en(1,1), en(2,1), 'b-o', 'LineWidth', 1 );

plot( ten1(1,1), ten1(2,1), 'r-o', 'LineWidth', 1 );
plot( ten2(1,1), ten2(2,1), 'm-o', 'LineWidth', 1 );
plot( ten3(1,1), ten3(2,1), 'y-o', 'LineWidth', 1 );
plot( ten4(1,1), ten4(2,1), 'c-o', 'LineWidth', 1 );

plot( ten1_dash_est(1,1), ten1_dash_est(2,1), 'r+', 'LineWidth', 1 );
plot( ten2_dash_est(1,1), ten2_dash_est(2,1), 'm+', 'LineWidth', 1 );
plot( ten3_dash_est(1,1), ten3_dash_est(2,1), 'y+', 'LineWidth', 1 );
plot( ten4_dash_est(1,1), ten4_dash_est(2,1), 'c+', 'LineWidth', 1 );

plot( [en_est(1,1) en_est_tmp(1,1)], [en_est(2,1) en_est_tmp(2,1)], 'k-', 'LineWidth', 1 );
plot( [en(1,1) en_tmp(1,1)], [en(2,1) en_tmp(2,1)], 'k-', 'LineWidth', 1 );

plot( [ten1(1,1) ten1_tmp(1,1)], [ten1(2,1) ten1_tmp(2,1)], 'r-', 'LineWidth', 1 );
plot( [ten2(1,1) ten2_tmp(1,1)], [ten2(2,1) ten2_tmp(2,1)], 'm-', 'LineWidth', 1 );
plot( [ten3(1,1) ten3_tmp(1,1)], [ten3(2,1) ten3_tmp(2,1)], 'y-', 'LineWidth', 1 );
plot( [ten4(1,1) ten4_tmp(1,1)], [ten4(2,1) ten4_tmp(2,1)], 'c-', 'LineWidth', 1 );


hold on
axis equal
% xlim([ -2 6 ]);
% ylim([ -2 6 ]);
grid on

end

en_error = en - en_est;
en_error

%%% �O��ۑ�
% ���m
en_tmp = en;
% ���m
ten1_tmp = ten1;
ten2_tmp = ten2;
ten3_tmp = ten3;
ten4_tmp = ten4;
normal_kiseki_est_tmp = normal_kiseki_est;
en_est_tmp = en_est;

%%% �X�V
% ���m
en = en + env * d_time;
theta1 = theta1 + enw * d_time;
theta2 = theta2 + enw * d_time;
theta3 = theta3 + enw * d_time;
theta4 = theta4 + enw * d_time;
% ���m
ten1 = en + r1 * [ cos( theta1 ) sin( theta1 ) ]';
ten2 = en + r2 * [ cos( theta2 ) sin( theta2 ) ]';
ten3 = en + r3 * [ cos( theta3 ) sin( theta3 ) ]';
ten4 = en + r4 * [ cos( theta4 ) sin( theta4 ) ]';

%%% �O�Ղ̐ڐ��̌X��
at_ten1 = ( ten1(2,1) - ten1_tmp(2,1) ) / ( ten1(1,1) - ten1_tmp(1,1) );
at_ten2 = ( ten2(2,1) - ten2_tmp(2,1) ) / ( ten2(1,1) - ten2_tmp(1,1) );
at_ten3 = ( ten3(2,1) - ten3_tmp(2,1) ) / ( ten3(1,1) - ten3_tmp(1,1) );
at_ten4 = ( ten4(2,1) - ten4_tmp(2,1) ) / ( ten4(1,1) - ten4_tmp(1,1) );

%%% �O�Ղ̖@���̌X���Ɛؕ�(ten��ten_tmp�̒��_��ʂ�@��)
an_ten1 = -1 / at_ten1;
an_ten2 = -1 / at_ten2;
an_ten3 = -1 / at_ten3;
an_ten4 = -1 / at_ten4;
bn_ten1 = ( ten1(2,1) + ten1_tmp(2,1) ) / 2 - an_ten1 * ( ten1(1,1) + ten1_tmp(1,1) ) / 2;
bn_ten2 = ( ten2(2,1) + ten2_tmp(2,1) ) / 2 - an_ten2 * ( ten2(1,1) + ten2_tmp(1,1) ) / 2;
bn_ten3 = ( ten3(2,1) + ten3_tmp(2,1) ) / 2 - an_ten3 * ( ten3(1,1) + ten3_tmp(1,1) ) / 2;
bn_ten4 = ( ten4(2,1) + ten4_tmp(2,1) ) / 2 - an_ten4 * ( ten4(1,1) + ten4_tmp(1,1) ) / 2;

%%% �@���̌�_�̍��W�����߂� ����1,2�̌�_�C����3,4�̌�_�C����1,3�̌�_�C����2,4�̌�_�̒��_
normal_kiseki12_est(1,1) = -( bn_ten2 - bn_ten1 ) / ( an_ten2 - an_ten1 );
normal_kiseki12_est(2,1) = ( an_ten2 * bn_ten1 - an_ten1 * bn_ten2 ) / ( an_ten2 - an_ten1 );
normal_kiseki34_est(1,1) = -( bn_ten4 - bn_ten3 ) / ( an_ten4 - an_ten3 );
normal_kiseki34_est(2,1) = ( an_ten4 * bn_ten3 - an_ten3 * bn_ten4 ) / ( an_ten4 - an_ten3 );
normal_kiseki13_est(1,1) = -( bn_ten3 - bn_ten1 ) / ( an_ten3 - an_ten1 );
normal_kiseki13_est(2,1) = ( an_ten3 * bn_ten1 - an_ten1 * bn_ten3 ) / ( an_ten3 - an_ten1 );
normal_kiseki24_est(1,1) = -( bn_ten4 - bn_ten2 ) / ( an_ten4 - an_ten2 );
normal_kiseki24_est(2,1) = ( an_ten4 * bn_ten2 - an_ten2 * bn_ten4 ) / ( an_ten4 - an_ten2 );
normal_kiseki_est(1,1) = ( normal_kiseki12_est(1,1) + normal_kiseki34_est(1,1) + normal_kiseki13_est(1,1) + normal_kiseki24_est(1,1) ) / 4;
normal_kiseki_est(2,1) = ( normal_kiseki12_est(2,1) + normal_kiseki34_est(2,1) + normal_kiseki13_est(2,1) + normal_kiseki24_est(2,1) ) / 4;

%%% ���x�\��
env_est = ( normal_kiseki_est - normal_kiseki_est_tmp) / d_time;
% %%% ten_tmp�����s�ړ���ten_dash_est����]��ten
% ten1_dash_est = ten1_tmp + env_est * d_time;
% ten2_dash_est = ten2_tmp + env_est * d_time;
% ten3_dash_est = ten3_tmp + env_est * d_time;
% ten4_dash_est = ten4_tmp + env_est * d_time;
% 
% %%% ���̌X��(ten��ten_dash_est��ʂ��)
% a_ten1_gen = ( ten1(2,1) - ten1_dash_est(2,1) ) / ( ten1(1,1) - ten1_dash_est(1,1) );
% a_ten2_gen = ( ten2(2,1) - ten2_dash_est(2,1) ) / ( ten2(1,1) - ten2_dash_est(1,1) );
% a_ten3_gen = ( ten3(2,1) - ten3_dash_est(2,1) ) / ( ten3(1,1) - ten3_dash_est(1,1) );
% a_ten4_gen = ( ten4(2,1) - ten4_dash_est(2,1) ) / ( ten4(1,1) - ten4_dash_est(1,1) );
% 
% %%% ���̖@���̌X���Ɛؕ�(ten��ten_dash_est�̒��_��ʂ�@��)
% a_ten1_gen_normal = -1 / a_ten1_gen;
% a_ten2_gen_normal = -1 / a_ten2_gen;
% a_ten3_gen_normal = -1 / a_ten3_gen;
% a_ten4_gen_normal = -1 / a_ten4_gen;
% b_ten1_gen_normal = ( ten1(2,1) + ten1_dash_est(2,1) ) / 2 - a_ten1_gen_normal * ( ten1(1,1) + ten1_dash_est(1,1) ) / 2;
% b_ten2_gen_normal = ( ten2(2,1) + ten2_dash_est(2,1) ) / 2 - a_ten2_gen_normal * ( ten2(1,1) + ten2_dash_est(1,1) ) / 2;
% b_ten3_gen_normal = ( ten3(2,1) + ten3_dash_est(2,1) ) / 2 - a_ten3_gen_normal * ( ten3(1,1) + ten3_dash_est(1,1) ) / 2;
% b_ten4_gen_normal = ( ten4(2,1) + ten4_dash_est(2,1) ) / 2 - a_ten4_gen_normal * ( ten4(1,1) + ten4_dash_est(1,1) ) / 2;
% 
% %%% ���̖@���̌�_(�~�̐�����W)
% en12_est(1,1) = -( b_ten2_gen_normal - b_ten1_gen_normal ) / ( a_ten2_gen_normal - a_ten1_gen_normal );
% en12_est(2,1) = ( a_ten2_gen_normal * b_ten1_gen_normal - a_ten1_gen_normal * b_ten2_gen_normal ) / ( a_ten2_gen_normal - a_ten1_gen_normal );
% en34_est(1,1) = -( b_ten4_gen_normal - b_ten3_gen_normal ) / ( a_ten4_gen_normal - a_ten3_gen_normal );
% en34_est(2,1) = ( a_ten4_gen_normal * b_ten3_gen_normal - a_ten3_gen_normal * b_ten4_gen_normal ) / ( a_ten4_gen_normal - a_ten3_gen_normal );
% en13_est(1,1) = -( b_ten3_gen_normal - b_ten1_gen_normal ) / ( a_ten3_gen_normal - a_ten1_gen_normal );
% en13_est(2,1) = ( a_ten3_gen_normal * b_ten1_gen_normal - a_ten1_gen_normal * b_ten3_gen_normal ) / ( a_ten3_gen_normal - a_ten1_gen_normal );
% en24_est(1,1) = -( b_ten4_gen_normal - b_ten2_gen_normal ) / ( a_ten4_gen_normal - a_ten2_gen_normal );
% en24_est(2,1) = ( a_ten4_gen_normal * b_ten2_gen_normal - a_ten2_gen_normal * b_ten4_gen_normal ) / ( a_ten4_gen_normal - a_ten2_gen_normal );
% en_est(1,1) = ( en12_est(1,1) + en34_est(1,1) + en13_est(1,1) + en24_est(1,1) ) / 4;
% en_est(2,1) = ( en12_est(2,1) + en34_est(2,1) + en13_est(2,1) + en24_est(2,1) ) / 4;

%%% ten_tmp�����s�ړ���ten_dash_est����]��ten
ten1_dash_est = ten1_tmp + env_est * d_time;
ten2_dash_est = ten2_tmp + env_est * d_time;
ten3_dash_est = ten3_tmp + env_est * d_time;
ten4_dash_est = ten4_tmp + env_est * d_time;

%%% ���̌X��(ten��ten_dash_est��ʂ��)
a_ten1_gen = ( ten1(2,1) - ten1_dash_est(2,1) ) / ( ten1(1,1) - ten1_dash_est(1,1) );
a_ten2_gen = ( ten2(2,1) - ten2_dash_est(2,1) ) / ( ten2(1,1) - ten2_dash_est(1,1) );
a_ten3_gen = ( ten3(2,1) - ten3_dash_est(2,1) ) / ( ten3(1,1) - ten3_dash_est(1,1) );
a_ten4_gen = ( ten4(2,1) - ten4_dash_est(2,1) ) / ( ten4(1,1) - ten4_dash_est(1,1) );

%%% ���̖@���̌X���Ɛؕ�(ten��ten_dash_est�̒��_��ʂ�@��)
a_ten1_gen_normal = -1 / a_ten1_gen;
a_ten2_gen_normal = -1 / a_ten2_gen;
a_ten3_gen_normal = -1 / a_ten3_gen;
a_ten4_gen_normal = -1 / a_ten4_gen;
b_ten1_gen_normal = ( ten1(2,1) + ten1_dash_est(2,1) ) / 2 - a_ten1_gen_normal * ( ten1(1,1) + ten1_dash_est(1,1) ) / 2;
b_ten2_gen_normal = ( ten2(2,1) + ten2_dash_est(2,1) ) / 2 - a_ten2_gen_normal * ( ten2(1,1) + ten2_dash_est(1,1) ) / 2;
b_ten3_gen_normal = ( ten3(2,1) + ten3_dash_est(2,1) ) / 2 - a_ten3_gen_normal * ( ten3(1,1) + ten3_dash_est(1,1) ) / 2;
b_ten4_gen_normal = ( ten4(2,1) + ten4_dash_est(2,1) ) / 2 - a_ten4_gen_normal * ( ten4(1,1) + ten4_dash_est(1,1) ) / 2;

%%% ���̖@���̌�_(�~�̐�����W)
en12_est(1,1) = -( b_ten2_gen_normal - b_ten1_gen_normal ) / ( a_ten2_gen_normal - a_ten1_gen_normal );
en12_est(2,1) = ( a_ten2_gen_normal * b_ten1_gen_normal - a_ten1_gen_normal * b_ten2_gen_normal ) / ( a_ten2_gen_normal - a_ten1_gen_normal );
en34_est(1,1) = -( b_ten4_gen_normal - b_ten3_gen_normal ) / ( a_ten4_gen_normal - a_ten3_gen_normal );
en34_est(2,1) = ( a_ten4_gen_normal * b_ten3_gen_normal - a_ten3_gen_normal * b_ten4_gen_normal ) / ( a_ten4_gen_normal - a_ten3_gen_normal );
en13_est(1,1) = -( b_ten3_gen_normal - b_ten1_gen_normal ) / ( a_ten3_gen_normal - a_ten1_gen_normal );
en13_est(2,1) = ( a_ten3_gen_normal * b_ten1_gen_normal - a_ten1_gen_normal * b_ten3_gen_normal ) / ( a_ten3_gen_normal - a_ten1_gen_normal );
en24_est(1,1) = -( b_ten4_gen_normal - b_ten2_gen_normal ) / ( a_ten4_gen_normal - a_ten2_gen_normal );
en24_est(2,1) = ( a_ten4_gen_normal * b_ten2_gen_normal - a_ten2_gen_normal * b_ten4_gen_normal ) / ( a_ten4_gen_normal - a_ten2_gen_normal );
en_est(1,1) = ( en12_est(1,1) + en34_est(1,1) + en13_est(1,1) + en24_est(1,1) ) / 4;
en_est(2,1) = ( en12_est(2,1) + en34_est(2,1) + en13_est(2,1) + en24_est(2,1) ) / 4;




end
