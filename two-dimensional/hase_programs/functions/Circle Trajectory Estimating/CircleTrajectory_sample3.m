%%%%%% �~�̋O�Ղ��璆�S���߂� �T���v�� %%%%%%  �^�[�Q�b�g����ڐG�̎��Ԃ̂ݎg�p��

clc
clear all
close all

global d_time
d_time = 0.0005;

% circle = linspace( 0, 2 * pi, 50 );

delta = 0.00000;

% ���m���
en = [ 0 0 ]';
en_tmp = zeros(2,1);
theta1_ini = 0;
theta2_ini = pi/2;
theta1 = theta1_ini;
theta2 = theta2_ini;
r1 = 0.12;
r2 = 0.15;
env = [ 5 5 ]';
enw = 1000;
enw = enw * pi/180;
% ���m���
ten1 = en + r1 * [ cos( theta1 ) sin( theta1 ) ]';
ten2 = en + r2 * [ cos( theta2 ) sin( theta2 ) ]';
ten1_tmp = zeros(2,1);
ten2_tmp = zeros(2,1);

% �\�����S�O��
en_est = zeros(2,1);
en_est_tmp = zeros(2,1);
en_error = 0;
normal_traj_est = zeros(2,1); % �Ƃ肠��������
normal_traj_est_tmp = zeros(2,1);
r1_est = 0;
r2_est = 0;
theta1_est = 0;
theta2_est = 0;
env_est = zeros(2,1);
enw_est = 0;
ten1_est = zeros(2,1);
ten2_est = zeros(2,1);
ten1_dash_est = zeros(2,1);
ten2_dash_est = zeros(2,1);

figure(1)
left =   400;   % �E�B���h�E�̈ʒu��ύX����ۂ̓R�R��������
bottom = 40;
width =  1000;
height = 950;
hold on;
axis equal;
grid on;
% grid minor;
box on;
position = [ left bottom width height ];
set( gcf, 'Position', position, 'Color', 'white', 'Resize', 'off', 'Renderer', 'OpenGL' );
set( gca, 'FontName', 'Times New Roman', 'FontSize', 30 );
xlim([ -.05 .8+.05 ]);
ylim([ -.05 .8+.05 ]);
xlabel( 'x [m]', 'FontName', 'Times New Roman', 'FontSize', 20 );
ylabel( 'y [m]', 'FontName', 'Times New Roman', 'FontSize', 20 );

writerObj = VideoWriter( 'C:\Users\falsi\Desktop\Orbital\2017_Orbital MT\180626\1.avi' );
set( writerObj, 'FrameRate', 40, 'Quality', 100 );
open( writerObj );

% normal_traj_estt_p = plot( normal_traj_est(1,1), normal_traj_est(2,1), 'g-o', 'MarkerFaceColor', 'g', 'LineWidth', 1 );
en_estx_p = en_est(1,1);
en_esty_p = en_est(2,1);
en_est_p = plot( en_estx_p, en_esty_p, 'y-o', 'MarkerFaceColor', 'y', 'LineWidth', 5 );
enx_p = en(1,1);
eny_p = en(2,1);
en_p = plot( enx_p, eny_p, 'k-o', 'LineWidth', 1.5 );
ten1x_p = ten1(1,1);
ten1y_p = ten1(2,1);
ten1_p = plot( ten1x_p, ten1y_p, 'r-o', 'LineWidth', 3 );
ten2x_p = ten2(1,1);
ten2y_p = ten2(2,1);
ten2_p = plot( ten2x_p, ten2y_p, 'b-o', 'LineWidth', 3 );
ten1_dash_estx_p = ten1_dash_est(1,1);
ten1_dash_esty_p = ten1_dash_est(2,1);
ten1_dash_est_p = plot( ten1_dash_estx_p, ten1_dash_esty_p, 'r+', 'LineWidth', 1 );
ten2_dash_estx_p = ten2_dash_est(1,1);
ten2_dash_esty_p = ten2_dash_est(2,1);
ten2_dash_est_p = plot( ten2_dash_estx_p, ten2_dash_esty_p, 'b+', 'LineWidth', 1 );
en_est_tmp = plot( [ en_est(1,1) en_est_tmp(1,1) ], [ en_est(2,1) en_est_tmp(2,1) ], 'k-', 'LineWidth', 1 );
en_tmp = plot( [ en(1,1) en_tmp(1,1) ], [ en(2,1) en_tmp(2,1) ], 'k-', 'LineWidth', 1 );
ten1_tmpx_p = [ ten1(1,1), ten1_tmp(1,1) ];
ten1_tmpy_p = [ ten1(2,1), ten1_tmp(2,1) ];
ten1_tmp_p = plot( ten1_tmpx_p, ten1_tmpy_p, 'r-', 'LineWidth', 1 );
ten2_tmpx_p = [ ten2(1,1), ten2_tmp(1,1) ];
ten2_tmpy_p = [ ten2(2,1), ten2_tmp(2,1) ];
ten2_tmp_p = plot( ten2_tmpx_p, ten2_tmpy_p, 'b-', 'LineWidth', 1 );
senx_p = [ en(1,1), ten1(1,1), ten2(1,1), en(1,1) ];
seny_p = [ en(2,1), ten1(2,1), ten2(2,1), en(2,1) ];
sen_p = plot( senx_p, seny_p, 'k-', 'LineWidth', 1 );

frame_n = 1;

set( en_est_p,  'EraseMode', 'normal' );
set( en_p, 'EraseMode', 'normal' );
set( ten1_p,  'EraseMode', 'normal' );
set( ten2_p, 'EraseMode', 'normal' );
% set( ten1_dash_est_p,  'EraseMode', 'normal' );
% set( ten2_dash_est_p, 'EraseMode', 'normal' );
set( ten1_tmp_p,  'EraseMode', 'normal' );
set( ten2_tmp_p, 'EraseMode', 'normal' );
set( sen_p,  'EraseMode', 'normal' );


% �V�~�����[�V�����X�^�[�g
for time = 0:d_time:0.15
% time

%%% �O��ۑ�
% ���m
en_tmp = en;
% ���m
ten1_tmp = ten1;
ten2_tmp = ten2;
normal_traj_est_tmp = normal_traj_est;
en_est_tmp = en_est;

%%% �X�V
% ���m
en = en + env * d_time;
theta1 = theta1 + enw * d_time;
theta2 = theta2 + enw * d_time;
% ���m
ten1 = en + r1 * [ cos( theta1 ) sin( theta1 ) ]' + [ randi(10,1) * delta randi(10,1) * delta ]';
ten2 = en + r2 * [ cos( theta2 ) sin( theta2 ) ]' + [ randi(10,1) * delta randi(10,1) * delta ]';

%%% �O�Ղ̐ڐ��̌X��
at_ten1_traj = ( ten1(2,1) - ten1_tmp(2,1) ) / ( ten1(1,1) - ten1_tmp(1,1) );
at_ten2_traj = ( ten2(2,1) - ten2_tmp(2,1) ) / ( ten2(1,1) - ten2_tmp(1,1) );

%%% �O�Ղ̖@���̌X���Ɛؕ�(ten��ten_tmp�̒��_��ʂ�@��)
an_ten1_traj = -1 / at_ten1_traj;
an_ten2_traj = -1 / at_ten2_traj;
bn_ten1_traj = ( ten1(2,1) + ten1_tmp(2,1) ) / 2 - an_ten1_traj * ( ten1(1,1) + ten1_tmp(1,1) ) / 2;
bn_ten2_traj = ( ten2(2,1) + ten2_tmp(2,1) ) / 2 - an_ten2_traj * ( ten2(1,1) + ten2_tmp(1,1) ) / 2;

%%% �@���̌�_�̍��W�����߂� ����1,2�̌�_
normal_traj_est(1,1) = -( bn_ten2_traj - bn_ten1_traj ) / ( an_ten2_traj - an_ten1_traj );
normal_traj_est(2,1) = ( an_ten2_traj * bn_ten1_traj - an_ten1_traj * bn_ten2_traj ) / ( an_ten2_traj - an_ten1_traj );

%%% ���葬�x�C�덷
env_est = ( normal_traj_est - normal_traj_est_tmp) / d_time;
env_error = env - env_est;
% env_error

%%% ten_tmp�����s�ړ���ten_dash_est����]��ten
ten1_dash_est = ten1_tmp + env_est * d_time;
ten2_dash_est = ten2_tmp + env_est * d_time;

%%% ���̌X��(ten��ten_dash_est��ʂ��)
a_ten1_gen = ( ten1(2,1) - ten1_dash_est(2,1) ) / ( ten1(1,1) - ten1_dash_est(1,1) );
a_ten2_gen = ( ten2(2,1) - ten2_dash_est(2,1) ) / ( ten2(1,1) - ten2_dash_est(1,1) );

%%% ���̖@���̌X���Ɛؕ�(ten��ten_dash_est�̒��_��ʂ�@��)
a_ten1_gen_normal = -1 / a_ten1_gen;
a_ten2_gen_normal = -1 / a_ten2_gen;
b_ten1_gen_normal = ( ten1(2,1) + ten1_dash_est(2,1) ) / 2 - a_ten1_gen_normal * ( ten1(1,1) + ten1_dash_est(1,1) ) / 2;
b_ten2_gen_normal = ( ten2(2,1) + ten2_dash_est(2,1) ) / 2 - a_ten2_gen_normal * ( ten2(1,1) + ten2_dash_est(1,1) ) / 2;

%%% ���̖@���̌�_(�~�̐�����W�C�덷)
en_est(1,1) = -( b_ten2_gen_normal - b_ten1_gen_normal ) / ( a_ten2_gen_normal - a_ten1_gen_normal );
en_est(2,1) = ( a_ten2_gen_normal * b_ten1_gen_normal - a_ten1_gen_normal * b_ten2_gen_normal ) / ( a_ten2_gen_normal - a_ten1_gen_normal );
en_error = en - en_est;
% en_error

%%% ���̒���L_est(ten��ten_dash_est)
L1_est = sqrt( ( ten1(1,1) -  ten1_dash_est(1,1) )^2 + ( ten1(2,1) -  ten1_dash_est(2,1) )^2 );
L2_est = sqrt( ( ten2(1,1) -  ten2_dash_est(1,1) )^2 + ( ten2(2,1) -  ten2_dash_est(2,1) )^2 );

%%% ���蔼�a�C����p���x�C����p�x�C���̌덷
r1_est = sqrt( ( en_est(1,1) -  ten1(1,1) )^2 + ( en_est(2,1) -  ten1(2,1) )^2 );
r2_est = sqrt( ( en_est(1,1) -  ten2(1,1) )^2 + ( en_est(2,1) -  ten2(2,1) )^2 );
r1_error = r1 - r1_est;
r2_error = r2 - r2_est;
% r1_error
% r2_error
enw_est = acos( 1 - L1_est^2 / ( 2 * r1_est^2 ) ) / d_time;
enw_error = enw - enw_est;
% enw_error

%%% �]���藝���琄��d�S�C�_1,2�̎O�p�`���̊p�x�𐄒�
R = sqrt( ( ten1(1,1) -  ten2(1,1) )^2 + ( ten1(2,1) -  ten2(2,1) )^2 );   % ten1��ten2�̋���
theta = acos( ( r1_est^2 + r2_est^2 - R^2 ) / ( 2 * r1_est * r2_est ) );
fai1 = acos( ( R^2 + r1_est^2 - r2_est^2 ) / ( 2 * r1_est * R ) );
fai2 = acos( ( R^2 + r2_est^2 - r1_est^2 ) / ( 2 * r2_est * R ) );
error_theta = ( theta1_ini - theta2_ini )*180/pi - ( theta )*180/pi;
% error_theta
% ( fai1 )*180/pi
% ( fai2 )*180/pi

en_estx_p = en_est(1,1);
en_esty_p = en_est(2,1);
enx_p = en(1,1);
eny_p = en(2,1);
ten1x_p = ten1(1,1);
ten1y_p = ten1(2,1);
ten2x_p = ten2(1,1);
ten2y_p = ten2(2,1);
ten1_dash_estx_p = ten1_dash_est(1,1);
ten1_dash_esty_p = ten1_dash_est(2,1);
ten2_dash_estx_p = ten2_dash_est(1,1);
ten2_dash_esty_p = ten2_dash_est(2,1);
ten1_tmpx_p = [ ten1(1,1), ten1_tmp(1,1) ];
ten1_tmpy_p = [ ten1(2,1), ten1_tmp(2,1) ];
ten2_tmpx_p = [ ten2(1,1), ten2_tmp(1,1) ];
ten2_tmpy_p = [ ten2(2,1), ten2_tmp(2,1) ];
senx_p = [ en(1,1), ten1(1,1), ten2(1,1), en(1,1) ];
seny_p = [ en(2,1), ten1(2,1), ten2(2,1), en(2,1) ];

set( en_est_p, 'XData', en_estx_p, 'YData', en_esty_p );
set( en_p, 'XData', enx_p, 'YData', eny_p );
set( ten1_p, 'XData', ten1x_p, 'YData', ten1y_p );
set( ten2_p, 'XData', ten2x_p, 'YData', ten2y_p );
set( ten1_dash_est_p, 'XData', ten1_dash_estx_p, 'YData', ten1_dash_esty_p );
set( ten2_dash_est_p, 'XData', ten2_dash_estx_p, 'YData', ten2_dash_esty_p );
set( ten1_tmp_p, 'XData', ten1_tmpx_p(1:2), 'YData', ten1_tmpy_p(1:2) );
set( ten2_tmp_p, 'XData', ten2_tmpx_p(1:2), 'YData', ten2_tmpy_p(1:2) );
set( sen_p, 'XData', senx_p(1:4), 'YData', seny_p(1:4) );

title( sprintf( 'Time = %0.3f [s]', time ), 'FontName', '���C���I', 'FontSize', 40 )
F( frame_n ) = getframe( 1 );
hold on
drawnow
writeVideo( writerObj, F( frame_n ) );
frame_n = frame_n + 1;

end

close( writerObj );

fig_ten1 = zeros(2,1);
fig_ten2 = [ R, 0 ]';
fig_en = fig_ten1 + r1_est * [ cos( fai1 ), sin( fai1 ) ]';

% figure( 2 )
% set( gcf, 'Position', position, 'Color', 'white', 'Resize', 'off', 'Renderer', 'OpenGL' );
% hold on
% plot( [ fig_ten1(1,1), fig_en(1,1)], [ fig_ten1(2,1), fig_en(2,1) ], 'r-', 'LineWidth', 3 )
% plot( [ fig_ten2(1,1), fig_en(1,1)], [ fig_ten2(2,1), fig_en(2,1) ], 'b-', 'LineWidth', 3 )
% plot( fig_ten1(1,1), fig_ten1(2,1), 'r-o', 'LineWidth', 5 )
% plot( fig_ten2(1,1), fig_ten2(2,1), 'b-o', 'LineWidth', 5 )
% plot( [ fig_ten1(1,1), fig_ten2(1,1)], [ fig_ten1(2,1), fig_ten2(2,1) ], 'k-', 'LineWidth', 2 )
% plot( fig_en(1,1), fig_en(2,1), 'g-o', 'MarkerFaceColor', 'k', 'LineWidth', 5 )
% xlim([ fig_ten1(1,1) - R/5 fig_ten2(1,1) + R/5 ]);
% legend( strcat('r_1=', num2str( r1_est ) ), strcat('r_2=', num2str( r2_est ) ) ), strcat('��_1=', num2str( fai1 * 180/pi ) ), strcat('��_2=', num2str( fai2 * 180/pi ) );
% grid on
% axis equal
