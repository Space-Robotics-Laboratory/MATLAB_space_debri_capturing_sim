%%%%%% �~�̋O�Ղ��璆�S���߂� �T���v�� %%%%%%  �^�[�Q�b�g����ڐG�̎��Ԃ̂ݎg�p��

clc
clear all
close all

global d_time
d_time = 0.005;

% circle = linspace( 0, 2 * pi, 50 );


% %%%%%%%%%% �l�̓ǂݍ��� %%%%%%%%%%
% path = 'C:\Users\falsi\Desktop\�Ƃ肠����\�����̂Ă�\1';
% test_path = strcat( path, '\test4.dat' );
% fid_test = fopen( test_path, 'r' );
% tmp_test = fscanf( fid_test, '%g\t%g\t%g\t%g\n', [4 inf] );
% tmp_test = tmp_test';
% fclose( fid_test );
% 
% figure( 2 )
% set( gcf, 'Name', 'Position' );
% hold on
% plot( tmp_test( :, 1 ), tmp_test( :, 2 ), 'b-', 'LineWidth', 2 )
% plot( tmp_test( :, 3 ), tmp_test( :, 4 ), 'r-', 'LineWidth', 2 )
% title( '���_�ʒu', 'FontSize', 15 );
% xlabel( 'x [m]' );
% ylabel( 'y [m]' );
% grid on
% axis equal


delta = 0.0000;

% ���m���
en = [ 0 0 0 ]';zeros(3,1);
en_tmp = zeros(3,1);
theta1_ini = 0;
theta2_ini = 4*pi/7;
theta1 = theta1_ini;
theta2 = theta2_ini;
r1 = 0.3;
r2 = 0.25;
env = [ 1 1 0 ]';
enw = 500;
enw = enw * pi/180;
% ���m���
ten1 = en + r1 * [ cos( theta1 ) sin( theta1 ) 0 ]';
ten2 = en + r2 * [ cos( theta2 ) sin( theta2 ) 0 ]';
% ten1 = tmp_test( 1, 1:2 )';
% ten2 = tmp_test( 1, 3:4 )';
ten1_tmp = zeros(3,1);
ten2_tmp = zeros(3,1);

% �\�����S�O��
en_est = zeros(3,1);
en_est_tmp = zeros(3,1);
en_error = 0;
normal_traj_est = zeros(3,1); % �Ƃ肠��������
normal_traj_est_tmp = zeros(3,1);
r1_est = 0;
r2_est = 0;
theta1_est = 0;
theta2_est = 0;
env_est = zeros(3,1);
enw_est = zeros(3,1);
% ten1_est = zeros(2,1);
% ten2_est = zeros(2,1);
ten1_dash_est = zeros(3,1);
ten2_dash_est = zeros(3,1);


figure(1)
FS = 20;
left =   4;
bottom = 30;
width =  750;
height = 750;
hold on;axis equal;grid on;box on;grid minor;
position = [ left bottom width height ];
set( gcf, 'Position', position, 'Color', 'white', 'Resize', 'off', 'Renderer', 'OpenGL' );
set( gca, 'FontName', 'Times New Roman', 'FontSize', FS );

% plot( normal_traj_est(1,1), normal_traj_est(2,1), 'g-o', 'MarkerFaceColor', 'g', 'LineWidth', 1 );
% plot( en_est(1,1), en_est(2,1), 'g-o', 'MarkerFaceColor', 'g', 'LineWidth', 3 );
plot( en(1,1), en(2,1), 'k-o', 'LineWidth', 1 );

plot( ten1(1,1), ten1(2,1), 'r-o', 'LineWidth', 1 );
plot( ten2(1,1), ten2(2,1), 'b-o', 'LineWidth', 1 );

% plot( ten1_dash_est(1,1), ten1_dash_est(2,1), 'r+', 'LineWidth', 1 );
% plot( ten2_dash_est(1,1), ten2_dash_est(2,1), 'b+', 'LineWidth', 1 );

% plot( [ en_est(1,1) en_est_tmp(1,1) ], [ en_est(2,1) en_est_tmp(2,1) ], 'k-', 'LineWidth', 1 );
plot( [ en(1,1) en_tmp(1,1) ], [ en(2,1) en_tmp(2,1) ], 'k-', 'LineWidth', 1 );

% plot( [ ten1(1,1) ten1_tmp(1,1) ], [ ten1(2,1) ten1_tmp(2,1) ], 'r-', 'LineWidth', 1 );
% plot( [ ten2(1,1) ten2_tmp(1,1) ], [ ten2(2,1) ten2_tmp(2,1) ], 'm-', 'LineWidth', 1 );

plot( [ en(1,1) ten1(1,1) ten2(1,1) en(1,1) ], [ en(2,1) ten1(2,1) ten2(2,1) en(2,1) ], 'k-', 'LineWidth', 1 );

hold on
axis equal
xlabel( 'x [m]', 'FontName', 'Times New Roman', 'FontSize', FS );
ylabel( 'y [m]', 'FontName', 'Times New Roman', 'FontSize', FS );
% xlim([ -.05 .8 ]);
% ylim([ -.05 .8 ]);
grid on

i = 1;


% �V�~�����[�V�����X�^�[�g
for time = 0:d_time:.1

time

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
ten1 = en + r1 * [ cos( theta1 ) sin( theta1 ) 0 ]' + [ randi(10,1) * delta randi(10,1) * delta 0 ]';
ten2 = en + r2 * [ cos( theta2 ) sin( theta2 ) 0 ]' + [ randi(10,1) * delta randi(10,1) * delta 0 ]';
% ten1 = tmp_test( i, 1:2 )';
% ten2 = tmp_test( i, 3:4 )';

% %%% �O�Ղ̐ڐ��̌X��
% at_ten1_traj = ( ten1(2,1) - ten1_tmp(2,1) ) / ( ten1(1,1) - ten1_tmp(1,1) );
% at_ten2_traj = ( ten2(2,1) - ten2_tmp(2,1) ) / ( ten2(1,1) - ten2_tmp(1,1) );
% 
% %%% �O�Ղ̖@���̌X���Ɛؕ�(ten��ten_tmp�̒��_��ʂ�@��)
% an_ten1_traj = -1 / at_ten1_traj;
% an_ten2_traj = -1 / at_ten2_traj;
% bn_ten1_traj = ( ten1(2,1) + ten1_tmp(2,1) ) / 2 - an_ten1_traj * ( ten1(1,1) + ten1_tmp(1,1) ) / 2;
% bn_ten2_traj = ( ten2(2,1) + ten2_tmp(2,1) ) / 2 - an_ten2_traj * ( ten2(1,1) + ten2_tmp(1,1) ) / 2;
% 
% %%% �@���̌�_�̍��W�����߂� ����1,2�̌�_
% normal_traj_est(1,1) = -( bn_ten2_traj - bn_ten1_traj ) / ( an_ten2_traj - an_ten1_traj );
% normal_traj_est(2,1) = ( an_ten2_traj * bn_ten1_traj - an_ten1_traj * bn_ten2_traj ) / ( an_ten2_traj - an_ten1_traj );
% 
% %%% ���葬�x�C�덷
% env_est = ( normal_traj_est - normal_traj_est_tmp) / d_time;
% % env_error = env - env_est;
% % env_error
% 
% %%% ten_tmp�����s�ړ���ten_dash_est����]��ten
% ten1_dash_est = ten1_tmp + env_est * d_time;
% ten2_dash_est = ten2_tmp + env_est * d_time;
% 
% %%% ���̌X��(ten��ten_dash_est��ʂ��)
% a_ten1_gen = ( ten1(2,1) - ten1_dash_est(2,1) ) / ( ten1(1,1) - ten1_dash_est(1,1) );
% a_ten2_gen = ( ten2(2,1) - ten2_dash_est(2,1) ) / ( ten2(1,1) - ten2_dash_est(1,1) );
% 
% %%% ���̖@���̌X���Ɛؕ�(ten��ten_dash_est�̒��_��ʂ�@��)
% a_ten1_gen_normal = -1 / a_ten1_gen;
% a_ten2_gen_normal = -1 / a_ten2_gen;
% b_ten1_gen_normal = ( ten1(2,1) + ten1_dash_est(2,1) ) / 2 - a_ten1_gen_normal * ( ten1(1,1) + ten1_dash_est(1,1) ) / 2;
% b_ten2_gen_normal = ( ten2(2,1) + ten2_dash_est(2,1) ) / 2 - a_ten2_gen_normal * ( ten2(1,1) + ten2_dash_est(1,1) ) / 2;
% 
% %%% ���̖@���̌�_(�~�̐�����W�C�덷)
% en_est(1,1) = -( b_ten2_gen_normal - b_ten1_gen_normal ) / ( a_ten2_gen_normal - a_ten1_gen_normal );
% en_est(2,1) = ( a_ten2_gen_normal * b_ten1_gen_normal - a_ten1_gen_normal * b_ten2_gen_normal ) / ( a_ten2_gen_normal - a_ten1_gen_normal );
% % en_error = en - en_est;
% % en_error
% 
% %%% ���̒���L_est(ten��ten_dash_est)
% L1_est = sqrt( ( ten1(1,1) - ten1_dash_est(1,1) )^2 + ( ten1(2,1) - ten1_dash_est(2,1) )^2 );
% L2_est = sqrt( ( ten2(1,1) - ten2_dash_est(1,1) )^2 + ( ten2(2,1) - ten2_dash_est(2,1) )^2 );
% 
% %%% ���蔼�a�C����p���x�C����p�x�C���̌덷
% r1_est = sqrt( ( en_est(1,1) - ten1(1,1) )^2 + ( en_est(2,1) - ten1(2,1) )^2 );
% r2_est = sqrt( ( en_est(1,1) - ten2(1,1) )^2 + ( en_est(2,1) - ten2(2,1) )^2 );
% % r1_error = r1 - r1_est;
% % r2_error = r2 - r2_est;
% % r1_error
% % r2_error
% enw_est = acos( 1 - L1_est^2 / ( 2 * r1_est^2 ) ) / d_time;
% % enw_error = enw - enw_est;
% % enw_error
% 
% 
% %%% �]���藝���琄��d�S�C�_1,2�̎O�p�`���̊p�x�𐄒�
% R = sqrt( ( ten1(1,1) - ten2(1,1) )^2 + ( ten1(2,1) - ten2(2,1) )^2 );   % ten1��ten2�̋���
% theta = acos( ( r1_est^2 + r2_est^2 - R^2 ) / ( 2 * r1_est * r2_est ) );
% fai1 = acos( ( R^2 + r1_est^2 - r2_est^2 ) / ( 2 * r1_est * R ) );
% fai2 = acos( ( R^2 + r2_est^2 - r1_est^2 ) / ( 2 * r2_est * R ) );
% % error_theta = ( theta1_ini - theta2_ini )*180/pi - ( theta )*180/pi;
% % error_theta
% % ( fai1 )*180/pi
% % ( fai2 )*180/pi


[ en_est, env_est, enw_est, normal_traj_est, r1_est, r2_est, phi1, phi2, R_12, Theta_12 ] = ...
EstimateCOM( d_time, ten1, ten2, ten1_tmp, ten2_tmp, normal_traj_est_tmp );



if time > d_time * 1

figure(1)
left =   4;
bottom = 30;
width =  750;
height = 750;
hold on;axis equal;grid on;box on;
position = [ left bottom width height ];
set( gcf, 'Position', position, 'Color', 'white', 'Resize', 'off', 'Renderer', 'OpenGL' );

% plot( normal_traj_est(1,1), normal_traj_est(2,1), 'g-o', 'MarkerFaceColor', 'g', 'LineWidth', 1 );
plot( en_est(1,1), en_est(2,1), 'g-o', 'MarkerFaceColor', 'g', 'LineWidth', 100 );
plot( en(1,1), en(2,1), 'k-o', 'LineWidth', 1 );

plot( ten1(1,1), ten1(2,1), 'r-o', 'LineWidth', 1 );
plot( ten2(1,1), ten2(2,1), 'b-o', 'LineWidth', 1 );

% if time > d_time * 0
plot( ten1_dash_est(1,1), ten1_dash_est(2,1), 'r+', 'LineWidth', 1 );
plot( ten2_dash_est(1,1), ten2_dash_est(2,1), 'b+', 'LineWidth', 1 );
% end

% plot( [ en_est(1,1) en_est_tmp(1,1) ], [ en_est(2,1) en_est_tmp(2,1) ], 'k-', 'LineWidth', 10 );
plot( [ en(1,1) en_tmp(1,1) ], [ en(2,1) en_tmp(2,1) ], 'k-', 'LineWidth', 1 );

% plot( [ ten1(1,1) ten1_tmp(1,1) ], [ ten1(2,1) ten1_tmp(2,1) ], 'r-', 'LineWidth', 1 );
% plot( [ ten2(1,1) ten2_tmp(1,1) ], [ ten2(2,1) ten2_tmp(2,1) ], 'm-', 'LineWidth', 1 );

plot( [ en(1,1) ten1(1,1) ten2(1,1) en(1,1) ], [ en(2,1) ten1(2,1) ten2(2,1) en(2,1) ], 'k-', 'LineWidth', 1 );

title( sprintf( 'Time = %0.3f [s]', time ), 'FontName', 'Times New Roman', 'FontSize', FS )
% xlim([ -0.1 1.5 ]);
% ylim([ -0.1 1.5 ]);

end

end

fig_ten1 = zeros(2,1);
fig_ten2 = [ R_12, 0 ]';
fig_en = fig_ten1 + r1_est * [ cos( phi1 ), sin( phi1 ) ]';


%%
figure( 2 )
left =   770;
bottom = 30;
width =  750;
height = 750;
position = [ left bottom width height ];
set( gcf, 'Position', position, 'Color', 'white', 'Resize', 'off', 'Renderer', 'OpenGL' );
hold on;grid on;grid minor;box on;axis equal;
plot( [ fig_ten1(1,1), fig_en(1,1)], [ fig_ten1(2,1), fig_en(2,1) ], 'r-', 'LineWidth', 3 )
plot( [ fig_ten2(1,1), fig_en(1,1)], [ fig_ten2(2,1), fig_en(2,1) ], 'b-', 'LineWidth', 3 )
plot( fig_ten1(1,1), fig_ten1(2,1), 'ro', 'LineWidth', 5 )
plot( fig_ten2(1,1), fig_ten2(2,1), 'bo', 'LineWidth', 5 )
plot( [ fig_ten1(1,1), fig_ten2(1,1)], [ fig_ten1(2,1), fig_ten2(2,1) ], 'k-', 'LineWidth', 2 )
plot( fig_en(1,1), fig_en(2,1), 'g-o', 'MarkerFaceColor', 'g', 'LineWidth', 5 )
xlim([ fig_ten1(1,1) - R_12/5, fig_ten2(1,1) + R_12/5 ]);
legend( strcat( ' r_1 = ', num2str( r1_est ), ' [m]' ), strcat( ' r_2 = ', num2str( r2_est ), ' [m]' ) , strcat( '��_1 = ', num2str( phi1 * 180/pi ), ' [deg]' ), strcat( '��_2 = ', num2str( phi2 * 180/pi ), ' [deg]' ) );
