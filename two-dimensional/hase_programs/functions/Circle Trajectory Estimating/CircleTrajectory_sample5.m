%%%%%% 円の軌跡から中心求める サンプル %%%%%%  ターゲットが非接触の時間のみ使用可

clc
clear all
close all

global d_time
d_time = 0.005;

% circle = linspace( 0, 2 * pi, 50 );


% %%%%%%%%%% 値の読み込み %%%%%%%%%%
% path = 'C:\Users\falsi\Desktop\とりあえず\すぐ捨てる\1';
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
% title( '動点位置', 'FontSize', 15 );
% xlabel( 'x [m]' );
% ylabel( 'y [m]' );
% grid on
% axis equal


delta = 0.0000;

% 未知情報
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
% 既知情報
ten1 = en + r1 * [ cos( theta1 ) sin( theta1 ) 0 ]';
ten2 = en + r2 * [ cos( theta2 ) sin( theta2 ) 0 ]';
% ten1 = tmp_test( 1, 1:2 )';
% ten2 = tmp_test( 1, 3:4 )';
ten1_tmp = zeros(3,1);
ten2_tmp = zeros(3,1);

% 予測中心軌跡
en_est = zeros(3,1);
en_est_tmp = zeros(3,1);
en_error = 0;
normal_traj_est = zeros(3,1); % とりあえずおく
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


% シミュレーションスタート
for time = 0:d_time:.1

time

%%% 前回保存
% 未知
en_tmp = en;
% 既知
ten1_tmp = ten1;
ten2_tmp = ten2;
normal_traj_est_tmp = normal_traj_est;
en_est_tmp = en_est;

%%% 更新
% 未知
en = en + env * d_time;
theta1 = theta1 + enw * d_time;
theta2 = theta2 + enw * d_time;
% 既知
ten1 = en + r1 * [ cos( theta1 ) sin( theta1 ) 0 ]' + [ randi(10,1) * delta randi(10,1) * delta 0 ]';
ten2 = en + r2 * [ cos( theta2 ) sin( theta2 ) 0 ]' + [ randi(10,1) * delta randi(10,1) * delta 0 ]';
% ten1 = tmp_test( i, 1:2 )';
% ten2 = tmp_test( i, 3:4 )';

% %%% 軌跡の接線の傾き
% at_ten1_traj = ( ten1(2,1) - ten1_tmp(2,1) ) / ( ten1(1,1) - ten1_tmp(1,1) );
% at_ten2_traj = ( ten2(2,1) - ten2_tmp(2,1) ) / ( ten2(1,1) - ten2_tmp(1,1) );
% 
% %%% 軌跡の法線の傾きと切片(tenとten_tmpの中点を通る法線)
% an_ten1_traj = -1 / at_ten1_traj;
% an_ten2_traj = -1 / at_ten2_traj;
% bn_ten1_traj = ( ten1(2,1) + ten1_tmp(2,1) ) / 2 - an_ten1_traj * ( ten1(1,1) + ten1_tmp(1,1) ) / 2;
% bn_ten2_traj = ( ten2(2,1) + ten2_tmp(2,1) ) / 2 - an_ten2_traj * ( ten2(1,1) + ten2_tmp(1,1) ) / 2;
% 
% %%% 法線の交点の座標を求める 直線1,2の交点
% normal_traj_est(1,1) = -( bn_ten2_traj - bn_ten1_traj ) / ( an_ten2_traj - an_ten1_traj );
% normal_traj_est(2,1) = ( an_ten2_traj * bn_ten1_traj - an_ten1_traj * bn_ten2_traj ) / ( an_ten2_traj - an_ten1_traj );
% 
% %%% 推定速度，誤差
% env_est = ( normal_traj_est - normal_traj_est_tmp) / d_time;
% % env_error = env - env_est;
% % env_error
% 
% %%% ten_tmp→平行移動→ten_dash_est→回転→ten
% ten1_dash_est = ten1_tmp + env_est * d_time;
% ten2_dash_est = ten2_tmp + env_est * d_time;
% 
% %%% 弦の傾き(tenとten_dash_estを通る線)
% a_ten1_gen = ( ten1(2,1) - ten1_dash_est(2,1) ) / ( ten1(1,1) - ten1_dash_est(1,1) );
% a_ten2_gen = ( ten2(2,1) - ten2_dash_est(2,1) ) / ( ten2(1,1) - ten2_dash_est(1,1) );
% 
% %%% 弦の法線の傾きと切片(tenとten_dash_estの中点を通る法線)
% a_ten1_gen_normal = -1 / a_ten1_gen;
% a_ten2_gen_normal = -1 / a_ten2_gen;
% b_ten1_gen_normal = ( ten1(2,1) + ten1_dash_est(2,1) ) / 2 - a_ten1_gen_normal * ( ten1(1,1) + ten1_dash_est(1,1) ) / 2;
% b_ten2_gen_normal = ( ten2(2,1) + ten2_dash_est(2,1) ) / 2 - a_ten2_gen_normal * ( ten2(1,1) + ten2_dash_est(1,1) ) / 2;
% 
% %%% 弦の法線の交点(円の推定座標，誤差)
% en_est(1,1) = -( b_ten2_gen_normal - b_ten1_gen_normal ) / ( a_ten2_gen_normal - a_ten1_gen_normal );
% en_est(2,1) = ( a_ten2_gen_normal * b_ten1_gen_normal - a_ten1_gen_normal * b_ten2_gen_normal ) / ( a_ten2_gen_normal - a_ten1_gen_normal );
% % en_error = en - en_est;
% % en_error
% 
% %%% 弦の長さL_est(tenとten_dash_est)
% L1_est = sqrt( ( ten1(1,1) - ten1_dash_est(1,1) )^2 + ( ten1(2,1) - ten1_dash_est(2,1) )^2 );
% L2_est = sqrt( ( ten2(1,1) - ten2_dash_est(1,1) )^2 + ( ten2(2,1) - ten2_dash_est(2,1) )^2 );
% 
% %%% 推定半径，推定角速度，推定角度，その誤差
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
% %%% 余弦定理から推定重心，点1,2の三角形内の角度を推定
% R = sqrt( ( ten1(1,1) - ten2(1,1) )^2 + ( ten1(2,1) - ten2(2,1) )^2 );   % ten1とten2の距離
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
legend( strcat( ' r_1 = ', num2str( r1_est ), ' [m]' ), strcat( ' r_2 = ', num2str( r2_est ), ' [m]' ) , strcat( 'φ_1 = ', num2str( phi1 * 180/pi ), ' [deg]' ), strcat( 'φ_2 = ', num2str( phi2 * 180/pi ), ' [deg]' ) );
