%%%%%% 円の軌跡から中心求める サンプル %%%%%%

clc
clear all
close all

global d_time

d_time = 0.001;
circle = linspace( 0, 2 * pi, 50 );   % 0～2πの区間の等間隔の点をenn個含む行ベクトルを返す
enx = 0;
eny = 0;
entheta1 = -pi / 6;
entheta2 = -pi-pi / 6;
entheta3 = -pi / 2;
entheta4 = -pi / 16;
radius1 = 0.09;
radius2 = .06;
radius3 = .05;
radius4 = 0.1;
en = [ enx eny ]';
ten1 = en + radius1 * [ cos( entheta1 ) sin( entheta1 ) ]';
ten2 = en + radius2 * [ cos( entheta2 ) sin( entheta2 ) ]';
ten3 = en + radius3 * [ cos( entheta3 ) sin( entheta3 ) ]';
ten4 = en + radius4 * [ cos( entheta4 ) sin( entheta4 ) ]';
en_tmp = zeros(2,1);
norkiseki_est_tmp = zeros(2,1);
ten1_tmp = zeros(2,1);
ten2_tmp = zeros(2,1);
ten3_tmp = zeros(2,1);
ten4_tmp = zeros(2,1);
ten1_tmptmp = zeros(2,1);
ten2_tmptmp = zeros(2,1);
ten3_tmptmp = zeros(2,1);
ten4_tmptmp = zeros(2,1);
env = [1,2]';
enw = 100;



% 予測中心軌跡
norkiseki_est = zeros(2,1); % とりあえずおく
en34_est = zeros(2,1);
en12_est = zeros(2,1);
r1_est = sqrt( ( ten1(1,1) - norkiseki_est(1,1) )^2 + ( ten1(2,1) - norkiseki_est(2,1) )^2 );
r2_est = sqrt( ( ten2(1,1) - norkiseki_est(1,1) )^2 + ( ten2(2,1) - norkiseki_est(2,1) )^2 );
r3_est = sqrt( ( ten3(1,1) - norkiseki_est(1,1) )^2 + ( ten3(2,1) - norkiseki_est(2,1) )^2 );
r4_est = sqrt( ( ten4(1,1) - norkiseki_est(1,1) )^2 + ( ten4(2,1) - norkiseki_est(2,1) )^2 );
theta1_est = atan( ( ten1(2,1) - norkiseki_est(2,1) ) / ( ten1(1,1) - norkiseki_est(1,1) ) );
theta2_est = atan( ( ten2(2,1) - norkiseki_est(2,1) ) / ( ten2(1,1) - norkiseki_est(1,1) ) );
theta3_est = atan( ( ten3(2,1) - norkiseki_est(2,1) ) / ( ten3(1,1) - norkiseki_est(1,1) ) );
theta4_est = atan( ( ten4(2,1) - norkiseki_est(2,1) ) / ( ten4(1,1) - norkiseki_est(1,1) ) );

v_est = [1,1];
w_est = 1;

ten1_est(1,1) = ten1(1,1) - r1_est * cos( theta1_est ) + v_est(1,1) * d_time + r1_est * cos( theta1_est + w_est * d_time );
ten2_est(1,1) = ten2(1,1) - r2_est * cos( theta1_est ) + v_est(1,1) * d_time + r2_est * cos( theta2_est + w_est * d_time );
ten3_est(1,1) = ten3(1,1) - r3_est * cos( theta1_est ) + v_est(1,1) * d_time + r3_est * cos( theta3_est + w_est * d_time );
ten4_est(1,1) = ten4(1,1) - r4_est * cos( theta1_est ) + v_est(1,1) * d_time + r4_est * cos( theta4_est + w_est * d_time );

error_ten1 = zeros(2,1);
error_ten2 = zeros(2,1);
error_ten3 = zeros(2,1);
error_ten4 = zeros(2,1);
error_theta1 = 0;
error_theta2 = 0;
error_theta3 = 0;
error_theta4 = 0;

error_v = zeros(2,1);
error_w = 0;




% シミュレーションスタート
for time = 0:d_time:3

% time

figure(1)
left =   400;
bottom = 30;
width =  750;
height = 750;
position = [ left bottom width height ];
set( gcf, 'Position', position, 'Color', 'white', 'Resize', 'off', 'Renderer', 'OpenGL' );

% plot( est_en12(1,1), est_en12(2,1), 'g-o', 'LineWidth', 1 );
% plot( est_en34(1,1), est_en34(2,1), 'g-o', 'LineWidth', 1 );
plot( norkiseki_est(1,1), norkiseki_est(2,1), 'g-o', 'LineWidth', 1 );
plot( en(1,1), en(2,1), 'b-o', 'LineWidth', 1 );
% plot( est_en34(1,1), est_en34(2,1), 'g-o', 'MarkerFaceColor', 'g', 'LineWidth', 1 );
% plot( en(1,1), en(2,1), 'b-o', 'MarkerFaceColor', 'b', 'LineWidth', 2 );
% plot( en(1,1) + radius1 * sin( circle ), en(2,1) + radius1 * cos( circle ), 'b-', 'LineWidth', 1 );

% plot( ten1(1,1), ten1(2,1), 'r-o', 'MarkerFaceColor', 'r', 'LineWidth', 1 );
% plot( ten2(1,1), ten2(2,1), 'm-o', 'MarkerFaceColor', 'm', 'LineWidth', 1 );
% plot( ten3(1,1), ten3(2,1), 'k-o', 'MarkerFaceColor', 'k', 'LineWidth', 1 );
% plot( ten4(1,1), ten4(2,1), 'c-o', 'MarkerFaceColor', 'g', 'LineWidth', 1 );
plot( ten1(1,1), ten1(2,1), 'r-o', 'LineWidth', 1 );
plot( ten2(1,1), ten2(2,1), 'm-o', 'LineWidth', 1 );
plot( ten3(1,1), ten3(2,1), 'y-o', 'LineWidth', 1 );
plot( ten4(1,1), ten4(2,1), 'c-o', 'LineWidth', 1 );

% plot( [en(1,1) ten1(1,1)], [en(2,1) ten1(2,1)], 'k-', 'LineWidth', 1 );
% plot( [est_en12(1,1) ten1(1,1)], [est_en12(2,1) ten1(2,1)], 'r-', 'LineWidth', 1 );
% plot( [ten1(1,1) ten1_tmp(1,1)], [ten1(2,1) ten1_tmp(2,1)], 'r-', 'LineWidth', 1 );
% plot( [en(1,1) ten2(1,1)], [en(2,1) ten2(2,1)], 'k-', 'LineWidth', 1 );
% plot( [est_en12(1,1) ten2(1,1)], [est_en12(2,1) ten2(2,1)], 'm-', 'LineWidth', 1 );
plot( [en(1,1) en_tmp(1,1)], [en(2,1) en_tmp(2,1)], 'b-', 'LineWidth', 1 );
% plot( [en_est(1,1) en_est_tmp(1,1)], [en_est(2,1) en_est_tmp(2,1)], 'g-', 'LineWidth', 1 );
% plot( [en_est(1,1) en(1,1)], [en_est(2,1) en(2,1)], 'k-', 'LineWidth', 1 );
plot( [ten1(1,1) ten1_tmp(1,1)], [ten1(2,1) ten1_tmp(2,1)], 'r-', 'LineWidth', 1 );
plot( [ten2(1,1) ten2_tmp(1,1)], [ten2(2,1) ten2_tmp(2,1)], 'm-', 'LineWidth', 1 );
plot( [ten3(1,1) ten3_tmp(1,1)], [ten3(2,1) ten3_tmp(2,1)], 'y-', 'LineWidth', 1 );
plot( [ten4(1,1) ten4_tmp(1,1)], [ten4(2,1) ten4_tmp(2,1)], 'c-', 'LineWidth', 1 );
% plot( [ten1(1,1), ten3(1,1), ten2(1,1), ten4(1,1), ten1(1,1)], [ten1(2,1), ten3(2,1), ten2(2,1), ten4(2,1), ten1(2,1)], 'k-', 'LineWidth', 1 );
% plot( [ten1(1,1), en(1,1), ten3(1,1), ten2(1,1), ten4(1,1), ten1(1,1)], [ten1(2,1), en(2,1), ten3(2,1), ten2(2,1), ten4(2,1), ten1(2,1)], 'k-', 'LineWidth', 1 );
% % % % % % % plot( [ten1(1,1), en(1,1), ten3(1,1), en(1,1), ten2(1,1), en(1,1), ten4(1,1), en(1,1), ten1(1,1)], [ten1(2,1), en(2,1), ten3(2,1), en(2,1), ten2(2,1), en(2,1), ten4(2,1), en(2,1), ten1(2,1)], 'k-', 'LineWidth', 1 );

% plot( ten3(1,1), ten3(2,1), 'k-o', 'LineWidth', 1 );
% plot( ten4(1,1), ten4(2,1), 'c-o', 'LineWidth', 1 );

hold on
axis equal
% xlim([ -2 6 ]);
% ylim([ -2 6 ]);
grid on

% 前回保存
en_tmp = en;
ten1_tmptmp = ten1_tmp;
ten2_tmptmp = ten2_tmp;
ten3_tmptmp = ten3_tmp;
ten4_tmptmp = ten4_tmp;
ten1_tmp = ten1;
ten2_tmp = ten2;
ten3_tmp = ten3;
ten4_tmp = ten4;

% 更新
en = en + [ env(1,1) * d_time env(2,1) * d_time ]';
entheta1 = entheta1 + enw * d_time;
entheta2 = entheta2 + enw * d_time;
entheta3 = entheta3 + enw * d_time;
entheta4 = entheta4 + enw * d_time;
ten1 = en + radius1 * [ cos( entheta1 ) sin( entheta1 ) ]';
ten2 = en + radius2 * [ cos( entheta2 ) sin( entheta2 ) ]';
ten3 = en + radius3 * [ cos( entheta3 ) sin( entheta3 ) ]';
ten4 = en + radius4 * [ cos( entheta4 ) sin( entheta4 ) ]';


% 
% % 予測中心軌跡
% en_est = zeros(2,1);
% en34_est = zeros(2,1);
% en12_est = zeros(2,1);
% r1_est = sqrt( ( ten1(1,1) - en_est(1,1) )^2 + ( ten1(2,1) - en_est(2,1) )^2 );
% r2_est = sqrt( ( ten2(1,1) - en_est(1,1) )^2 + ( ten2(2,1) - en_est(2,1) )^2 );
% r3_est = sqrt( ( ten3(1,1) - en_est(1,1) )^2 + ( ten3(2,1) - en_est(2,1) )^2 );
% r4_est = sqrt( ( ten4(1,1) - en_est(1,1) )^2 + ( ten4(2,1) - en_est(2,1) )^2 );
% theta1_est = atan( ( ten1(2,1) - en_est(2,1) ) / ( ten1(1,1) - en_est(1,1) ) );
% theta2_est = atan( ( ten2(2,1) - en_est(2,1) ) / ( ten2(1,1) - en_est(1,1) ) );
% theta3_est = atan( ( ten3(2,1) - en_est(2,1) ) / ( ten3(1,1) - en_est(1,1) ) );
% theta4_est = atan( ( ten4(2,1) - en_est(2,1) ) / ( ten4(1,1) - en_est(1,1) ) );
% 
% v_est = [1,1];
% w_est = 1;
% 
% ten1_est(1,1) = ten1(1,1) - r1_est * cos( theta1_est ) + v_est(1,1) * d_time + r1_est * cos( theta1_est + w_est * d_time );
% ten2_est(1,1) = ten2(1,1) - r2_est * cos( theta1_est ) + v_est(1,1) * d_time + r2_est * cos( theta2_est + w_est * d_time );
% ten3_est(1,1) = ten3(1,1) - r3_est * cos( theta1_est ) + v_est(1,1) * d_time + r3_est * cos( theta3_est + w_est * d_time );
% ten4_est(1,1) = ten4(1,1) - r4_est * cos( theta1_est ) + v_est(1,1) * d_time + r4_est * cos( theta4_est + w_est * d_time );
% 
% error_ten1 = zeros(2,1);
% error_ten2 = zeros(2,1);
% error_ten3 = zeros(2,1);
% error_ten4 = zeros(2,1);
% error_theta1 = 0;
% error_theta2 = 0;
% error_theta3 = 0;
% error_theta4 = 0;
% 
% error_v = zeros(2,1);
% error_w = 0;





% % 軌跡の接線の傾き
% if ( ten1(1,1) - ten1_tmp(1,1) ) ~= 0
at_ten1 = ( ten1(2,1) - ten1_tmp(2,1) ) / ( ten1(1,1) - ten1_tmp(1,1) );
% else
% at_ten1 = 0;
% end
% if ( ten2(1,1) - ten2_tmp(1,1) ) ~= 0
at_ten2 = ( ten2(2,1) - ten2_tmp(2,1) ) / ( ten2(1,1) - ten2_tmp(1,1) );
% else
% at_ten1 = 0;
% end
% if ( ten3(1,1) - ten3_tmp(1,1) ) ~= 0
at_ten3 = ( ten3(2,1) - ten3_tmp(2,1) ) / ( ten3(1,1) - ten3_tmp(1,1) );
% else
% at_ten1 = 0;
% end
% if ( ten4(1,1) - ten4_tmp(1,1) ) ~= 0
at_ten4 = ( ten4(2,1) - ten4_tmp(2,1) ) / ( ten4(1,1) - ten4_tmp(1,1) );
% else
% at_ten1 = 0;
% end

% % 軌跡の法線の傾き
% if at_ten1 ~= 0
an_ten1 = -1 / at_ten1;
% else
% an_ten1 = 0;
% end
% if at_ten2 ~= 0
an_ten2 = -1 / at_ten2;
% else
% an_ten2 = 0;
% end
% if at_ten3 ~= 0
an_ten3 = -1 / at_ten3;
% else
% an_ten3 = 0;
% end
% if at_ten4 ~= 0
an_ten4 = -1 / at_ten4;
% else
% an_ten4 = 0;
% end

% 軌跡の法線の切片
% bn_ten1 = ten1_tmp(2,1) - an_ten1 * ten1_tmp(1,1);
% bn_ten2 = ten2_tmp(2,1) - an_ten2 * ten2_tmp(1,1);
% bn_ten3 = ten3_tmp(2,1) - an_ten3 * ten3_tmp(1,1);
% bn_ten4 = ten4_tmp(2,1) - an_ten4 * ten4_tmp(1,1);
% bn_ten1 = ten1(2,1) - an_ten1 * ten1(1,1);
% bn_ten2 = ten2(2,1) - an_ten2 * ten2(1,1);
% bn_ten3 = ten3(2,1) - an_ten3 * ten3(1,1);
% bn_ten4 = ten4(2,1) - an_ten4 * ten4(1,1);
% bn_ten1 = ( ten1_tmp(2,1) + ten1_tmptmp(2,1) ) / 2 - an_ten1 * ( ten1_tmp(1,1) + ten1_tmptmp(1,1)) / 2;
% bn_ten2 = ( ten2_tmp(2,1) + ten2_tmptmp(2,1) ) / 2 - an_ten2 * ( ten2_tmp(1,1) + ten2_tmptmp(1,1)) / 2;
% bn_ten3 = ( ten3_tmp(2,1) + ten3_tmptmp(2,1) ) / 2 - an_ten3 * ( ten3_tmp(1,1) + ten3_tmptmp(1,1)) / 2;
% bn_ten4 = ( ten4_tmp(2,1) + ten4_tmptmp(2,1) ) / 2 - an_ten4 * ( ten4_tmp(1,1) + ten4_tmptmp(1,1)) / 2;
bn_ten1 = ( ten1(2,1) + ten1_tmp(2,1) ) / 2 - an_ten1 * ( ten1(1,1) + ten1_tmp(1,1)) / 2;
bn_ten2 = ( ten2(2,1) + ten2_tmp(2,1) ) / 2 - an_ten2 * ( ten2(1,1) + ten2_tmp(1,1)) / 2;
bn_ten3 = ( ten3(2,1) + ten3_tmp(2,1) ) / 2 - an_ten3 * ( ten3(1,1) + ten3_tmp(1,1)) / 2;
bn_ten4 = ( ten4(2,1) + ten4_tmp(2,1) ) / 2 - an_ten4 * ( ten4(1,1) + ten4_tmp(1,1)) / 2;


norkiseki_est_tmp = norkiseki_est;

% 重心の座標を求める 直線1,2の交点，直線3,4の交点の中点　ここは要改善
if ( an_ten2 - an_ten1 ) ~= 0
est_en12(1,1) = -( bn_ten2 - bn_ten1 ) / ( an_ten2 - an_ten1 );
est_en12(2,1) = ( an_ten2 * bn_ten1 - an_ten1 * bn_ten2 ) / ( an_ten2 - an_ten1 );
end
if ( an_ten4 - an_ten3 ) ~= 0
est_en34(1,1) = -( bn_ten4 - bn_ten3 ) / ( an_ten4 - an_ten3 );
est_en34(2,1) = ( an_ten4 * bn_ten3 - an_ten3 * bn_ten4 ) / ( an_ten4 - an_ten3 );
end
if ( an_ten3 - an_ten1 ) ~= 0
est_en13(1,1) = -( bn_ten3 - bn_ten1 ) / ( an_ten3 - an_ten1 );
est_en13(2,1) = ( an_ten3 * bn_ten1 - an_ten1 * bn_ten3 ) / ( an_ten3 - an_ten1 );
end
if ( an_ten4 - an_ten2 ) ~= 0
est_en24(1,1) = -( bn_ten4 - bn_ten2 ) / ( an_ten4 - an_ten2 );
est_en24(2,1) = ( an_ten4 * bn_ten2 - an_ten2 * bn_ten4 ) / ( an_ten4 - an_ten2 );
end
norkiseki_est(1,1) = ( est_en12(1,1) + est_en34(1,1) + est_en24(1,1) + est_en13(1,1) ) / 4;
norkiseki_est(2,1) = ( est_en12(2,1) + est_en34(2,1) + est_en24(2,1) + est_en13(2,1) ) / 4;

a_norkiseki_est = (norkiseki_est(2,1) - norkiseki_est_tmp(2,1)) / (norkiseki_est(1,1) - norkiseki_est_tmp(1,1))

a_norkiseki_est_gyaku = -1 / a_norkiseki_est;
b_norkiseki_est_gyaku = norkiseki_est(2,1) - a_norkiseki_est_gyaku * norkiseki_est(1,1);



% en(2,1) - ( a_norkiseki_est_gyaku * en(1,1) + b_norkiseki_est_gyaku )

% [ en(2,1), a_norkiseki_est_gyaku * en(1,1) + b_norkiseki_est_gyaku ]

% [ en(1,1), en(2,1),  norkiseki_est(1,1), norkiseki_est(2,1), a_en_est ]

end


