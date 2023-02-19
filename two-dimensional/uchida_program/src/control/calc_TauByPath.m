% ターゲット接触状況等から目標関節トルクを計算する関数．
% 具体的な制御はこの関数内でおこなっている．
% 
% 2023.2 uchida akiyoshi
%
% 先首関節はバネダンパ系の受動関節であるので，関節速度に制約が加わる
% input : DualArmRobo class, pathway 4*n*2, 
%         RoboExtWrench 6*3(base, leftEdgem, rightEdge) 
% output: JointTorque 8*1, but only used 6*1
%
% 

function jointTau = calc_TauByPath(dualArmRobo, pathway, RoboExtEst, mode, time)
global d_time

robo = dualArmRobo;

desiredVel(1:3, 1) = calc_Vel(pathway(:, :, 1), time, mode);  % left
desiredVel(4:6, 1) = calc_Vel(pathway(:, :, 2), time, mode);  % right

jointTau = calc_TauByVel(robo, desiredVel, RoboExtEst, [false, false]);

