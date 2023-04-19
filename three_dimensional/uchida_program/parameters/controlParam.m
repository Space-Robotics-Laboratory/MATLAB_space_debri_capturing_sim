% CONTROL PARAMETERS
% 2023.4.13 Akiyoshi Uchida
%
% To set parameters for Controller(Class), used in set_Param.m
%

function controlParam = controlParam()

%% Used in Direct Capture
controlParam.margin.capture = 1.1;              % 捕獲した後の手先とターゲットの余裕
controlParam.margin.waitNearTArget = 1.03;      % 捕獲前接近する段階での余裕．1以上にする
controlParam.time.approachWaitPosition = 1;     % 捕獲準備位置まで手先を接近させる最小時間
controlParam.time.capture = .15;                % 接近後，捕獲に要する時間

%% Used in One Hand Contact
% impedance parameters


end