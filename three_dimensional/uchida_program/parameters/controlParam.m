% CONTROL PARAMETERS
% 2023.4.13 Akiyoshi Uchida
%
% To set parameters for Controller(Class), used in set_Param.m
%

function controlParam = controlParam()
%% Controller Setting
controlParam.controlMode = 'TEST1';          % コントローラーモード
controlParam.velocityMode = 'str_tru';          % pathwayから速度を計算する方法
controlParam.impedanceMode = 'addmitance';      % インピーダンス制御モード

%% FeedBack Gain Useed in Pathway Following(in vel mode 'str_fbk')
controlParam.kp = [5, 5, .0]';             % フィードバックによる位置制御：比例係数
controlParam.dp = [0, 0, .0]';                % フィードバックによる位置制御：減衰係数

%% Used in Direct Capture
controlParam.captureDistantMargin = 1.05;        % 捕獲した後の手先とターゲットの余裕
controlParam.approachDistantMargin = 1.03;      % 捕獲前接近する段階での余裕．1以上にする
controlParam.approachTime = 1;                  % 捕獲準備位置まで手先を接近させる最小時間
controlParam.captureTime = .15;                 % 接近後，捕獲に要する時間

%% Used in One Hand Contact
% timing parameters
controlParam.impedanceDuration = .2;            % 接触後，インピーダンス制御を持続させる時間
controlParam.swichingDelay2Direct = .35;        % ターゲット角速度が減衰したのち，直接捕獲モードに切り替える待ち時間

% border value
controlParam.switchingTargetAngVel = 1;         % 境界値となるターゲット角速度

% impedance parameters
controlParam.mi = [1, 1, 1]'*.5;                   % アドミタンス制御仮想質量   [.5, .5, .5]';
controlParam.di = [1, 1, 1]'*10;                   % アドミタンス制御ダンパ特性  [10, 10, 10]';
controlParam.ki = [.0, .0, .0]';                   % アドミタンス制御バネ特性   [.5, .5, .5]';

% contact parameters
controlParam.contactPositionratio = .8;             % 接触位置がターゲット辺のどの割合にあるかを表す.0で中心1で頂点
controlParam.endEffecAngleDeviation = deg2rad(10);  % 接触時点のエンドエフェクターの角度
controlParam.contactTargetAngLim = deg2rad(20);     % 接触時のターゲット角度をターゲットの速度方向によって定める時の上限.大きいと手先の姿勢が達成しづらくなる


end