% CONTROL PARAMETERS
% 2023.4.13 Akiyoshi Uchida
%
% To set parameters for Controller(Class), used in set_Param.m
%

function controlParam = controlParam()
%% Controller Setting
controlParam.controlMode = 'DIRECT_IMP';          % コントローラーモード
controlParam.velocityMode = 'str_poly3';          % pathwayから速度を計算する方法
controlParam.impedanceMode = 'addmitance';      % インピーダンス制御モード

%% FeedBack Gain Useed in Pathway Following(in vel mode 'str_fbk')
controlParam.kp = [5, 5, .0]';             % フィードバックによる位置制御：比例係数
controlParam.dp = [0, 0, .0]';                % フィードバックによる位置制御：減衰係数

%% Used in Direct Capture
controlParam.captureDistantMargin = 1.01;        % 捕獲した後の手先とターゲットの余裕
controlParam.approachDistantMargin = 1.03;      % 捕獲前接近する段階での余裕．1以上にする
controlParam.approachTime = 1;                  % 捕獲準備位置まで手先を接近させる最小時間
controlParam.captureTime = .3;                 % 接近後，捕獲に要する時間

%% Used in One Hand Contact
% timing parameters
controlParam.impedanceDuration = .35;            % 接触後，インピーダンス制御を持続させる時間
controlParam.swichingDelay2Direct = .5;        % ターゲット角速度が減衰したのち，直接捕獲モードに切り替える待ち時間

% border value
controlParam.switchingTargetAngVel = .5;                     % 複数回接触の境界値となるターゲット角速度
controlParam.nonContactArm2targetMinDistanceRatio =1.2;      % 非接触アームがターゲットに接触しないための制御を開始する境界値

% impedance parameters
controlParam.contactTipSelection = 2; % 1: contact with farther tip, 2: contact with closer tip
controlParam.mi = [1, 1, 1]'*0.8;                   % アドミタンス制御仮想質量   [.5, .5, .5]';
controlParam.di = [1, 1, 1]'*15;                   % アドミタンス制御ダンパ特性  [10, 10, 10]';
controlParam.ki = [1, 1, 1]'*10;                   % アドミタンス制御バネ特性   [.5, .5, .5]';

% contact parameters
controlParam.contactPositionRatio = .75;             % 接触位置がターゲット辺のどの割合にあるかを表す.0で中心1で頂点
controlParam.endEffecAngleDeviation = deg2rad(5);  % 接触時点のエンドエフェクターの角度
controlParam.contactTargetAngLim = deg2rad(0);     % 接触時のターゲット角度をターゲットの速度方向によって定める時の上限.大きいと手先の姿勢が達成しづらくなる

% sigurality avoidance
controlParam.minMiddleJointsAngle = pi/6;

%% Capturing Evaluation
controlParam.stoppingTargetAngVel = 0.5;                   % ターゲットの回転が停止したとみなせる，ロボットへの相対角速度
controlParam.stoppingTargetVel = 0.1;                      % ターゲットの回転が停止したとみなせる，ロボットへの相対速度 
controlParam.reachable = 0.6;                               % 捕獲不可能と判断する境界値
controlParam.maxContactForce = 5;   % 判定に使用する最大の接触力　制御には影響せず結果の判定に使用


end