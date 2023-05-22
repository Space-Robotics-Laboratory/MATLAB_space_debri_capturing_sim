% ロボット設定用パラメータ
% [kg], [m], [s]

function robotParam = robotParam()
%% Shape of Robot
% Define Base
robotParam.baseWidth    = 0.15757*2;
robotParam.baseDepth    = 0.16;
robotParam.baseHeight   = 0;

% Define Links 
robotParam.length_links = [0.25, 0.175, 0.091]; % link A, B, C. 左右対称

% Define End-Effector
robotParam.endEffector_h        = 0.081393154503312;    % 手先関節から手先球までの距離
robotParam.endEffector_gamma    = 0.717235030908703;    % 上記を結んだ直線の角度
robotParam.diameter_endTip      = 0.018;

%% Initial State of Robot
robotParam.initial_position = [0, 0, 0]';
robotParam.initial_orientation = [0, 0, 0]';
robotParam.initial_velocity = [0, 0, 0]';
robotParam.initial_angularVelocity = [0, 0, 0]';
robotParam.initial_jointsAngle(1:3,  1) = [0 pi/3 -pi*4/9 -pi*7/18]';
robotParam.initial_jointsAngle(5:7, 1)  =-[0 pi/3 -pi*4/9 -pi*7/18]';


%% Phisics of Robot
% Define Base
robotParam.mass_base    = 8;
robotParam.inertia_base = [1e9,   0, 0  ;
                             0, 1e9, 0  ;
                             0,   0, 0.5];

% Define Link
robotParam.mass_links = [1.09, 0.98, 0.32];                                     % link A, B, C. 左右対称
robotParam.inertia_links = [1e9, 0, 0,          1e9, 0, 0,          1e9, 0, 0;  % link A, B, C. 左右対称
                            0, 1e9, 0,          0, 1e9, 0,          0, 1e9, 0;
                            0, 0, 0.00371,      0, 0, 0.00149,      0,   0, 0.000752];

% Define End-Effector
robotParam.mass_endEffector = 0.21;
robotParam.inertia_endEffector = [1e9,      0,          0;
                                     0,   1e9,          0;
                                     0,      0,  0.000254];
robotParam.comOffset_endEffector = [0, 0, 0]';          % エンドエフェクター重心の，幾何中心からのずれ


end