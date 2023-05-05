% ロボット設定用パラメータ
% [kg], [m], [s]

function robotParam = robotParam()
%% Shape of Robot
% Define Base
robotParam.baseWidth    = 0.15757*2;
robotParam.baseDepth    = 0.16;
robotParam.baseHeight   = 0.16;

% Define Links 
robotParam.length_links = [0.1, 0.20, 0.20, 0.1, 0.08]; % link A, B, C, D, E. 左右対称

% Define End-Effector
robotParam.length_endEffector   = 0.05;
robotParam.radius_endEffector   = 0.035; % 手先全体の半径であり，球の半径でないことに注意
robotParam.diameter_endTip      = 0.02;

%% Initial State of Robot
robotParam.initial_position = [0, 0, 0]';
robotParam.initial_orientation = [0, 0, 0]';
robotParam.initial_velocity = [0, 0, 0]';
robotParam.initial_angularVelocity = [0, 0, 0]';
robotParam.initial_jointsAngle(1:6,  1) = [0 pi/3 -pi*4/9 0 -pi*7/18 0]';
robotParam.initial_jointsAngle(7:12, 1) =-[0 pi/3 -pi*4/9 0 -pi*7/18 0]';


%% Phisics of Robot
% Define Base
robotParam.mass_base    = 10;
robotParam.inertia_base = [0.5,   0, 0  ;
                             0, 0.5, 0  ;
                             0,   0, 0.5];

% Define Links as "having a motor at the end of an aluminum round bar"
robotParam.mass_motor = 0.1;
robotParam.inertia_motor = [3e-3,   0,    0;
                            0,   3e-3,    0;
                            0,      0, 1e-3];
robotParam.rho_link_frame = 2700;                                   % 丸棒密度:alminum
robotParam.radius_links = [0.015, 0.015, 0.015, 0.015 ,0.015];      % 丸棒半径

% Define End-Effector
robotParam.mass_endEffector = 0.5;
robotParam.inertia_endEffector = [1e-3,   0,    0;
                                     0,   1e-3,    0;
                                     0,      0, 1e-3];
robotParam.comOffset_endEffector = [0, 0, 0]';          % エンドエフェクター重心の，幾何中心からのずれ


end