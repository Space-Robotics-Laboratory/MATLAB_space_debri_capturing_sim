% ロボット設定用パラメータ
% 左手リンクが[1,2,3,4], 右手リンクが[5,6,7,8]
% [1 2 3], [5, 6, 7] リンクが，[linkA,linkB,linkC]と表される
% [kg], [m], [s]

function robotParam = robotParam()
%% Shape of Robot
% Define Base
robotParam.baseWidth    = 0.32;
robotParam.baseDepth    = 0.16;
robotParam.baseHeight   = 0;

% Define Links 
robotParam.length_links = [0.25, 0.18, 0.13655]; % link A, B, C. 左右対称 [0.27, 0.2, 0.1];%

% Define End-Effector
robotParam.endEffector_h        = 0.057505101269;    % 手先関節から手先球までの距離
robotParam.endEffector_gamma    = 0.763179598071;    % 上記を結んだ直線の角度
robotParam.diameter_endTip      = 0.03;

%% Initial State of Robot
robotParam.initial_position = [0, 0, 0]';
robotParam.initial_orientation = [0, 0, 0]';
robotParam.initial_velocity = [0, 0, 0]';
robotParam.initial_angularVelocity = [0, 0, 0]';
robotParam.initial_jointsAngle(1:4,  1) =   [ pi/3 -pi*4/9 -pi*7/18 0 ]';
robotParam.initial_jointsAngle(5:8, 1)  = - [ pi/3 -pi*4/9 -pi*7/18 0 ]';


%% Phisics of Robot
% Define Base
robotParam.mass_base    = 8.3084;
robotParam.comOffset_base = [0; 0.076078375; 0];
robotParam.inertia_base = [1e9,   0, 0  ;
                             0, 1e9, 0  ;
                             0,   0, 0.13526810];

% Define Link
robotParam.mass_links = [0.6328, 0.6470, 0.14170];                                     % link A, B, C. 左右対称
robotParam.inertia_links = [1e9, 0, 0,          1e9, 0, 0,          1e9, 0, 0;  % link A, B, C. 左右対称
                            0, 1e9, 0,          0, 1e9, 0,          0, 1e9, 0;
                            0, 0, 0.00255150,      0, 0, 0.00118507,      0,   0, 0.00017569304];%0.002,        0, 0, 0.001,        0, 0, 0.0005];%
robotParam.comOffset_links = [0, 0, 0;
                              0.09598, 0.07243, -0.005145;
                              0, 0, 0];    % link A(vec3), B(vec3), C(vec3). 左右対称リンク幾何中心から重心までの位置ベクトル

% Define End-Effector
robotParam.mass_endEffector = 0.065;
robotParam.inertia_endEffector = [1e9,      0,          0;
                                     0,   1e9,          0;
                                     0,      0,  0.00011548236];%0.000254];
robotParam.comOffset_endEffector = [0, 0, 0]';          % エンドエフェクター重心の，幾何中心からのずれ


% Wrist Spring Setting
robotParam.compliantWrist = false;
if(robotParam.compliantWrist)
    robotParam.wristDamp  = 0.1;%0.4;       % 手首関節減衰係数
    robotParam.wristElast = .2;%0.8;       % 手首関節弾性係数
end

% Motor Limitation
robotParam.motorTorque_max = repmat(10, [8,1]); % 受動関節に関しては関係ない
robotParam.jointAngle_max = [];     % still unused
robotParam.jointAngle_min = [];     % still unused


end