% Target parameter
%
% 2023.4 akiyoshi uchida
%
function targetParam = targetParam()
%% Size Definition
targetParam.depth = .15;                % 奥行き[m]
targetParam.width = targetParam.depth;  % 幅 [m]
targetParam.height = 0;                 % 高さ 0[m]

%% Mass Definition
targetParam.comOffset = [0, 0, 0]'; % 幾何中心から質量重心までの位置ベクトル
% targetParam.density = 50;                  % 密度[kg/m^2]
targetParam.mass = 2.34844;
targetParam.inertia = [1e9,   0, 0  ;
                         0, 1e9, 0  ;
                         0,   0, 0.00932724];

%% Initial State
targetParam.initial_position = [0, .35, 0]';
targetParam.initial_orientation = [0, 0, 0]';
targetParam.initial_velocity = [0, 0, 0]';
targetParam.initial_angular_velocity = [0, 0, 1]';


end