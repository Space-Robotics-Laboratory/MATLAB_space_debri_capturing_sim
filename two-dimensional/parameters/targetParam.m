% Target parameter
%
% 2023.4 akiyoshi uchida
%
function targetParam = targetParam()
%% Size Definition
targetParam.depth = .16;                % 奥行き[m]
targetParam.width = targetParam.depth;  % 幅 [m]
targetParam.height = 0;                 % 高さ 0[m]

%% Mass Definition
targetParam.mCenter2gCenter = [0, 0, 0]';
targetParam.density = 150;                  % 密度[kg/m^2]

%% Initial State
targetParam.init_position = [0, 0.35, 0]';
targetParam.init_orientation = [0, 0, 0]';
targetParam.init_velocity = [0, 0, 0]';
targetParam.init_angular_velocity = [0, 0, 1]';


end