% Target parameter
%
% 2023.4 akiyoshi uchida
%

% not integarated yet; changes in this file not affect simulation!! edit
% set_Param.m
function targetParam = targetParam()
%% Size Definition
targetParam.depth = .16;                % 奥行き[m]
targetParam.width = targetParam.width;  % 幅 [m]
targetParam.height = 0;                 % 高さ [m]

%% Mass Definition
targetParam.mCenter2gCenter = [0, 0, 0]';

%% Movement Definition


end