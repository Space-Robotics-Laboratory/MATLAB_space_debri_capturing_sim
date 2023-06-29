% シミュレーション全体の設定ファイル
function gParam = generalParam()
%% File Setting
gParam.dataSavePath = '/Users/akiyoshi/develop/srl/github/MATLAB_space_debri_capturing_sim/two-dimensional/dat';
gParam.dataSaveName = 'myFile';


%% Simulation Time Setting
gParam.minusTime = 0; % minus value like -1
gParam.endTime   = 2;
gParam.divTime   = .00001;

% Simulation Break Setting
gParam.breakTimeDuration = 1;

%% Animation Time Setting
gParam.anime_frameRate = 100;    % fps


end