% シミュレーション全体の設定ファイル
function gPram = generalParam()
%% File Setting
gPram.dataSavePath = '/Users/akiyoshi/develop/srl/github/MATLAB_space_debri_capturing_sim/two-dimensional/dat';
gPram.dataSaveName = 'myFile';


%% Simulation Time Setting
gPram.minusTime = 0; % minus value like -1
gPram.endTime   = 10;
gPram.divTime   = .001;

%% Animation Time Setting
gPram.anime_frameRate = 100;    % fps


end