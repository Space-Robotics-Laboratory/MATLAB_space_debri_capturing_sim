% シミュレーション全体の設定ファイル
function gPram = generalParam()
%% File Setting
gPram.dataSavePath = '/Users/akiyoshi/develop/srl/github/MATLAB_space_debri_capturing_sim/two-dimensional/uchida_program/dat';
gPram.dataSaveName = 'myFile';


%% Simulation Time Setting
gPram.minusTime = 0;
gPram.endTime   = 1;
gPram.divTime   = .001;

%% Animation Time Setting
gPram.anime_divTime = .01;


end