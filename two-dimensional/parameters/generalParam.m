% シミュレーション全体の設定ファイル
function gParam = generalParam()
%% File Setting
gParam.dataSavePath = './dat';
gParam.dataSaveName = 'myFile';


%% Simulation Time Setting
gParam.minusTime = 0; % minus value like -1
gParam.endTime   = .01;
gParam.divTime   = .001;

% Simulation Break Setting
gParam.breakTimeDuration = 1;

%% Animation Time Setting
gParam.anime_frameRate = 100;    % fps
gParam.snapShot_frameRate = 5000; % fps


end