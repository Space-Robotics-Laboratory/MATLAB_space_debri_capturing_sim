% シミュレーション全体の設定ファイル
function gParam = generalParam()
%% File Setting
gParam.dataSavePath = './dat';
gParam.dataSaveName = 'myFile';


%% Simulation Time Setting
gParam.minusTime = 0; % minus value like -1
gParam.endTime   = 200;
gParam.divTime   = .001;

% Simulation Break Setting
gParam.breakTimeDuration = 8;

%% Animation Time Setting
gParam.visualizeAnimation = true;
gParam.saveData = true;
gParam.makeAnimation = true;
gParam.makeGraph = true;
gParam.anime_frameRate = 10;    % fps
gParam.snapShot_frameRate = 1; % fps


end