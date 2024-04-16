% シミュレーション全体の設定ファイル
function gParam = generalParam()
%% File Setting
gParam.dataSavePath = './dat';
gParam.dataSaveName = 'myFile';


%% Simulation Time Setting
gParam.minusTime = 0; % minus value like -1
gParam.endTime   = 25;
gParam.divTime   = .001;

% Simulation Break Setting
gParam.breakTimeDuration = 25;

%% Animation Time Setting
gParam.visualizeAnimation = false;
gParam.saveData = true;
gParam.makeAnimation = true;
gParam.makeGraph = true;
gParam.anime_frameRate = 10;    % fps
gParam.snapShot_frameRate = 10; % fps


end