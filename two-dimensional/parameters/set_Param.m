function param = set_Param()

% シミュレーション条件 & パス設定
param.general = generalParam();

% ロボットパラメータ
param.robot = robotParam();

% ターゲットパラメータ
param.target = targetParam();

% コントロールパラメータ
param.control = controlParam();

% 接触係数
param.contact = contactParam();
end