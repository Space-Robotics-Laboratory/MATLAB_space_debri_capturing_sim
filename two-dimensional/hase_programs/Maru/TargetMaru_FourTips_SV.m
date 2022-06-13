function SV = TargetMaru_FourTips_SV( ~ ) % ~ は void と一緒   % 〇型ターゲット

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%% 変数の初期化 %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

SV.q = zeros(0,1);   % 関節角
SV.qd  = zeros(0,1); % 関節角速度
SV.qdd = zeros(0,1); % 関節角加速度

SV.v0  = [ 0 0 0 ]'; % ベースの並進速度
SV.w0  = [ 0 0 0 ]'; % ベースの角速度
SV.vd0 = [ 0 0 0 ]'; % ベースの加速度
SV.wd0 = [ 0 0 0 ]'; % ベースの角加速度

SV.vv = zeros(3,0); % 各リンクの重心速度
SV.ww = zeros(3,0); % 各リンクの重心角速度
SV.vd = zeros(3,0); % 各リンクの重心加速度
SV.wd = zeros(3,0); % 各リンクの重心角加速度

SV.R0 = [ 0 0 0 ]'; % ベースの位置
SV.Q0 = [ 0 0 0 ]'; % ベースの角度 (オイラー角表現)
SV.A0 = eye(3); % ベースの角度 (方向余弦行列表現)  eyeは単位行列

SV.Fe = zeros(3,0); % 端点へかかる外力
SV.Te = zeros(3,0); % 端点へかかるトルク
SV.F0 = [ 0 0 0 ]'; % ベース重心へかかる外力
SV.T0 = [ 0 0 0 ]'; % ベース重心へかかるトルク

SV.tau =zeros(0,1); % 各関節へかかる軸トルク
