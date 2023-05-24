% 与えられた点を経由して目標位置を達成する直線速度を計算する．
%
% 2023.2 akiyoshi uchida
%
% input : Pathway.pathway(7*n) , currentTime, robo:class,
% param:struct arm:cahr( 'L' or 'R' ) 
% output: velocity 6*1
%
% index : 現在時刻においてpathWayのどの過程にいるか示す 
% gain : 位置変化の大きさとの比.gain=tの場合，t秒で変位を達成する .
%        v = dPosition * gain
% s: n -> n+1間における，無次元化時間
%
% mode1: 直線軌道・ステップ速度．加速度発散の危険性大 gain = 1/dTime (not related to s)
% mode2: 直線軌道・三角速度．gain = 4s (s < .5), -4s + 4 ( s> .5)．max(gain) = 2であり，
%        これは時間による積分を1にするためである．
% mode3: 目標位置からのずれに対して，PT制御によって目標速度を求める．ゲイン調整が重要


function vel = calc_vel_from_pathway(pathway, currentTime, robo, param, arm)

velMode = param.control.velocityMode;
isLeftArm = arm == 'L';
DOF = 6;
timeRow = 7;

if any(isnan(pathway))
    vel = nan(DOF, 1);
    return
end

% 初期時刻以前，最終時刻以降は速度０
if (currentTime < pathway(timeRow, 1)) || (currentTime >= pathway(timeRow, end))
    vel = zeros(DOF,1);
    return
end

shiftedPathway = circshift(pathway, -1, 2);
dPathWay = shiftedPathway - pathway;                                      % 差分計算
index = (pathway(timeRow, :) <= currentTime) & ( shiftedPathway(timeRow, :) > currentTime );  % 時刻をもとに経路のフェーズを判定

% 時刻(pathway(4,:)が順番通りでない場合，エラーを排出する
if sum(index) >= 2  
    error("you have to set pathway in order of time")
end

dP = dPathWay(1:DOF, index);        % 位置ベクトル差分
dTime = dPathWay(timeRow, index);   % 時刻差分       

%%% modeによって手先起動・軌道追従の速さ分布を変更する
switch velMode
    % 直線軌道・一定速度（時刻に対してステップ速度）
    % 最も単純であるが，加速度が発散する危険が非常に高い．
    case 'str_str'
    gain = 1;
    vel = dP ./ dTime * gain;
    return

    % 直線軌道・山形速度（境界で加速度0の折れ曲がった直線）
    % 時間に対して線形に速度が変化する(0->max->0)
    case 'str_tru'
    s = ( currentTime - pathway(timeRow, index) ) / dTime;
    gain = -abs(4*s - 2) + 2;
    vel = dP ./ dTime * gain;
    return

    % フィードバックによって位置制御を行うための速度
    case 'str_fbk'
    findex = [false, index];
    Ck = param.control.kp;
    Cd = param.control.dp;
    if arm == 'L'
        eeIndexL = 6;
        nowPos  = [robo.POS_e_L; robo.SV.QwL           ];   % 6*1
        dnowPos = [robo.VEL_e_L; robo.SV.ww(:, eeIndexL)];  % 6*1
    else
        eeIndexR = 12;
        nowPos  = [robo.POS_e_R; robo.SV.QwR           ];
        dnowPos = [robo.VEL_e_R; robo.SV.ww(:, eeIndexR)];
    end
    deltP = pathway(:, findex) - nowPos;    % 位置差分
    deltD = -dnowPos;                       % 速度差分
    vel = Ck .* deltP + Cd .* deltD;        % PD制御
    return

    % b-spline 曲線軌道
    % 時間に対して線形に速度が変化する(0->max->0)
    case 'spl_tru'
    findex = [false, index];
    armSign = [isLeftArm, ~isLeftArm];
end
error('No such velocity mode')


