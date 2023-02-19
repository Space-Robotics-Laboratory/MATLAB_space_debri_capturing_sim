% 与えられた点を経由して目標位置を達成する直線速度を計算する．
%
% 2023.2 akiyoshi uchida
%
% input : pathWay 4*n [x, y, theta, t]' , currentTime
% output: velocity [vx, vy, w]
%
% pathwayは目標位置，その時の絶対時刻を合わせた行列．列0には初期位置，初期時刻を代入する.
% pathWay(1:3, n+1) -> pathWay(1:3, n) の移動を，pathWay(4, n+1) -> pathWay(4,
% n)秒で達成する.これをn-1回繰り返すことにより，経由地点を経た軌道を達成する．
% 
% index : 現在時刻においてpathWayのどの過程にいるか示す 
% gain : 位置変化の大きさとの比.gain=tの場合，t秒で変位を達成する .
%        v = dPosition * gain
% s: n -> n+1間における，無次元化時間
%
% mode1: 直線軌道・ステップ速度．加速度発散の危険性大 gain = 1/dTime (not related to s)
% mode2: 直線軌道・三角速度．gain = 4s (s < .5), -4s + 4 ( s> .5)．max(gain) = 2であり，
%        これは時間による積分を1にするためである．


function vel = calc_Vel(pathWay, currentTime, mode)

[~, n] = size(pathWay);
% 初期時刻以前，最終時刻以降は速度０
if (currentTime < pathWay(4, 1)) || (currentTime >= pathWay(4, n))
    vel = zeros(3,1);
    return
end
switch mode
    % 直線軌道・一定速度（時刻に対してステップ速度）
    case 1
    dPathWay = pathWay(:, [2:n, 1]) - pathWay;
    index = (pathWay(4, :) <= currentTime) & (pathWay(4, [2:n, 1]) > currentTime);
    dP = dPathWay(1:3, index);
    dTime = dPathWay(4, index);
    vel = dP ./ dTime;

    % 直線軌道・山形速度（境界で加速度0の折れ曲がった直線）
    case 2
    dPathWay = pathWay(:, [2:n, 1]) - pathWay;
    index = (pathWay(4, :) <= currentTime) & (pathWay(4, [2:n, 1]) > currentTime);
    dP = dPathWay(1:3, index);
    dTime = dPathWay(4, index);
    s = ( currentTime - pathWay(4, index) ) / dTime;
    gain = -abs(4*s - 2) + 2;
    vel = dP * gain ./ dTime;

end
