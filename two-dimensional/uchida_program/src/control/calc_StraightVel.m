% 与えられた点を経由して目標位置を達成する直線速度を計算する．
%
% 2023.2 akiyoshi uchida
%
% input : pathWay 4*n [x, y, theta, t]' , currentTime
% output: velocity [vx, vy, w]
%
% pathwayは目標位置，その時の絶対時刻を合わせた行列．列0には初期位置，初期時刻を代入する.
%

function vel = calc_StraightVel(pathWay, currentTime)
    [~, n] = size(pathWay);
    if (currentTime < pathWay(4, 1)) || (currentTime >= pathWay(4, n))
        vel = zeros(3,1);
        return
    end
    dPathWay = pathWay(:, [2:n, 1]) - pathWay;
    index = (pathWay(4, :) <= currentTime) & (pathWay(4, [2:n, 1]) > currentTime);
    vel = dPathWay(1:3, index) ./ dPathWay(4, index);
end