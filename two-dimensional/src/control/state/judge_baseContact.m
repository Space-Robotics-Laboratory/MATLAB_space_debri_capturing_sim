% ベースとの接触を判定
% ベクトルは2次元
function isBaseContact = judge_baseContact(robo, target)

basePos = robo.SV.R0(1:2,1);
targetPos = target.SV.R0(1:2,1) + target.m2G(1:2,1);
aboutDistance = vecnorm([target.width, target.depth]) + vecnorm([robo.width, robo.depth]);

isNear = vecnorm(basePos - targetPos) <= aboutDistance;
if ~isNear
    isBaseContact = false;
    return
end

targetTips_temp = calc_SquareTips(targetPos, target.width, target.depth, target.SV.Q0(3));
targetTips_temp = repmat(targetTips_temp, [1,1,4]);
targetTips = permute(targetTips_temp, [1,3,2]);
baseTips = calc_SquareTips(basePos, robo.width, robo.depth, robo.SV.Q0(3));

directions = baseTips - circshift(baseTips, 1, 2);
norms_temp = directions ./ [robo.width, robo.depth, robo.width, robo.depth];
norms = repmat(norms_temp, [1,1,4]);

baseTip2targetTip = targetTips - baseTips;
distances = dot(norms, baseTip2targetTip, 1);

isPenetSide = all(distances<=0, 2);
isBaseContact = any(isPenetSide, "all");