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

% ターゲット頂点のベースのめり込み判定
targetTips_ground = calc_SquareTips(targetPos, target.width, target.depth, target.SV.Q0(3));
targetTips_ground = repmat(targetTips_ground, [1,1,4]);
targetTips_roof = permute(targetTips_ground, [1,3,2]);
baseTips = calc_SquareTips(basePos, robo.width, robo.depth, robo.SV.Q0(3));

baseDirections = baseTips - circshift(baseTips, 1, 2);
baseNorms_temp = baseDirections ./ [robo.width, robo.depth, robo.width, robo.depth];
baseNorms = repmat(baseNorms_temp, [1,1,4]);


baseTip2targetTip = targetTips_roof - baseTips;
distances_base2target = dot(baseNorms, baseTip2targetTip, 1);

isPenetBaseSide = all(distances_base2target<=0, 2);

% ベース頂点のターゲットへのめり込み判定
targetDirections = targetTips_ground - circshift(targetTips_ground, 1, 2);
targetNorms = targetDirections ./ [target.width, target.depth, target.width, target.depth];
targetTip2baseTip = -permute(baseTip2targetTip, [1, 3, 2]);
distances_target2base = dot(targetNorms, targetTip2baseTip, 1);

isPenetTargetSide = all(distances_target2base<=0, 2);


isBaseContact = any([isPenetBaseSide, isPenetTargetSide], "all");