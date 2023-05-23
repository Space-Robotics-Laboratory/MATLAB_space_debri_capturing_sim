% ターゲット描画関数
% 
% 2023.3 akiyoshi uchida
% 

function vis_Target(targR0, targQ0, param)
ori_mat = rpy2dc(targQ0)';
targetMCenter2GCenter = ori_mat * param.target.comOffset;
targetWidth = param.target.width;
targetDepth = param.target.depth;

targetCenterPos = targR0(1:2);
targetOriZ      = targQ0(3);                                                                                    % z軸周りの角度ラジアン
targetTips = calc_SquareTips(targetCenterPos + targetMCenter2GCenter(1:2), targetWidth, targetDepth, targetOriZ);    % ベース頂点計算
PolyTarget = polyshape(targetTips(1,:), targetTips(2,:));                                                       % 四角描画
plot(PolyTarget)
end
