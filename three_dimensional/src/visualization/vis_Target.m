% ターゲット描画関数
% 
% 2023.3 akiyoshi uchida
% 

function vis_Target(targR0, targQ0, param)
targetMCenter2GCenter = param.TargetMCenter2GCenter;
targetWidth = param.TargetWidth;
targetDepth = param.TargetDepth;
targetHeight = param.TargetHeight;
targetORI = rpy2dc(targQ0)';

targetCenterPos = targR0+targetMCenter2GCenter;
[targetVertices, targetFaces] = calc_cubic(targetCenterPos, targetWidth, targetDepth, targetHeight, targetORI);
patch('Vertices',targetVertices,'Faces',targetFaces, 'FaceColor', param.general.targetColor)
end
