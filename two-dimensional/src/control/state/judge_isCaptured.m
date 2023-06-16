% オブジェクトクロージャの考え方に基づいてターゲット捕獲状況を評価する関数
% 阿部さん修士論文p42 参考
% 内容の理解が甘いので後で見返す
%
% 2023.2 akiyohshi uchida
%
% input  robo: DualArmRobo class, target: TargetSquare class
% output isCapture bool schalar 

function isCapture = judge_isCaptured(robo, target, param)

d = robo.r * 2;
lt = target.width;
LdH = param.robot.endEffector_h;
LdGamma = param.robot.endEffector_gamma;
LdD = param.robot.diameter_endTip;
l = LdH * sin(LdGamma) * 2;

endEffecPos = [robo.POS_es_L, robo.POS_es_R];                 % dim1:[xyz], dim2:roboArmTip 3*4
endEffecDiff = endEffecPos(:, [1, 2, 1, 2]) - endEffecPos(:, [4, 3, 3, 4]); % difference of (L1, R2), (L2, R1), (L1, R1), (L2, R2)
endEffecDistance = vecnorm(endEffecDiff);

underCondition(1) = sqrt(2) * (lt + d) - l;
underCondition(3) = sqrt( (sqrt(2)*(lt + d) - l)^2 + l*l );
underCondition([2, 4]) = underCondition([1, 3]);
overCondition(1) = sqrt( 2*(lt*lt + d*d) - l*l );
overCondition(3) = sqrt(2* (lt*lt + d*d) );
overCondition([2, 4]) = overCondition([1, 3]);
mayCapture = (underCondition < endEffecDistance) & (endEffecDistance < overCondition);
isCapture = all(mayCapture);