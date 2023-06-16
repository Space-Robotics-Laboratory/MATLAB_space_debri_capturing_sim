% オブジェクトクロージャの考え方に基づいてターゲット捕獲状況を評価する関数
% 阿部さん修士論文p42 参考
% 余裕を持たせた方がいいかも？
%
% 2023.2 akiyohshi uchida
%
% input  robo: DualArmRobo class, target: TargetSquare class
% output isCapture bool schalar 

function isCapture = judge_isCaptured(robo, target, param)
d = robo.r * 2;
lt = target.width;
th_t = mod(target.SV.Q0(3), pi/2);
th_L = robo.SV.QeL(3) + pi/2;
th_R = robo.SV.QeR(3) - pi/2;
LdH = param.robot.endEffector_h;
LdGamma = param.robot.endEffector_gamma;
l = LdH * sin(LdGamma) * 2;

endEffector_L = calc_armTipsPos(robo.POS_e_L, robo.ORI_e_L, param);
endEffector_R = calc_armTipsPos(robo.POS_e_R, robo.ORI_e_R, param);

T = target.SV.R0(1:2, 1);

th_t_L_ = mod((th_t - th_L), pi/2);
th_t_R_ = mod((th_t - th_R), pi/2);

% Object C-Space Side length limit
U1 = dot((endEffector_R(1:2, 2) - endEffector_L(1:2, 2)), [cos(th_t); sin(th_t)]) - (lt + d);
U2 = dot((endEffector_R(1:2, 1) - endEffector_L(1:2, 1)), [sin(th_t);-cos(th_t)]) - (lt + d);

if U1 < 0 || U2 < 0 % no room for target
    isCapture = false;
    return
end

% Object C-Space Side length
S1 = l * sin(th_t_L_);
S2 = l * cos(th_t_L_);
S3 = l * sin(th_t_R_);
S4 = l * cos(th_t_R_);

if (U2 > min(S2, S4)) || (U1 > min(S1, S3))
    isCapture = false;
    return
end

if U1 > (min(S1, S3) - d) && U2 > (min(S2, S4) - d)
    d1 = vecnorm([U1-S3+d, U2-S2+d]) - d;
    d2 = vecnorm([U1-S1+d, U2-S4+d]) - d;
    if max(d1, d2) > 0
        isCapture = false;
        return
    end
end

% now robot arm satisfies object closure
% next, see if target is in it
TS1 = dot((T - endEffector_L(1:2, 1)), [sin(th_t);-cos(th_t)]) - (lt + d)/2;
TS2 = dot((T - endEffector_L(1:2, 2)), [cos(th_t); sin(th_t)]) - (lt + d)/2;

isCapture = (TS1 > 0) && (TS1 < U2) && (TS2 > 0) && ( TS2 < U1);




