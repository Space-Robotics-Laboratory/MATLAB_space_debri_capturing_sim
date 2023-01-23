% 2023.1 uchida akiyoshi
%
% ターゲット状態から目標の手先位置を計算する
% vel [left_vel; right_vel]
% left_vel = [vx, vy, wz]'
% 関節自由度が6なので，desiredVelocityも6次元

function vel = calc_DesiredHandVelocity(phase, Target, Robo)
    if phase == 1
        vel_L = [0.1/5, -0.1/5, 0]';
    else
        vel_L = [0.1/5, 0.1/5, 0]';
    end
%     vel_L = [0, 0, 0]';
    vel_R = [0, 0, 0]';
    vel = [vel_L; vel_R];
end