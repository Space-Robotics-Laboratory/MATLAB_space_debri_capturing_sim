% ターゲット状態から目標の手先位置を計算する
% vel [left_vel; right_vel]
% left_vel = [vx, vy, vz, wz]'

function vel = calc_DesiredHandVelocity(Target, Robo)
    vel_L = [0.1, 0, 0, 0]';
    vel_R = [0, 0, 0, 0]';
    vel = [vel_L; vel_R];
end