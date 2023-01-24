% 2023.1 uchida akiyoshi
%
% ターゲット状態から目標の手先位置を計算する
% vel [left_vel; right_vel]
% left_vel = [vx, vy, wz]'
% 関節自由度が6なので，desiredVelocityも6次元

function vel = calc_DesiredHandVelocity(time)
    mode = 3;
    switch mode
        case 1
            r = 0.225016;
            th = time;
            vel_L = [r*sin(th), -r*cos(th), 1]';
            vel_R = [-0.1, -0.1, 1]';
        case 2
            vel_L = [0.1, -0.1, 1]';
            vel_R = [-0.1, 0.05, 1]';
        case 3
            vel_L = [0, 0, 1]';
            vel_R = [0, 0, 0]';
    end
    vel = [vel_L; vel_R];
end