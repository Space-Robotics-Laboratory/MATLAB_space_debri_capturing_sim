% 一般化ヤコビ行列を用いて目標手先速度から関節トルクを計算する
% 
% 2023.4 uchida akiyoshi
%
% 阿部さん修論参照．変数名もできるだけ論文を参考しにているので，確認するときは論文と比較しながら
% 先首関節はバネダンパ系の受動関節であるので，関節速度に制約が加わる
% input : DualArmRobo class, AllPartVelocity 9*1(base, leftArm, roghtArm), 
%         RoboExtWrench 6*3(base, leftEdgem, rightEdge) 
%         inBaseFrame 1*2 [left, right] is expressed in base fmare, or in
%         inertia frame
%         achievedVel 1*18 bool [Bvx, Bvy, Bwz, Lvx, Lvy, Lwz, Rvx, Rvy, Rwz]
%         desired velocity. If exceed 6, results can be apploxmate 
% output: JointTorque 8*1, but only used 6*1
%

function JointTau = calc_tau_from_ee_base_vel(DualArmRobo, allPartVel, RoboFTsensor, velConsidered)
global d_time
    LP = DualArmRobo.LP;
    SV = DualArmRobo.SV;
    num_eL = DualArmRobo.num_eL;
    num_eR = DualArmRobo.num_eR;
    if ~all(size(allPartVel)==[9,1])
        error('input velocity dimention must be [9,1]')
    end

    % 双腕ロボ一般化慣性行列，一般化速度非線形項計算
    [H_asuta, C_asuta] = calc_asuta_2arm(LP, SV, num_eL, num_eR);

    % 一般化ヤコビアン，ベース手先ヤコビアン，慣性行列計算
    [Jgg, Jb, ~, HH] = calc_ggj_2arm(LP, SV, num_eL, num_eR);
    Hb = HH(1:6, 1:6);


    % 現時刻の運動量計算
    % 現時刻でのロボット速度を使用している．ロボット速度が既知かどうかに注意
    PL = calc_momentum(LP, SV);
%     PL = zeros(6,1);

    % 目標自由度削減 18*8 -> 6*8
    index_ = [1, 2, 6, 7, 8, 12, 13, 14, 18];
    index = index_(velConsidered);
    Jgg_s = Jgg(index, :);          % 目標速度を選択
    Jgg_s(:, [4,8]) = 0;            % 受動関節の関節速度0を仮定
    Jb_s = Jb([1,2,6, 7,8,12], :);  % ベース速度に対する手先ヤコビアン

    % 手先にかかる外力の推定値を代入 
    F_c = reshape(RoboFTsensor, [12,1]);

    % inBodyFrameの速度の場合，ヤコビアンはJm
    % inInertiaFrameの速度の場合，タコビアンはJg
    baseVel_by_moment = (Hb\PL);                                    % 運動量変化による見かけの速度
    eeVel_by_moment = Jb_s * baseVel_by_moment;                     % 運動量変化による見かけの速度
    desiredVel = allPartVel - [baseVel_by_moment([1,2,6]); eeVel_by_moment];

    % 目標関節角速度計算
    qd_des = pinv(Jgg_s) * desiredVel(velConsidered');              % 関節角速度．運動量変化について考える.
                                                                    % 次元が6を超える場合，最小二乗近似となる
    qdd_des = (qd_des - SV.qd) / d_time;                            % 関節角加速度
    JointTau = H_asuta * qdd_des + C_asuta - Jgg(1:12,:)' * F_c;    % 8関節トルク（set wrist active joint）
    %JointTau = Ck .* (qd_des - SV.qd) + Cd .* (qdd_des - SV.qdd)
    
end