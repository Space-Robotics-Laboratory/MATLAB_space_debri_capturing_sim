% 一般化ヤコビ行列を用いて目標手先速度から関節トルクを計算する
% 
% 2023.2 uchida akiyoshi
%
% 阿部さん修論参照．変数名もできるだけ論文を参考しにているので，確認するときは論文と比較しながら
% 先首関節はバネダンパ系の受動関節であるので，関節速度に制約が加わる
% input : DualArmRobo class, DesierdHandVelocity 6*1, 
%         RoboExtWrench 6*3(base, leftEdgem, rightEdge) 
%         inBaseFrame 1*2 [left, right] is expressed in base fmare, or in
%         inertia frame
% output: JointTorque 8*1, but only used 6*1
%

function JointTau = calc_tau_from_ee_vel(DualArmRobo, endEffecVel, RoboFTsensor, inBodyFrame)
global d_time
    % ゲイン設定
    Ck = [5, 5, 5, 5, 5, 5, 5, 5]';
    Cd = [5, 5, 5, 5, 5, 5, 5, 5]';

    LP = DualArmRobo.LP;
    SV = DualArmRobo.SV;
    num_eL = DualArmRobo.num_eL;
    num_eR = DualArmRobo.num_eR;
    if ~all(size(endEffecVel)==[6,1])
        error('desired velocity dimention must be [6,1]')
    end

    % 双腕ロボ一般化慣性行列，一般化速度非線形項計算
    [H_asuta, C_asuta] = calc_asuta_2arm(LP, SV, num_eL, num_eR);

    % 一般化ヤコビアン，ベース手先ヤコビアン，慣性行列計算
    [Jg, Jb, Jm, HH] = calc_gj_2arm(LP, SV, 1, 2);
    Hb = HH(1:6, 1:6);


    % 現時刻の運動量計算
    % 現時刻でのロボット速度を使用している．ロボット速度が既知かどうかに注意
    PL = calc_momentum(LP, SV);
%     PL = zeros(6,1);

    % 目標自由度削減 18*8 -> 6*8
    Jg_s = Jg([1,2,6, 7,8,12], :);   
    Jg_s(:, [4,8]) = 0;                         % jg_sは目標速度の次元を削減し，受動関節角速度0を仮定したヤコビアン
    Jb_s = Jb([1,2,6, 7,8,12], :);              % ベース速度に対する手先ヤコビアン
    Jm_s = Jm([1,2,6, 7,8,12], :);
    Jm_s(:, [4,8]) = 0;

    % 手先にかかる外力の推定値を代入 
    F_c = reshape(RoboFTsensor, [12,1]);

    % inBodyFrameの速度の場合，ヤコビアンはJm
    % inInertiaFrameの速度の場合，タコビアンはJg
    % inBodyFrameの場合，ボディ系で速度を定義するため，運動量による速度は考慮しない．よってVelbyPl = 0
    index = repmat(inBodyFrame, [3,1]);
    J = Jg_s;
    J(index, :) = Jm_s(index, :);
    VelbyPL = zeros(6, 1);
    temp = Jb_s * (Hb\PL);
    VelbyPL(~index) =  temp(~index);                                        % 運動量変化による見かけの速度

    % 目標関節角速度計算
    qd_des = pinv(J) * (endEffecVel - VelbyPL);                                     % 関節角速度．運動量変化について考える.
    qdd_des = (qd_des - SV.qd) / d_time;                                    % 関節角加速度
    JointTau = H_asuta * qdd_des + C_asuta;% - Jg' * F_c;                     % 8関節トルク（set wrist active joint）
    %JointTau = Ck .* (qd_des - SV.qd) + Cd .* (qdd_des - SV.qdd)
    
end