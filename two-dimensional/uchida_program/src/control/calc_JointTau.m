% 一般化ヤコビ行列を用いて目標手先速度から関節トルクを計算する
% 手先力がある場合にも手先速度を制御可能にする作業が残っている
% 2023.1 uchida akiyoshi
%
% 阿部さん修論参照．変数名もできるだけ論文を参考しにているので，確認するときは論文と比較しながら
% 運動量，角運動量が0の場合
% 先首関節はバネダンパ系の受動関節であるので，関節速度に制約が加わる
% input : DualArmRobo class, DesierdHandVelocity 6*1
% output: JointTorque 6*1

function JointTau = calc_JointTau(DualArmRobo, DesiredHandVel, RoboExtWrench, Parameters)
global d_time
    LP = DualArmRobo.LP;
    SV = DualArmRobo.SV;
    num_eL = DualArmRobo.num_eL;
    num_eR = DualArmRobo.num_eR;

    % 一般化慣性行列，一般化速度非線形項計算
    [H_asuta, C_asuta] = calc_asuta_2arm(LP, SV, num_eL, num_eR);

    % 一般化ヤコビアン計算
    Jg = calc_gj_2arm(LP, SV, 1, 2);

    % 目標自由度削減 6*8
    Jg_s = Jg([1,2,6, 7,8,12], :);

    % 受動関節j4, j8のトルク条件を満たすqdの条件計算 
    % H_c * qdd + C_c = tau_c
    % tau_cは前時間のものを使用
%     tau([4, 8]) = -Parameters.WristDamp  * obj.SV.qd([4, 8]) ...      % 手首関節トルクをバネダンパ系で計算
%                          -Parameters.WristElast * obj.SV.q([4, 8]);          % 物理係数はパラメータで設定
    F_c = reshape(RoboExtWrench(:, 2:3), [12,1]);
%     F_c = 0;


    % 目標関節角速度計算
    qd_des = pinv(Jg_s) * DesiredHandVel;
    qdd_des = (qd_des - SV.qd) / d_time;
    JointTau_0 = H_asuta * qdd_des + C_asuta - Jg' * F_c;
%     JointTau = JointTau_0([1:3, 5:7]');
    JointTau = JointTau_0;
end
