% 一般化ヤコビ行列を用いて目標手先速度から関節トルクを計算する
% 
% 2023.1 uchida akiyoshi
%
% 阿部さん修論参照．変数名もできるだけ論文を参考しにているので，確認するときは論文と比較しながら
% 運動量，角運動量が0の場合
% 先首関節はバネダンパ系の受動関節であるので，関節速度に制約が加わる
% input : DualArmRobo class, DesierdHandVelocity 6*1, 
%         RoboExtWrench 6*3(base, leftEdgem, rightEdge) 
% output: JointTorque 8*1, but only used 6*1
%
% to do: HHを計算する際，複数の関数で計算しているのでロスがある．関数の構造を変更することを検討．
% 

function JointTau = calc_JointTau(DualArmRobo, DesiredHandVel, RoboExtEst)
global d_time
    LP = DualArmRobo.LP;
    SV = DualArmRobo.SV;
    num_eL = DualArmRobo.num_eL;
    num_eR = DualArmRobo.num_eR;


    % 双腕ロボ一般化慣性行列，一般化速度非線形項計算
    [H_asuta, C_asuta] = calc_asuta_2arm(LP, SV, num_eL, num_eR);

    % 一般化ヤコビアン，ベース手先ヤコビアン，慣性行列計算
    [Jg, Jb, HH] = calc_gj_2arm(LP, SV, 1, 2);
    Hb = HH(1:6, 1:6);

    % 現時刻の運動量計算
    % 現時刻でのロボット速度を使用している．想定される環境に注意
    PL = HH(1:6, 1:6) * [SV.v0; SV.w0] + HH(1:6, 7:6+LP.num_q) * SV.qd;

    % 目標自由度削減 6*8
    Jg_s = Jg([1,2,6, 7,8,12], :);                      
    Jg_s(:, [4,8]) = 0;                     % jg_sは目標速度の次元を削減し，受動関節角速度0を仮定したヤコビアン
    Jb_s = Jb([1,2,6, 7,8,12], :);

    % 手先にかかる外力の推定値を代入 
    F_c = reshape(RoboExtEst(:, 2:3), [12,1]);

    % 目標関節角速度計算
    qd_des = pinv(Jg_s) * (DesiredHandVel - Jb_s * (Hb\PL));                % 関節角速度．運動量変化について考える.
    qdd_des = (qd_des - SV.qd) / d_time;                                    % 関節角加速度
    JointTau = H_asuta * qdd_des + C_asuta - Jg' * F_c;                     % 8関節トルク（set wrist active joint）
end
