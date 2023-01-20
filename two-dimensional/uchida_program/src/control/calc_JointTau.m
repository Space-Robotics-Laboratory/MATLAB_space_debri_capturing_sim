% 一般化ヤコビ行列を用いて目標手先速度から関節トルクを計算する
% 2023.1 uchida akiyoshi
% 阿部さん修論参照．変数名もできるだけ論文を参考しにているので，確認するときは論文と比較しながら
% 運動量，角運動量が0の場合
% 先首関節はバネダンパ系の受動関節
% input : DualArmRobo class, DesierdHandVelocity 3*2
% output: JointTorque 6*1（手首以外）

function JointTau = calc_JointTau(DualArmRobo, DesiredHandVel, Parameters)
global d_time
    JointTau = [0, 0, 0, -0, 0, 0];

    % 関節軸ベクトル 3*4
    % 関節方向行列のZ成分は関節軸
    k_L = DualArmRobo.ORI_j_L(:, 3:3:12);   % LeftArm
    k_R = DualArmRobo.ORI_j_R(:, 3:3:12);   % RightArm

    % 関節位置 3*4
    p_L = DualArmRobo.POS_j_L;
    p_R = DualArmRobo.POS_j_R;

    % 手先位置　3*1
    ph_L = DualArmRobo.POS_e_L;
    ph_R = DualArmRobo.POS_e_R;

    % ベース重心位置 3*1
    r0 = DualArmRobo.SV.R0;

    % リンク重心位置 3*4
    r_L = DualArmRobo.SV.RR(:, 1:4);
    r_R = DualArmRobo.SV.RR(:, 5:8);

    % リンク質量 1*4
    m_L = DualArmRobo.LP.m(1:4);
    m_R = DualArmRobo.LP.m(5:8);

    % ロボットベース慣性行列
    Hb = DualArmRobo.LP.inertia0;

    % リンク慣性行列
    I_L = DualArmRobo.LP.inertia(:, 1:12);
    I_R = DualArmRobo.LP.inertia(:, 13:24);


    % Calculate Jacobian matrix related to the motion of the end-effector and the joints
    % spacesynのcrossでなく，matlabのcrossを使うことによって行列どうしの外積を計算可能
    % kは関節の回転軸ベクトル．
    Jm_L = [cross(k_L, (ph_L - p_L)); k_L];
    Jm_R = [cross(k_R, (ph_R - p_R)); k_R];


    % Calculate Jacobian matrix related to the motion of the end-effector and the base
    % tildeは歪対称行列作成関数 
    Jb_L = eye(6);
    Jb_R = eye(6);
    Jb_L([1:3], [4:6]) = -tilde(ph_L - r0);
    Jb_R([1:3], [4:6]) = -tilde(ph_R - r0);


    % Calculate Interference matrix of the base and the arm
    % Jtwを計算するための一次変数計算
    Jtw_L_0 = zeros(3, 4, 4);
    Jtw_L_0(:, 1 ,:) = m_L               .* (r_L - p_L(:, 1)); 
    Jtw_L_0(:, 2, :) = [0, m_L(2:4)]     .* (r_L - p_L(:, 2));
    Jtw_L_0(:, 3, :) = [0, 0, m_L(3:4)]  .* (r_L - p_L(:, 3));
    Jtw_L_0(:, 4, :) = [0, 0, 0, m_L(4)] .* (r_L - p_L(:, 4));
    Jtw_R_0 = zeros(3, 4, 4);
    Jtw_R_0(:, 1 ,:) = m_R               .* (r_R - p_R(:, 1)); 
    Jtw_R_0(:, 2, :) = [0, m_R(2:4)]     .* (r_R - p_R(:, 2));
    Jtw_R_0(:, 3, :) = [0, 0, m_R(3:4)]  .* (r_R - p_R(:, 3));
    Jtw_R_0(:, 4, :) = [0, 0, 0, m_R(4)] .* (r_R - p_R(:, 4));

    % Jtwを計算
    Jtw_L = cross(k_L, sum(Jtw_L_0, 3), 1);
    Jtw_R = cross(k_R, sum(Jtw_R_0, 3), 1);    

    % Hwmを計算
    Htw_L_0 = zeros(3, 4, 4);
    Jtw_L_0(:, 1 ,:) = m_L               .* (r_L - p_L(:, 1)); 
    Jtw_L_0(:, 2, :) = [0, m_L(2:4)]     .* (r_L - p_L(:, 2));
    Jtw_L_0(:, 3, :) = [0, 0, m_L(3:4)]  .* (r_L - p_L(:, 3));
    Jtw_L_0(:, 4, :) = [0, 0, 0, m_L(4)] .* (r_L - p_L(:, 4));



    %Jstar_L = [Jm_L - Jb_L*(Hb\Hbm_L) ]
end
