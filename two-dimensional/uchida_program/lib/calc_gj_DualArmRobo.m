% 双腕ロボ一般化ヤコビアンを計算する
%
% 2023.1 uchida akiyoshi
%
% SpaceDynを用いて簡略化できるが，勉強のためある程度自分で書いてみた．特に意味はなかった．
% -> calc_gj_2arm
%
% 阿部さん修論参照．変数名もできるだけ論文を参考しにているので，確認するときは論文と比較しながら

function Jg = calc_gj_DualArmRobo(DualArmRobo)

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
    HH = calc_hh(DualArmRobo.LP, DualArmRobo.SV);
    Hb = HH(1:6, 1:6);

    % リンク慣性行列
    % [Ii * 4], 3*3*4次元
    I_L = reshape( DualArmRobo.LP.inertia(:,  1:12), [3, 3, 4]);
    I_R = reshape( DualArmRobo.LP.inertia(:, 13:24), [3, 3, 4]);


    % Calculate Jacobian matrix related to the motion of the end-effector and the joints
    % spacesynのcrossでなく，matlabのcrossを使うことによって行列どうしの外積を計算可能
    % kは関節の回転軸ベクトル．
    Jm_L = [cross(k_L, (ph_L - p_L)); k_L];
    Jm_R = [cross(k_R, (ph_R - p_R)); k_R];


    % Calculate Jacobian matrix related to the motion of the end-effector and the base
    % tildeは歪対称行列作成関数 
    Jb_L = eye(6);
    Jb_R = eye(6);
    Jb_L(1:3, 4:6) = -tilde(ph_L - r0);
    Jb_R(1:3, 4:6) = -tilde(ph_R - r0);


    % Calculate Interference matrix of the base and the arm
    % Jtを計算
    E = true(4);
    index = ~triu(E);

    % Left
    K_L = repmat(k_L, 1, 1, 4);                         % k_Lの次元を拡張 
    Jr_L = K_L;                                        
    Jr_L(:, index) = 0;                                % K_L(:, i, j)の上三角部分以外を0
    M_L = permute( repmat(m_L, 3, 1, 4), [1, 3, 2]);    % m_Lの次元を拡張
    R_L = permute( repmat(r_L, 1, 1, 4), [1, 3, 2]);    % r_Lの次元を拡張
    P_L = repmat(p_L, 1, 1, 4);                         % p_Lの次元を拡張
    Jt_L_0 = (R_L - P_L);
    Jt_L = cross(Jr_L, Jt_L_0);

    % Right
    K_R = repmat(k_R, 1, 1, 4);     
    Jr_R = K_R;                                        
    Jr_R(:, index) = 0;  
    M_R = permute( repmat(m_R, 3, 1, 4), [1, 3, 2]);
    R_R = permute( repmat(r_R, 1, 1, 4), [1, 3, 2]);
    P_R = repmat(p_R, 1, 1, 4);
    Jt_R_0 = (R_R - P_R);
    Jt_R = cross(Jr_R, Jt_R_0);

    % Jtwを計算
    Jtw_L = sum(Jt_L .* M_L, 3);
    Jtw_R = sum(Jt_R .* M_R, 3);

    % Hwmを計算
    % Left
    Hw_L_1_0 = pagemtimes(I_L, Jr_L);
    Hw_L_1 = sum(Hw_L_1_0, 3);
    Hw_L_2_0 =  cross((R_L - r0), Jt_L) .* M_L;
    Hw_L_2 = sum(Hw_L_2_0, 3);
    Hw_L = Hw_L_1 + Hw_L_2;
    % Right
    Hw_R_1_0 = pagemtimes(I_R, Jr_R);
    Hw_R_1 = sum(Hw_R_1_0, 3);
    Hw_R_2_0 =  cross((R_R - r0), Jt_R) .* M_R;
    Hw_R_2 = sum(Hw_R_2_0, 3);
    Hw_R = Hw_R_1 + Hw_R_2;

    % Hbmを計算
    Hbm_L = [Jtw_L; Hw_L];
    Hbm_R = [Jtw_R; Hw_R];

    % 双腕ロボ一般化ヤコビアン計算
    Jg = [ Jm_L - Jb_L*(Hb\Hbm_L), -Jb_L*(Hb\Hbm_R)       ;
           -Jb_R*(Hb\Hbm_L)      , Jm_R - Jb_R*(Hb\Hbm_R)];
end