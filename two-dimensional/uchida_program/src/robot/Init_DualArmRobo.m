% initialize DualArmRobo structure

function DualArmRobo = Init_DualArmRobo(Param)
    % ロボットリンクパラメータ設定
    DualArmRobo.LP = DualArmRobo_LP(Param);
    % ロボット状態を初期化
    DualArmRobo.SV = init_SV(DualArmRobo.LP);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%% 初期値設定 %%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%% ロボット初期値設定 %%%%%%%%%%
    % ベースからnum_eで指定された手先までを結ぶ関節(リンク)を求める  1アーム多リンクなら1, リンクの数を表す
    DualArmRobo.num_eL = 1;
    DualArmRobo.jointsL = j_num( DualArmRobo.LP, DualArmRobo.num_eL );   % 左手のジョイント数
    DualArmRobo.num_eR = 2;
    DualArmRobo.jointsR = j_num( DualArmRobo.LP, DualArmRobo.num_eR );   % 右手のジョイント数
    
    % ベースの初期位置・姿勢・速度・角速度
    DualArmRobo.SV.R0 = Param.BasePosition0;            % 初期位置
    DualArmRobo.SV.Q0 = Param.BaseOrientation0;         % 初期姿勢
    DualArmRobo.SV.A0 = rpy2dc( DualArmRobo.SV.Q0 )';   % 初期姿勢から方向余弦行列を算出
    DualArmRobo.SV.v0 = Param.BaseVelocity0;            % 初期並進速度
    DualArmRobo.SV.w0 = Param.BaseAngVel0;              % 初期角速度
    DualArmRobo.SV = calc_aa(  DualArmRobo.LP, DualArmRobo.SV );        % 各リンクの座標返還行列(方向余弦行列)の計算(リンクi->慣性座標系)
    DualArmRobo.SV = calc_pos( DualArmRobo.LP, DualArmRobo.SV );        % 各リンク重心位置の計算

    % % ロボットの初期関節角度を設定
    DualArmRobo.SV.q = [ Param.LinkAngLeft; Param.LinkAngRight ];       % 縦に格納
end