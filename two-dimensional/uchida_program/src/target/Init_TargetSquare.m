% initialize target, which shsape is square

function TargetSquare = Init_TargetSquare(Param)
    % ターゲットリンクパラメータ設定
    TargetSquare.LP = TargetSquare_LP(Param);
    % ターゲット状態を初期化
    TargetSquare.SV = init_SV(TargetSquare.LP);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%% 初期値設定 %%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%% ターゲット初期値設定 %%%%%%%%%%
   
    % ベースの初期位置・姿勢・速度・角速度
    TargetSquare.SV.R0 = Param.TargetPosition0;         % 初期位置
    TargetSquare.SV.Q0 = Param.TargetOrientation0;      % 初期姿勢
    TargetSquare.SV.A0 = rpy2dc( TargetSquare.SV.Q0 )'; % 初期姿勢から方向余弦行列を算出
    TargetSquare.SV.v0 = Param.TargetVelocity0;         % 初期並進速度
    TargetSquare.SV.w0 = Param.TargetAngVel0;           % 初期角速度
    TargetSquare.SV = calc_aa(  TargetSquare.LP, TargetSquare.SV ); % 各リンクの座標返還行列(方向余弦行列)の計算(リンクi->慣性座標系)
    TargetSquare.SV = calc_pos( TargetSquare.LP, TargetSquare.SV ); % 各リンク重心位置の計算
end