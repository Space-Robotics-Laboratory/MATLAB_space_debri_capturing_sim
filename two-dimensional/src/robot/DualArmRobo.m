% 双腕ロボットクラス
% 
% 2023.1 uchia akiyoshi
% 
% シミュレーション中でロボットの動作計算と制御の区別を明確にするために作成
% LP設定 -> DualArmRobo_LP
% クラスを持ちることで計算が遅くなるかは不明．検証が必要
% 前時間のクラスを保持．メモリ圧迫するかも？

classdef DualArmRobo
    properties
        r;                  % 先端球半径
        depth;              % ベース縦長さ
        width;              % ベース横長さ
        height;             % ベース高さ
        wristElast;         % 手首弾性係数
        wristDamp;          % 手首減衰係数
        jointAngMax;        % 関節可動範囲
        jointAngMin;        % 関節可動範囲
        motorTrqMax;        % モータトルク限界
        LP;                 % ロボットリンクパラメータ
        SV;                 % ロボット状態パラメータ
        num_eL;             % 左手手先の番号 1
        num_eR;             % 右手手先の番号 2
        jointsL;            % 左手のジョイント数
        jointsR;            % 右手のジョイント数
        POS_j_L;            % 左手の関節位置 3*4
        POS_j_R;            % 右手の関節位置 3*4
        ORI_j_L;            % 左手の関節角度 3*3*4
        ORI_j_R;            % 右手の関節角度 3*3*4
        POS_e_L;            % 左手先の位置 3*1
        POS_e_R;            % 右手先の位置 3*1
        ORI_e_L;            % 左手先の姿勢 3*3
        ORI_e_R;            % 右手先の姿勢 3*3
        VEL_e_L;            % 左手先の速度 3*1
        VEL_e_R;            % 右手先の速度 3*1
        POS_es_L;           % 左手先端級の位置 3*2
        POS_es_R;           % 右手先端球の位置 3*2
        VEL_es_L;           % 右手先端球の速度 3*2
        VEL_es_R;           % 右手先端球の速度 3*2
%         DualArmRoboPrevious;% 1ステップ前のRoboclass
    end
    methods
        % constructor
        % 呼び出し時に実行し，ParamSettingをもとに初期位置姿勢，各リンクの重心位置，方向余弦行列を計算する
        function obj = DualArmRobo(param)
            if nargin ~= 1
                error("DualArmTestBed must have Paramsters")
            end
            % ロボットリンクパラメータ設定
            obj.LP = DualArmRobo_LP(param);
            % ロボット状態を初期化
            obj.SV = init_SV_DAR(obj.LP);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%% 初期値設定 %%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %%%%%%%%%% ロボット初期値設定 %%%%%%%%%%
            % 形状設定
            obj.depth = param.robot.baseDepth;
            obj.width = param.robot.baseWidth;
            obj.height = param.robot.baseHeight;
            obj.r = param.robot.diameter_endTip * .5;
            obj.wristElast = param.robot.wristElast;
            obj.wristDamp = param.robot.wristDamp;

            % モータ限界値設定
            obj.jointAngMax = param.robot.jointAngle_max; % yet unused
            obj.jointAngMin = param.robot.jointAngle_min; % yet unused
            obj.motorTrqMax = param.robot.motorTorque_max;

            % ベースからnum_eで指定された手先までを結ぶ関節(リンク)を求める  1アーム多リンクなら1, リンクの数を表す
            obj.num_eL = 1;
            obj.jointsL = j_num( obj.LP, obj.num_eL );   % 左手のジョイント数
            obj.num_eR = 2;
            obj.jointsR = j_num( obj.LP, obj.num_eR );   % 右手のジョイント数
            
            % ベースの初期位置・姿勢・速度・角速度
            obj.SV.R0 = param.robot.initial_position;       % 初期位置
            obj.SV.Q0 = param.robot.initial_orientation;    % 初期姿勢
            obj.SV.A0 = rpy2dc( obj.SV.Q0 )';               % 初期姿勢から方向余弦行列を算出
            obj.SV.v0 = param.robot.initial_velocity;       % 初期並進速度
            obj.SV.w0 = param.robot.initial_angularVelocity;% 初期角速度
            
            % ロボットの初期関節角度を設定
            obj.SV.q = param.robot.initial_jointsAngle;     % 縦に格納
            
            % 設定反映
            obj.SV = calc_aa(  obj.LP, obj.SV );        % 各リンクの座標返還行列(方向余弦行列)の計算(リンクi->慣性座標系)
            obj.SV = calc_pos( obj.LP, obj.SV );        % 各リンク重心位置の計算

            % 関節位置角度初期化
            [ obj.POS_j_L, obj.ORI_j_L ] = f_kin_j( obj.LP, obj.SV, obj.jointsL );   % 左手 関節位置・姿勢　jointsLは左手の関節数
            [ obj.POS_j_R, obj.ORI_j_R ] = f_kin_j( obj.LP, obj.SV, obj.jointsR );   % 右手 関節位置・姿勢　jointsRは右手の関節数
            [ obj.POS_e_L, obj.ORI_e_L ] = f_kin_e( obj.LP, obj.SV, obj.jointsL );   % 左手先位置・姿勢（位置は２つの手先の中点）
            [ obj.POS_e_R, obj.ORI_e_R ] = f_kin_e( obj.LP, obj.SV, obj.jointsR );   % 右手先位置・姿勢（位置は２つの手先の中点）
            obj.SV.Q0 = dc2rpy( obj.SV.A0' );                                        % ベース角度のオイラー角表現
            obj.SV.QeL= dc2rpy( obj.ORI_e_L' );                                      % 左端リンクのオイラー角表現
            obj.SV.QeR= dc2rpy( obj.ORI_e_R' );                                      % 右端リンクのオイラー角表現
            obj.POS_es_L = calc_armTipsPos(obj.POS_e_L, obj.ORI_e_L, param);    % 左手の先端球位置 3*2
            obj.POS_es_R = calc_armTipsPos(obj.POS_e_R, obj.ORI_e_R, param);    % 右手の先端球位置 3*2

            % 関節速度初期化
            obj.SV = calc_vel(obj.LP, obj.SV);                                          % 全リンク重心の並進・回転速度計算
            obj.VEL_e_L = calc_vel_e(obj.LP, obj.SV, obj.jointsL);                      % 左手先の並進速度計算
            obj.VEL_e_R = calc_vel_e(obj.LP, obj.SV, obj.jointsR);                      % 右手先の並進速度計算
            obj.VEL_es_L = calc_armTipsVel(obj.VEL_e_L, obj.ORI_e_L, obj.SV.ww(:, 4), param);  % 左手先端球の並進速度計算
            obj.VEL_es_R = calc_armTipsVel(obj.VEL_e_R, obj.ORI_e_R, obj.SV.ww(:, 8), param);  % 右手先端球の並進速度計算

            % 前状態初期化
%             obj.DualArmRoboPrevious = obj;                                           % 0時間では前状態と現状態が一致
        end
        
        % 動力学を計算し，単位時間でロボットの状態を更新する．実際のシミュレーションループでこれを回す．
        % Tauは関節制御トルク．手首の値(tau[4,8])は上書きされ使用されないことに注意（受動関節のため）．
        % ExtWrenchは外力レンチ[[BaseForce; BaseTorque],[LeftEndEfecForce; LeftEndEfecTorque], [RightEndEfecForce; RightEndEfecTorque]]
        % SV.F0 ; [0, 0, 0]'
        % SV.Fe ; zeros(3, 8)
        function obj = update(obj, JointTau, ExtWrench, Parameters)
            % 前時間の自身クラスを保存, 必要か？
%             obj.DualArmRoboPrevious = obj;

            % 能動的な力
            obj.SV.tau = min_abs(JointTau, obj.motorTrqMax);                  % トルク限界値でフィルタした関節トルク代入．tau([4,8])は受動関節なので，上書きされる．

            % 受動的な力
            obj.SV.tau([4, 8]) = -obj.wristDamp  * obj.SV.qd([4, 8]) ...      % 手首関節トルクをバネダンパ系で計算
                                 -obj.wristElast * obj.SV.q([4, 8]);          % 物理係数はパラメータで設定
            obj.SV.Fes = ExtWrench(1:3, 2:5);                       % 手先球にかかる力
            obj.SV.Tes = ExtWrench(4:6, 2:5);                       % 手先球にかかるトルク
            obj.SV.F0 = ExtWrench(1:3, 1);                          % ベース力
            obj.SV.T0 = ExtWrench(4:6, 1);                          % ベーストルク
            obj.SV.Fe(:, 4) = ExtWrench(1:3, 2)+ExtWrench(1:3, 3);  % 左手手先力
            obj.SV.Te(:, 4) = ExtWrench(4:6, 2)+ExtWrench(4:6, 3);  % 左手手先トルク
            obj.SV.Fe(:, 8) = ExtWrench(1:3, 4)+ExtWrench(1:3, 5);  % 右手手先力
            obj.SV.Te(:, 8) = ExtWrench(4:6, 4)+ExtWrench(4:6, 5);  % 右手手先トルク

            % 順運動学によって関節位置，角度を計算
            obj.SV = f_dyn_rk2(obj.LP, obj.SV);                                         % ロボットに関する順動力学
            obj.SV = calc_aa(  obj.LP, obj.SV );                                        % 各リンクの座標返還行列(方向余弦行列)の計算(リンクi->慣性座標系)
            obj.SV = calc_pos( obj.LP, obj.SV );                                        % 各リンク重心位置の計算
            [ obj.POS_j_L, obj.ORI_j_L ] = f_kin_j( obj.LP, obj.SV, obj.jointsL );      % 左手 関節位置・姿勢　jointsLは左手の関節数
            [ obj.POS_j_R, obj.ORI_j_R ] = f_kin_j( obj.LP, obj.SV, obj.jointsR );      % 右手 関節位置・姿勢　jointsRは右手の関節数
            [ obj.POS_e_L, obj.ORI_e_L ] = f_kin_e( obj.LP, obj.SV, obj.jointsL );      % 左手先位置・姿勢（位置は２つの手先の中点）
            [ obj.POS_e_R, obj.ORI_e_R ] = f_kin_e( obj.LP, obj.SV, obj.jointsR );      % 右手先位置・姿勢（位置は２つの手先の中点）
            obj.SV.Q0 = dc2rpy( obj.SV.A0' );                                           % ベース角度のオイラー角表現
            obj.SV.QeL= dc2rpy( obj.ORI_e_L' );                                         % 左端リンクのオイラー角表現
            obj.SV.QeR= dc2rpy( obj.ORI_e_R' );                                         % 右端リンクのオイラー角表現
            obj.POS_es_L = calc_armTipsPos(obj.POS_e_L, obj.ORI_e_L, Parameters);       % 左手の先端球位置 3*2
            obj.POS_es_R = calc_armTipsPos(obj.POS_e_R, obj.ORI_e_R, Parameters);       % 右手の先端球位置 3*2

            % 順運動学によって関節速度を計算
            obj.SV = calc_vel(obj.LP, obj.SV);                                          % 全リンク重心の並進・回転速度計算
            obj.VEL_e_L = calc_vel_e(obj.LP, obj.SV, obj.jointsL);                      % 左手先の並進速度計算 3*1
            obj.VEL_e_R = calc_vel_e(obj.LP, obj.SV, obj.jointsR);                      % 右手先の並進速度計算 3*1
            obj.VEL_es_L = calc_armTipsVel(obj.VEL_e_L, obj.ORI_e_L, obj.SV.ww(:, 4), Parameters);  % 左手先端球の並進速度計算 3*2
            obj.VEL_es_R = calc_armTipsVel(obj.VEL_e_R, obj.ORI_e_R, obj.SV.ww(:, 8), Parameters);  % 右手先端球の並進速度計算 3*2
        end
    end
end