classdef DualArmRobo
    properties
        LP;      % ロボットリンクパラメータ
        SV;      % ロボット状態パラメータ
        num_eL;  % 左手手先の番号 1
        num_eR;  % 右手手先の番号 2
        jointsL; % 左手のジョイント数
        jointsR; % 右手のジョイント数
        POS_j_L; % 左手の関節位置 3*4
        POS_j_R; % 右手の関節位置 3*4
        ORI_j_L; % 左手の関節角度 3*3*4
        ORI_j_R; % 右手の関節角度 3*3*4
        POS_e_L; % 左手先の位置 3*1
        POS_e_R; % 右手先の位置 3*1
        ORI_e_L; % 左手先の姿勢 3*3
        ORI_e_R; % 右手先の姿勢 3*3
        POS_es_L; % 左手先端級の位置 3*2
        POS_es_R; % 右手先端球の位置 3*2
    end
    methods
        % constructor
        % 呼び出し時に実行し，ParamSettingをもとに初期位置姿勢，各リンクの重心位置，方向余弦行列を計算する
        function obj = DualArmRobo(Parameters)
            if nargin ~= 1
                error("DualArmTestBed must have Paramsters")
            end
            % ロボットリンクパラメータ設定
            obj.LP = DualArmRobo_LP(Parameters);
            % ロボット状態を初期化
            obj.SV = init_SV(obj.LP);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%% 初期値設定 %%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %%%%%%%%%% ロボット初期値設定 %%%%%%%%%%
            % ベースからnum_eで指定された手先までを結ぶ関節(リンク)を求める  1アーム多リンクなら1, リンクの数を表す
            obj.num_eL = 1;
            obj.jointsL = j_num( obj.LP, obj.num_eL );   % 左手のジョイント数
            obj.num_eR = 2;
            obj.jointsR = j_num( obj.LP, obj.num_eR );   % 右手のジョイント数
            
            % ベースの初期位置・姿勢・速度・角速度
            obj.SV.R0 = Parameters.BasePosition0;       % 初期位置
            obj.SV.Q0 = Parameters.BaseOrientation0;    % 初期姿勢
            obj.SV.A0 = rpy2dc( obj.SV.Q0 )';           % 初期姿勢から方向余弦行列を算出
            obj.SV.v0 = Parameters.BaseVelocity0;       % 初期並進速度
            obj.SV.w0 = Parameters.BaseAngVel0;         % 初期角速度
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
            obj.POS_es_L = calc_ArmTips(obj.POS_e_L, obj.ORI_e_L, Parameters);       % 左手の先端球位置 3*2
            obj.POS_es_R = calc_ArmTips(obj.POS_e_R, obj.ORI_e_R, Parameters);       % 右手の先端球位置 3*2

            % % ロボットの初期関節角度を設定
            obj.SV.q = [ Parameters.LinkAngLeft; Parameters.LinkAngRight ];    % 縦に格納
        end
        
        % 動力学を計算し，単位時間でロボットの状態を更新する．実際のシミュレーションループでこれを回す．
        % Tauは関節制御トルク.手首を除く，joint[1 2 3, 5 6 7]'であることに注意
        % ExtWrenchは外力レンチ[[BaseForce; BaseTorque],[LeftEdgeForce; LeftEdgeTorque], [RightEdgeForce; RightEdgeTorque]]
        % SV.F0 ; [0, 0, 0]'
        % SV.Fe ; zeros(3, 8)
        function obj = update(obj, JointTau, ExtWrench, Parameters)
            % 能動的な力
%             obj.SV.tau([1,2,3,5,6,7]) = JointTau;   % 関節トルク代入．手首は受動関節．
            obj.SV.tau = JointTau;                  % 関節トルク代入．手首は能動関節．

            % 受動的な力
            obj.SV.tau([4, 8]) = -Parameters.WristDamp  * obj.SV.qd([4, 8]) ...      % 手首関節トルクをバネダンパ系で計算
                                 -Parameters.WristElast * obj.SV.q([4, 8]);          % 物理係数はパラメータで設定
            obj.SV.F0 = ExtWrench(1:3, 1);          % ベース力
            obj.SV.T0 = ExtWrench(4:6, 1);          % ベーストルク
            obj.SV.Fe(:, 4) = ExtWrench(1:3, 2);    % 左手手先力
            obj.SV.Te(:, 4) = ExtWrench(4:6, 2);    % 左手手先トルク
            obj.SV.Fe(:, 8) = ExtWrench(1:3, 3);    % 右手手先力
            obj.SV.Te(:, 8) = ExtWrench(4:6, 3);    % 右手手先トルク

            % 順運動学によって関節位置，角度を計算
            obj.SV = f_dyn_rk2(obj.LP, obj.SV);                                      % ロボットに関する順動力学
            obj.SV = calc_aa(  obj.LP, obj.SV );                                     % 各リンクの座標返還行列(方向余弦行列)の計算(リンクi->慣性座標系)
            obj.SV = calc_pos( obj.LP, obj.SV );                                     % 各リンク重心位置の計算
            [ obj.POS_j_L, obj.ORI_j_L ] = f_kin_j( obj.LP, obj.SV, obj.jointsL );   % 左手 関節位置・姿勢　jointsLは左手の関節数
            [ obj.POS_j_R, obj.ORI_j_R ] = f_kin_j( obj.LP, obj.SV, obj.jointsR );   % 右手 関節位置・姿勢　jointsRは右手の関節数
            [ obj.POS_e_L, obj.ORI_e_L ] = f_kin_e( obj.LP, obj.SV, obj.jointsL );   % 左手先位置・姿勢（位置は２つの手先の中点）
            [ obj.POS_e_R, obj.ORI_e_R ] = f_kin_e( obj.LP, obj.SV, obj.jointsR );   % 右手先位置・姿勢（位置は２つの手先の中点）
            obj.SV.Q0 = dc2rpy( obj.SV.A0' );                                        % ベース角度のオイラー角表現
            obj.SV.QeL= dc2rpy( obj.ORI_e_L' );                                      % 左端リンクのオイラー角表現
            obj.SV.QeR= dc2rpy( obj.ORI_e_R' );                                      % 右端リンクのオイラー角表現
            obj.POS_es_L = calc_ArmTips(obj.POS_e_L, obj.ORI_e_L, Parameters);       % 左手の先端球位置 3*2
            obj.POS_es_R = calc_ArmTips(obj.POS_e_R, obj.ORI_e_R, Parameters);       % 右手の先端球位置 3*2
        end
    end
end