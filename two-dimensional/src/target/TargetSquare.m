classdef TargetSquare
    properties
        depth;   % ターゲット縦
        width;   % ターゲット横
        height;  % ターゲット高さ  
        m2G;     % 質量中心から幾何中心への相対位置ベクトル
        LP;      % ターゲットリンクパラメータ
        SV;      % ターゲット状態パラメータ
    end
    methods
        % constructor
        % 呼び出し時に実行し，ParamSettingをもとに初期位置姿勢，各リンクの重心位置，方向余弦行列を計算する
        function obj = TargetSquare(Parameters)
            if nargin ~= 1
                error("TargetSquare must have Paramsters")
            end
            % ターゲットリンクパラメータ設定
            obj.LP = TargetSquare_LP(Parameters);
            % ターゲット状態を初期化
            obj.SV = init_SV_DAR(obj.LP);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%% 初期値設定 %%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % 形状設定
            obj.depth = Parameters.TargetDepth;
            obj.width = Parameters.TargetWidth;
            obj.height = Parameters.TargetHeight;
            obj.m2G = Parameters.TargetMCenter2GCenter;
            
            %%%%%%%%%% ターゲット初期値設定 %%%%%%%%%%
           
            % ベースの初期位置・姿勢・速度・角速度
            obj.SV.R0 = Parameters.TargetPosition0;       % 初期位置
            obj.SV.Q0 = Parameters.TargetOrientation0;    % 初期姿勢
            obj.SV.A0 = rpy2dc( obj.SV.Q0 )';             % 初期姿勢から方向余弦行列を算出
            obj.SV.v0 = Parameters.TargetVelocity0;       % 初期並進速度
            obj.SV.w0 = Parameters.TargetAngVel0;         % 初期角速度
            obj.SV = calc_aa(  obj.LP, obj.SV );          % 各リンクの座標返還行列(方向余弦行列)の計算(リンクi->慣性座標系)
            obj.SV = calc_pos( obj.LP, obj.SV );          % 各リンク重心位置の計算
            obj.m2G = obj.SV.A0 * obj.m2G;                % ターゲット質量中心に対する幾何中心位置ベクトル更新
        end
        
        % 動力学を計算し，単位時間でロボットの状態を更新する．実際のシミュレーションループでこれを回す．
        % Tauは関節制御トルク
        % ExtWrenchは外力レンチ[[BaseTorque; BaseForce],[LeftEdgeTorque; LeftEdgeForce], [RightEdgeTorque; RightEdgeForce]]
        % SV.F0 ; [0, 0, 0]'
        % SV.Fe ; zeros(3, 8)
        function obj = update(obj, ExtWrench)
            %受動的な力
            obj.SV.F0 = ExtWrench(1:3, 1);          % ターゲット力
            obj.SV.T0 = ExtWrench(4:6, 1);          % ターゲットトルク


            % 順運動学によって関節位置，角度を計算
            obj.SV = f_dyn_rk2(obj.LP, obj.SV);     % ターゲットに関する順動力学
            obj.SV.Q0 = dc2rpy( obj.SV.A0' );       % ベース角度のオイラー角表現
            obj.m2G = obj.SV.A0 * obj.m2G;          % ターゲット質量中心に対する幾何中心位置ベクトル更新
        end
    end
end