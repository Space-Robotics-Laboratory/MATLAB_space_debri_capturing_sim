% 構造対のメンバを列ベクトルとして，時系列データとしてファイルに保存するためのクラス
% 保存されるメンバは自体は構造体ではなく行列である必要がある
% 
% 2023.1 uchida akioshi
%

classdef DataSaver
    properties
        paramStruct;
        datStruct;
        filePath;
        datNum;
        index;
    end
    methods
        % constructor インスタンス作成時に呼び出し
        function obj = DataSaver(paths, param)
            obj.filePath = [paths.datfile, '/savedDat.csv'];
            row = (param.EndTime + param.MinusTime) / param.DivTime;
            obj.datNum = row;
            obj.index = 1;

            % パラメータ保存
%             obj.paramStruct.linkLength = 
            
            % 時刻
            obj.datStruct.time = zeros(row, 1);

            %%% Anime information
            % robot
            obj.datStruct.roboR0 = zeros(row, 3);                            % ベース重心位置
            obj.datStruct.roboQ0 = zeros(row, 3);                            % ベースオイラー角
            obj.datStruct.roboJointLPos = zeros(row, 18);                    % 左手関節位置
            obj.datStruct.roboJointRPos = zeros(row, 18);                    % 右手関節位置
            obj.datStruct.roboEndEffecLPos = zeros(row, 9);                  % 左手手先先端球位置
            obj.datStruct.roboEndEffecRPos = zeros(row, 9);                  % 右手手先球位置
            obj.datStruct.roboEndEffecLOri = zeros(row, 3);                  % 左手手先オイラー角
            obj.datStruct.roboEndEffecROri = zeros(row, 3);                  % 右手手先オイラー角
            obj.datStruct.jointAng = zeros(row, 12);                          % 関節角度

            % target 
            obj.datStruct.targR0 = zeros(row, 3);
            obj.datStruct.targQ0 = zeros(row, 3);
                
            %%% Force and Torque information
            % ロボット手先力
            obj.datStruct.endTipL1Force = zeros(row, 3);
            obj.datStruct.endTipL2Force = zeros(row, 3);
            obj.datStruct.endTipL3Force = zeros(row, 3);
            obj.datStruct.endTipR1Force = zeros(row, 3);
            obj.datStruct.endTipR2Force = zeros(row, 3);
            obj.datStruct.endTipR3Force = zeros(row, 3);

            obj.datStruct.endTipL1Torque = zeros(row, 3);
            obj.datStruct.endTipL2Torque = zeros(row, 3);
            obj.datStruct.endTipL3Torque = zeros(row, 3);
            obj.datStruct.endTipR1Torque = zeros(row, 3);
            obj.datStruct.endTipR2Torque = zeros(row, 3);
            obj.datStruct.endTipR3Torque = zeros(row, 3);
            
            % ロボ関節トルク
            obj.datStruct.jointTorque = zeros(row, 12);

            % ターゲット外力
            obj.datStruct.targForce = zeros(row, 3);

            % 運動量
            obj.datStruct.robo_moment   = zeros(row, 6);
            obj.datStruct.target_moment = zeros(row, 6);

            % 目標手先位置
            obj.datStruct.desHandPos = zeros(row, 6);

            %%% velocity information
            % ターゲット並進速度
            obj.datStruct.targetV = zeros(row, 3);

            % ターゲット角速度
            obj.datStruct.targetWs = zeros(row, 3);
            obj.datStruct.targetWb = zeros(row, 3);

            %%% error log
%             obj.datStruct.error = zeros(row, 1);
        end
        % 保存するデータを更新
        function obj = update(obj, robo, target, controller, time, param)
            % 時刻
            obj.datStruct.time(obj.index, :) = time;
            
            %%% Anime information
            % robot
            obj.datStruct.roboR0(obj.index, :) = robo.SV.R0';                            % ベース重心位置
            obj.datStruct.roboQ0(obj.index, :) = robo.SV.Q0';                            % ベースオイラー角
            obj.datStruct.roboJointLPos(obj.index, :) = reshape(robo.POS_j_L, [1, 18]);  % 左手関節位置
            obj.datStruct.roboJointRPos(obj.index, :) = reshape(robo.POS_j_R, [1, 18]);  % 右手関節位置
            obj.datStruct.roboEndEffecLPos(obj.index, :) = reshape(robo.POS_es_L,[1, 9]);% 左手手先位置
            obj.datStruct.roboEndEffecRPos(obj.index, :) = reshape(robo.POS_es_R,[1, 9]);% 右手手先位置
            obj.datStruct.roboEndEffecLOri(obj.index, :) = robo.SV.QeL';                 % 左手手先オイラー角
            obj.datStruct.roboEndEffecROri(obj.index, :) = robo.SV.QeR';                 % 右手手先オイラー角
            obj.datStruct.jointAng(obj.index, :) = robo.SV.q';                           % 関節角度

            % target 
            obj.datStruct.targR0(obj.index, :) = target.SV.R0';
            obj.datStruct.targQ0(obj.index, :) = target.SV.Q0';

            %%% Force and Torque information
            % ロボット手先力
            obj.datStruct.endTipL1Force(obj.index, :) = robo.SV.Fes(:, 1)';
            obj.datStruct.endTipL2Force(obj.index, :) = robo.SV.Fes(:, 2)';
            obj.datStruct.endTipL3Force(obj.index, :) = robo.SV.Fes(:, 3)';
            obj.datStruct.endTipR1Force(obj.index, :) = robo.SV.Fes(:, 4)';
            obj.datStruct.endTipR2Force(obj.index, :) = robo.SV.Fes(:, 5)';
            obj.datStruct.endTipR3Force(obj.index, :) = robo.SV.Fes(:, 6)';

            obj.datStruct.endTipL1Torque(obj.index, :) = robo.SV.Tes(:, 1)';
            obj.datStruct.endTipL2Torque(obj.index, :) = robo.SV.Tes(:, 2)';
            obj.datStruct.endTipL3Torque(obj.index, :) = robo.SV.Tes(:, 3)';
            obj.datStruct.endTipR1Torque(obj.index, :) = robo.SV.Tes(:, 4)';
            obj.datStruct.endTipR2Torque(obj.index, :) = robo.SV.Tes(:, 5)';
            obj.datStruct.endTipR3Torque(obj.index, :) = robo.SV.Tes(:, 6)';
            
            % ロボ関節トルク
            obj.datStruct.jointTorque(obj.index, :) = robo.SV.tau';

            % ターゲット外力
            obj.datStruct.targForce(obj.index, :) = target.SV.F0';

            % 運動量
            obj.datStruct.robo_moment(obj.index, :) = calc_momentum(robo.LP, robo.SV);
            obj.datStruct.target_moment(obj.index, :) = calc_momentum(target.LP, target.SV);

            % 目標手先位置
            desPathway(1:7, 1) = controller.pathway_L.goingTo(time, param);
            desPathway(1:7, 2) = controller.pathway_R.goingTo(time, param);
            obj.datStruct.desHandPos(obj.index, :) = reshape(desPathway(1:3, :), [1 ,6]);

            %%% Velocity information
            % ターゲット並進速度
            obj.datStruct.targetV(obj.index, :) = target.SV.v0';

            % ターゲット角速度
            obj.datStruct.targetWs(obj.index, :) = target.SV.w0'; % in inertia frame
            obj.datStruct.targetWb(obj.index, :) = bodyFrameBaseAngVel(target.SV)'; % in body frame
            
            %%%%%%%%%%%%%%%%%%%%%%%%
            % インデックス更新
            obj.index = obj.index +1;
        end
        
        % ファイルに書き出し
        % datListは一次元配列であり，fileIdに保存
        function write(obj)
            table = struct2table(obj.datStruct);
            writetable(table, obj.filePath)
        end
    end
end