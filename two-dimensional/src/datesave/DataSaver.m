% 構造対のメンバを列ベクトルとして，時系列データとしてファイルに保存するためのクラス
% 保存されるメンバは自体は構造体ではなく行列である必要がある
% 
% 2023.1 uchida akioshi
%

classdef DataSaver
    properties
        paramStruct;
        datStruct;
        datSaveName;
        datSaveFolder;
        datNum;
        index;
        timer_length;
    end
    methods
        % constructor インスタンス作成時に呼び出し
        function obj = DataSaver(paths, param)
            times = param.general.minusTime : param.general.divTime : param.general.endTime ;
            obj.datSaveName = [paths.datfile, '/savedDat.csv'];
            obj.datSaveFolder = paths.datfile;
            row = length(times);
            obj.datNum = row;
            obj.index = 0;

            % パラメータ保存
%             obj.paramStruct.linkLength = 
            
            % 時刻
            obj.datStruct.time = times';

            %%% Anime information
            % robot
            obj.datStruct.roboR0 = zeros(row, 3);                            % ベース重心位置
            obj.datStruct.roboQ0 = zeros(row, 3);                            % ベースオイラー角
            obj.datStruct.roboJointLPos = zeros(row, 12);                    % 左手関節位置
            obj.datStruct.roboJointRPos = zeros(row, 12);                    % 右手関節位置
            obj.datStruct.roboEndEffecLPos = zeros(row, 6);                  % 左手手先先端球位置
            obj.datStruct.roboEndEffecRPos = zeros(row, 6);                  % 右手手先球位置
            obj.datStruct.roboEndEffecLOri = zeros(row, 3);                  % 左手手先オイラー角
            obj.datStruct.roboEndEffecROri = zeros(row, 3);                  % 右手手先オイラー角
            obj.datStruct.jointAng = zeros(row, 8);                          % 関節角度

            % target 
            obj.datStruct.targR0 = zeros(row, 3);
            obj.datStruct.targQ0 = zeros(row, 3);
                
            %%% Force and Torque information
            % ロボット手先力
            obj.datStruct.endTipL1Force = zeros(row, 3);
            obj.datStruct.endTipL2Force = zeros(row, 3);
            obj.datStruct.endTipR1Force = zeros(row, 3);
            obj.datStruct.endTipR2Force = zeros(row, 3);

            obj.datStruct.endTipL1Torque = zeros(row, 3);
            obj.datStruct.endTipL2Torque = zeros(row, 3);
            obj.datStruct.endTipR1Torque = zeros(row, 3);
            obj.datStruct.endTipR2Torque = zeros(row, 3);
            
            % ロボ関節速度
            obj.datStruct.jointVelocity = zeros(row, 8);

            % ロボ関節トルク
            obj.datStruct.jointTorque = zeros(row, 8);

            % ターゲット外力
            obj.datStruct.targForce = zeros(row, 3);

            % 運動量
            obj.datStruct.PLsum = zeros(row, 2);

            % 目標手先位置
            obj.datStruct.desHandPos = zeros(row, 6);

            %%% velocity information
            % ロボベース回転速度
            obj.datStruct.baseW = zeros(row, 3);

            % ターゲット並進速度
            obj.datStruct.targetV = zeros(row, 3);

            % ターゲット角速度
            obj.datStruct.targetW = zeros(row, 3);

            % ロボット手先速度(エンドエフェクター代表点速度)
            obj.datStruct.roboEndEffecLVel = zeros(row, 3);
            obj.datStruct.roboEndEffecRVel = zeros(row, 3);

            %%% error log
%             obj.datStruct.error = zeros(row, 1);
            
            %%% stability
            obj.datStruct.velocityManipulability = zeros(row, 2);
        end
        % 保存するデータを更新
        function obj = update(obj, robo, target, controller, param)
            %%%%%%%%%%%%%%%%%%%%%%%%
            % インデックス更新
            obj.index = obj.index +1;
            
            %%% Anime information
            % robot
            obj.datStruct.roboR0(obj.index, :) = robo.SV.R0';                            % ベース重心位置
            obj.datStruct.roboQ0(obj.index, :) = robo.SV.Q0';                            % ベースオイラー角
            obj.datStruct.roboJointLPos(obj.index, :) = reshape(robo.POS_j_L, [1, 12]);  % 左手関節位置
            obj.datStruct.roboJointRPos(obj.index, :) = reshape(robo.POS_j_R, [1, 12]);  % 右手関節位置
            obj.datStruct.roboEndEffecLPos(obj.index, :) = reshape(robo.POS_es_L,[1, 6]);% 左手手先位置
            obj.datStruct.roboEndEffecRPos(obj.index, :) = reshape(robo.POS_es_R,[1, 6]);% 右手手先位置
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
            obj.datStruct.endTipR1Force(obj.index, :) = robo.SV.Fes(:, 3)';
            obj.datStruct.endTipR2Force(obj.index, :) = robo.SV.Fes(:, 4)';

            obj.datStruct.endTipL1Torque(obj.index, :) = robo.SV.Tes(:, 1)';
            obj.datStruct.endTipL2Torque(obj.index, :) = robo.SV.Tes(:, 2)';
            obj.datStruct.endTipR1Torque(obj.index, :) = robo.SV.Tes(:, 3)';
            obj.datStruct.endTipR2Torque(obj.index, :) = robo.SV.Tes(:, 4)';

            % ロボ関節速度
            obj.datStruct.jointVelocity(obj.index, :) = robo.SV.qd';
            
            % ロボ関節トルク
            obj.datStruct.jointTorque(obj.index, :) = robo.SV.tau';

            % ターゲット外力
            obj.datStruct.targForce(obj.index, :) = target.SV.F0';

            % 運動量
            dat = calc_momentum(robo.LP, robo.SV) + calc_momentum(target.LP, target.SV);
            obj.datStruct.PLsum(obj.index, :) = [vecnorm(dat(1:3,1)), vecnorm(dat(4:6,1))];

            % 目標手先位置
            time = obj.datStruct.time(obj.index);
            desPathway = controller.pathway.goingTo(time, param);
            obj.datStruct.desHandPos(obj.index, :) = reshape(desPathway(1:3, :), [1 ,6]);

            %%% Velocity information
            % ロボベース回転速度
            obj.datStruct.baseW(obj.index, :) = robo.SV.w0';

            % ターゲット並進速度
            obj.datStruct.targetV(obj.index, :) = target.SV.v0';

            % ターゲット角速度
            obj.datStruct.targetW(obj.index, :) = target.SV.w0';

            % ロボット手先速度
            obj.datStruct.roboEndEffecLVel(obj.index, :) = robo.VEL_e_L;
            obj.datStruct.roboEndEffecRVel(obj.index, :) = robo.VEL_e_R;

            %%% stability
            obj.datStruct.velocityManipulability(obj.index, 1) = measure_velocity_manipulability(1, 'n', 'volume', robo.LP, robo.SV);
            obj.datStruct.velocityManipulability(obj.index, 2) = measure_velocity_manipulability(2, 'n', 'volume', robo.LP, robo.SV);
        end
        
        % ファイルに書き出し
        % datListは一次元配列であり，fileIdに保存
        function obj=write(obj, param)
            % save as csv
            obj.timer_length = obj.index;
            table = struct2table(obj.datStruct);
            writetable(table, obj.datSaveName)

            % save as mat
            temp = obj.datStruct;
            save([obj.datSaveFolder, '/param.mat'], "param", '-mat')
            save([obj.datSaveFolder, '/datStruct.mat'], "temp", '-mat')
        end
    end
end