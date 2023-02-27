% 構造対のメンバを列ベクトルとして，時系列データとしてファイルに保存するためのクラス
% 保存されるメンバは自体は構造体ではなく行列である必要がある
% 
% 2023.1 uchida akioshi
%

classdef DataSaver
    properties
        datStruct;
        datTable;
        filePath;
        datNum;
    end
    methods
        % constructor インスタンス作成時に呼び出し
        function obj = DataSaver(paths, param)
            obj.filePath = [paths.datfile, '/savedDat.csv'];
            row = (param.EndTime + param.MinusTime) / param.DivTime;
            obj.datNum = row;
            
            % 時刻
            obj.datStruct.time = zeros(row, 1);

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

            % target 
            obj.datStruct.targR0 = zeros(row, 3);
            obj.datStruct.targQ0 = zeros(row, 3);
                
            %%% Force and Torque information
            % ロボット手先力
            obj.datStruct.endEffecLForce = zeros(row, 3);
            obj.datStruct.endEffecRForce = zeros(row, 3);
            obj.datStruct.endEffecLTorque = zeros(row, 3);
            obj.datStruct.endEffecRTorque = zeros(row, 3);
            
            % ロボ関節トルク
            obj.datStruct.jointTorque = zeros(row, 8);
        end
        % 保存するデータを更新
        function obj = update(obj, robo, target, time, index)
            % 時刻
            obj.datStruct.time(index, :) = time;
            
            %%% Anime information
            % robot
            obj.datStruct.roboR0(index, :) = robo.SV.R0';                            % ベース重心位置
            obj.datStruct.roboQ0(index, :) = robo.SV.Q0';                            % ベースオイラー角
            obj.datStruct.roboJointLPos(index, :) = reshape(robo.POS_j_L, [1, 12]);  % 左手関節位置
            obj.datStruct.roboJointRPos(index, :) = reshape(robo.POS_j_R, [1, 12]);  % 右手関節位置
            obj.datStruct.roboEndEffecLPos(index, :) = reshape(robo.POS_es_L,[1, 6]);% 左手手先位置
            obj.datStruct.roboEndEffecRPos(index, :) = reshape(robo.POS_es_R,[1, 6]);% 右手手先位置
            obj.datStruct.roboEndEffecLOri(index, :) = robo.SV.QeL';                 % 左手手先オイラー角
            obj.datStruct.roboEndEffecROri(index, :) = robo.SV.QeR';                 % 右手手先オイラー角

            % target 
            obj.datStruct.targR0(index, :) = target.SV.R0';
            obj.datStruct.targQ0(index, :) = target.SV.Q0';

            %%% Force and Torque information
            % ロボット手先力
            obj.datStruct.endEffecLForce(index, :) = robo.SV.Fe(:, 4)';
            obj.datStruct.endEffecRForce(index, :) = robo.SV.Fe(:, 8)';
            obj.datStruct.endEffecLTorque(index, :) = robo.SV.Te(:, 4)';
            obj.datStruct.endEffecRTorque(index, :) = robo.SV.Te(:, 8)';
            
            % ロボ関節トルク
            obj.datStruct.jointTorque(index, :) = robo.SV.tau';

            %%% Velocity information
        end
        
        % ファイルに書き出し
        % datListは一次元配列であり，fileIdに保存
        function write(obj)
            table = struct2table(obj.datStruct);
            writetable(table, obj.filePath)
        end
    end
end