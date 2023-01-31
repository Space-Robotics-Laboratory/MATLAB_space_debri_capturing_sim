% 構造対のメンバを列ベクトルとして，時系列データとしてファイルに保存するためのクラス
% 保存されるメンバは自体は構造体ではなく行列である必要がある
% 
% 2023.1 uchida akioshi
%

classdef DataSaver
    properties
        filePath;
        fileId;
        delimiter;
        newline;
        datline;
        matrixList;
    end
    methods
        % constructor インスタンス作成時に呼び出し
        function obj = DataSaver(matList, param)
            matrixList = matList;
            dataName = 0
            newline = '\n';
            delimiter = param.Delimiter;

        end
        % ヘッダー作成
        % 保存する行列の変数名，行番号，列番号をヘッダーに設定する
        function set_Head()
            
        end
        
        % ファイルに書き出し
        % datListは一次元配列であり，fileIdに保存
        function write(datList)
            num = size(datList);
            line = [repmat([ValueType, Delimiter], num), NewLine];
            fprintf(FileID, line, Values);
        end
    end
end