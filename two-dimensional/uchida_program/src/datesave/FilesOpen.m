%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%データ書き出し用ファイル準備
%Paths構造体，ファイル名(構造体名)ベクトルを受け取り，FileIDを格納したベクトルを返す
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function FileIDList = FilesOpen(Paths, FileNameList)
    n = size(FileNameList);     %作成するファイルIDサイズ
    FileIDList = zeros(n);      %ファイルID格納配列初期化
    index = 1;                  %インデックス

    for FileName = FileNameList
        FileIDList(1, index) = fopen([Paths.datfile, '/', char(FileName)], "w");
        index = index + 1;
    end
end