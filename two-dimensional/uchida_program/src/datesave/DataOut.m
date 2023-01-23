%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%データ書き出し用関数
%パスの場所に，メンバー種類毎の列，時系列の行で保存．
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function DataOut(FileID, Values, ValueType, Delimiter)
    NewLine   = '\n';   %改行文字

    Num = size(Values);
    Line = [repmat([ValueType, Delimiter], Num), NewLine];
    fprintf(FileID, Line, Values);

end