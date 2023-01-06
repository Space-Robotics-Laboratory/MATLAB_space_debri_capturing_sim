%importdatで読み込んだファイルから，ヘッダー名の列を検索する

function HeaderCol = FindHeader(Data, HeaderName,type)
    Header = sprintf(type, HeaderName);
    HeaderCol = find(strncmp(Header, Data.colheaders, 10));
    if isempty(HeaderCol)
        error("can not find hedder name %s",HeaderName);
    end 
end