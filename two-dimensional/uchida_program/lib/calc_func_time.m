% 関数実行時間計算関数
% n回フォーループを回す
% 今のとこ引数１のやつだけ．そのうち改良する．
function calc_func_time(func, arg, n)

% タイマースタート                                               
StartCPUT = cputime;
StartT = clock();

for i = 1:n
    func(arg);
end

% シミュレーション全体時間 単位:秒
ntime = cputime - StartCPUT;

nhour = floor( ntime / 3600 );                    % 単位:時間 各要素以下の最も近い整数に丸める
nmin  = floor( ( ntime - nhour * 3600 ) / 60 );   % 単位:分 残りの分，整数に丸める
nsec  = ntime - nhour * 3600 - nmin * 60;         % 単位:秒 残りの秒，整数に丸める

% 結果表示
fprintf( '\n\n %s %s', '開始時間 :', datestr( StartT, 31 ) );
fprintf( '\n %s %s',   '終了時間 :', datestr( clock,  31 ) );
fprintf( '\n %s %d %s %02d %s %04.1f %s \n\n\n', '計算所要時間 :', nhour, ' 時間 ', nmin, ' 分 ', nsec, ' 秒 ' );