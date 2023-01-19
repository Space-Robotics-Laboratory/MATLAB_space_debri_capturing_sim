function paths = make_DataFolder(Parameters)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% フォルダパス指定 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%paths.datfile -> データ保存用
%paths.figfile -> グラフ保存用
%paths.pngfile -> 画像保存用
%paths.movfile -> アニメーション保存用

% フォルダ作成
param_name = Parameters.FileName;               % ファイル名設定
timepath = datestr( now, 'yyyy-mmdd-HHMMSS' );  % yはyear，mはmonth,dはday,Hはhour,Mはminute,Sはsecond.それぞれの文字数分出力する
datepath = datestr( now, 'yyyy-mmdd' );
path_temp = [ Parameters.DateSavePath '/' datepath '/' timepath ];                 % データ保存のディレクトリを作る ;          
paths.datfile = [ path_temp, '/', param_name, '-', 'dat' ];                        % datファイルの名前指定
paths.figfile = [ path_temp, '/', param_name, '-', 'fig' ];                        % グラフファイルの名前指定
paths.pngfile = [ path_temp, '/', param_name, '-', 'png' ];                        % 画像ファイルの名前指定
paths.movfile = [ path_temp, '/', param_name, '-', 'mov' ];                        % 動画ファイルの名前指定

% それぞれのディレクトリを作る命令
mkdir( paths.datfile );
mkdir( paths.figfile ); 
mkdir( paths.pngfile ); 
mkdir( paths.movfile ); 
end