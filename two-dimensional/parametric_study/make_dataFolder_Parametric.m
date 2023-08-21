function paths = make_dataFolder_Parametric(param_name, path_prametric)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% フォルダパス指定 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%paths.datfile -> データ保存用
%paths.figfile -> グラフ保存用
%paths.pngfile -> 画像保存用
%paths.movfile -> アニメーション保存用

% フォルダ作成
paths.datfile = [ path_prametric, '/sim_res/', param_name, '/', param_name, '-', 'dat' ];                        % datファイルの名前指定
paths.figfile = [ path_prametric, '/sim_res/', param_name, '/', param_name, '-', 'fig' ];                        % グラフファイルの名前指定
paths.pngfile = [ path_prametric, '/sim_res/', param_name, '/', param_name, '-', 'png' ];                        % 画像ファイルの名前指定
paths.movfile = [ path_prametric, '/sim_res/', param_name, '/', param_name, '-', 'mov' ];                        % 動画ファイルの名前指定

% それぞれのディレクトリを作る命令
mkdir( paths.datfile );
mkdir( paths.figfile ); 
mkdir( paths.pngfile ); 
mkdir( paths.movfile ); 
end