% アニメ描画
% 動画，png保存

function Make2dAnime(datfilename, paths, Parameters)
    % パラメータインポート
    BaseWidth = Parameters.BaseWidth;
    BaseDepth = Parameters.BaseDepth;
    BaseMCenter2GCenter = Parameters.BaseMCenter2GCenter(1:2);
    TargetWidth = Parameters.TaegetWidth;
    TargetDepth = Parameters.TargetDepth;
    TargetMCenter2GCenter = Parameters.TargetMCenter2GCenter(1:2);
    LdGamma = Parameters.LdGamma;
    LdH  = Parameters.LdH;
    LdHs = LdH * sin(LdGamma);
    LdD = Parameters.LdD;
    mov_dtime = Parameters.MovDivTime;
    
    % データインポート
    datfile = [paths.datfile, '/', char(datfilename)];
    Data = importdata(datfile, Parameters.Delimiter);
    datanum = size(Data.data(:, :));

    % データヘッダー名からインデックス作成
    BaseCenterPos_Index = FindHeader(Data, 'BasePosX',   Parameters.StringType):FindHeader(Data,'BasePosY',Parameters.StringType);
    BaseOriZ_Index      = FindHeader(Data, 'BaseOriZ',  Parameters.StringType);
    JointPos_Index      = FindHeader(Data, 'JointPos1X',Parameters.StringType):FindHeader(Data,'JointPos8Z', Parameters.StringType);
    EndETipPos_Index= FindHeader(Data, 'ETL1PosX', Parameters.StringType):FindHeader(Data, 'ETR2PosZ', Parameters.StringType);
    EndEfectLeftOriZ_Index   = FindHeader(Data, 'EndLOriZ', Parameters.StringType);
    EndEfectRightOriZ_Index  = FindHeader(Data, 'EndROriZ', Parameters.StringType);
    TargetCenterPos_Index     = FindHeader(Data, 'TargetPosX', Parameters.StringType) : FindHeader(Data, 'TargetPosY', Parameters.StringType);
    TargetOriZ_Index     = FindHeader(Data, 'TargetOriZ', Parameters.StringType);
    
    FigureNumber = 101;     % 図番号設定
    figure(FigureNumber);   % 図定義
    index = 1;              % 画像保存インデックス
    
    % ファイル名定義
    pngfilename = "AnimeFig";   % pngファイル名
    movfilename = "AnimeMov";   % 動画ファイル名

    % 動画用ファイル定義
    FrameRate = 1 / mov_dtime;                                              % フレームレート計算
    video = VideoWriter([paths.movfile, '/', char(movfilename), '.avi']);   % 動画ライター定義
    video.FrameRate = FrameRate;                                            % フレームレート設定
    open(video)                                                             % 動画ライターオープン

    % 描画開始
    for count = 1:datanum(1)
        % アニメ時間
        anime_time = Parameters.DivTime * count;
        
        % アニメ時間が動画刻み時間で割り切れる時に図を描画，動画に保存
        if rem(anime_time, mov_dtime) == 0
            % ロボベース描画
            BaseCenterPos = Data.data(count, BaseCenterPos_Index)';                                       % 重心位置
            BaseOriZ      = Data.data(count, BaseOriZ_Index);                                             % z軸周りの角度ラジアン
            BaseTips = CalcSquareTips(BaseCenterPos + BaseMCenter2GCenter, BaseWidth, BaseDepth, BaseOriZ); % ベース頂点計算
            PolyBase = polyshape(BaseTips(1,:), BaseTips(2,:));                                           % 四角描画
            plot(PolyBase)
            xlim( [ -0.5, 0.5 ] ); 
            ylim( [ -0.2, 0.8 ] );
            daspect([1, 1, 1]);

            hold on
    
            % ロボアーム描画
            % ジョイント
            JointPos = Data.data(count, JointPos_Index); % 関節位置
            JointPos = reshape(JointPos, [3,8]);
            plot(JointPos(1,:),   JointPos(2,:),  'o','MarkerSize',10,'Color','r', 'MarkerFaceColor','r')
            % リンク
            plot(JointPos(1,1:4), JointPos(2,1:4),'-','LineWidth',  4,'Color','b') % 左手リンク線描画
            plot(JointPos(1,5:8), JointPos(2,5:8),'-','LineWidth',  4,'Color','b') % 右手リンク線描画
            % エンドエフェクター
            EndLOriZ = Data.data(count, EndEfectLeftOriZ_Index);                   % データインポート左手先位置
            EndROriZ = Data.data(count, EndEfectRightOriZ_Index);                  % データインポート右手先位置
            EndPos   = zeros([2,8]);
            EndPos(:,1:4) = JointPos(1:2,4) + [ LdH * sin(-EndLOriZ-LdGamma), -LdHs * cos(-EndLOriZ), ... % 左手先端の区の字頂点
                                                LdHs * cos(-EndLOriZ),  LdH * sin(-EndLOriZ+LdGamma); ...
                                                LdH * cos(-EndLOriZ-LdGamma),  LdHs * sin(-EndLOriZ), ...
                                               -LdHs * sin(-EndLOriZ),  LdH * cos(-EndLOriZ+LdGamma)];
            EndPos(:,5:8) = JointPos(1:2,8) + [ LdH * sin(-EndROriZ-LdGamma), -LdHs * cos(-EndROriZ), ... % 右手先端の区の字頂点
                                                LdHs * cos(-EndROriZ),  LdH * sin(-EndROriZ+LdGamma); ...
                                                LdH * cos(-EndROriZ-LdGamma),  LdHs * sin(-EndROriZ), ... 
                                               -LdHs * sin(-EndROriZ),  LdH * cos(-EndROriZ+LdGamma)];
            EndTipsPos = Data.data(count, EndETipPos_Index);
            EndTipsPos = reshape(EndTipsPos, [3, 4]);
            plot(EndPos(1,1:4), EndPos(2,1:4),'-','LineWidth',  4,'Color','b') % 左手区の字描画
            PlotCircle(EndTipsPos(1:2,1), LdD*.5, 'black')                     % 左手先端球描画
            PlotCircle(EndTipsPos(1:2,2), LdD*.5, 'black')                     % 左手先端球描画
            plot(EndPos(1,5:8), EndPos(2,5:8),'-','LineWidth',  4,'Color','b') % 右手区の字描画
            PlotCircle(EndTipsPos(1:2,3), LdD*.5, 'black')                     % 右手先端球描画
            PlotCircle(EndTipsPos(1:2,4), LdD*.5, 'black')                     % 右手先端球描画
    
            % ターゲット描画
            TargetCenterPos = Data.data(count, TargetCenterPos_Index)';                                               % 重心位置
            TargetOriZ      = Data.data(count, TargetOriZ_Index);                                                     % z軸周りの角度ラジアン
            TargetTips = CalcSquareTips(TargetCenterPos + TargetMCenter2GCenter, TargetWidth, TargetDepth, TargetOriZ); % ベース頂点計算
            PolyTarget = polyshape(TargetTips(1,:), TargetTips(2,:));                                                 % 四角描画
            plot(PolyTarget)
            
            hold off

            % 図をVideoWriteに保存
            frame = getframe(figure(FigureNumber));
            writeVideo(video, frame);


            % 図をpng形式で保存
%             PictureName = sprintf('%s%d.png', pngfilename, index);              % png名定義
%             saveas(figure(FigureNumber), [paths.pngfile, '/', PictureName]);    % png保存
%             index = index + 1;
        end
    end

    close(video)                % 動画ライタークローズ
end