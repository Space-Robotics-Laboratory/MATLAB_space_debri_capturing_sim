%アニメ描画，保存

function Make2dAnime(datfile, Parameters)
    % パラメータインポート
    BaseWidth = Parameters.BaseWidth;
    BaseDepth = Parameters.BaseDepth;
    GCenter2MCenter = Parameters.GCenter2MCenter(1:2);
    LdGamma = Parameters.LdGamma;
    LdH  = Parameters.LdH;
    LdHs = LdH * sin(LdGamma);
    LdD = Parameters.LdD;
    
    % データインポート
    Data = importdata(datfile, Parameters.Delimiter);
    datanum = size(Data.data(:, :));
    % インデックス作成
    BaseCenterPos_Index = FindHeader(Data, 'BasePosX',   Parameters.StringType):FindHeader(Data,'BasePosY',Parameters.StringType);
    BaseOriZ_Index      = FindHeader(Data, 'BaseOriZ',  Parameters.StringType);
    JointPos_Index      = FindHeader(Data, 'JointPos1X',Parameters.StringType):FindHeader(Data,'JointPos8Z', Parameters.StringType);
    EndETipPOS_Index= FindHeader(Data, 'ETL1PosX', Parameters.StringType):FindHeader(Data, 'ETR2PosZ', Parameters.StringType);
    EndEfectLeftOriZ_Index   = FindHeader(Data, 'EndLOriZ', Parameters.StringType);
    EndEfectRightOriZ_Index  = FindHeader(Data, 'EndROriZ', Parameters.StringType);
    
    figure(101)
    % 描画開始
    for count = 1:datanum(1)
        % ロボベース描画
        BaseCenterPos = Data.data(count, BaseCenterPos_Index)';  % 重心位置
        BaseOriZ      = Data.data(count, BaseOriZ_Index);        % z軸周りの角度ラジアン
        Tips = CalcBaseTips(BaseCenterPos - GCenter2MCenter, BaseWidth, BaseDepth, BaseOriZ); % ベース頂点計算
        PolyBase = polyshape(Tips(1,:), Tips(2,:));              % 四角描画
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
        EndLOriZ = Data.data(count, EndEfectLeftOriZ_Index);
        EndROriZ = Data.data(count, EndEfectRightOriZ_Index);
        EndPos   = zeros([2,8]);
        EndPos(:,1:4) = JointPos(1:2,4) + [ LdH * sin(-EndLOriZ-LdGamma), -LdHs * cos(-EndLOriZ), ... % 左手先端区の字頂点
                                            LdHs * cos(-EndLOriZ),  LdH * sin(-EndLOriZ+LdGamma); ...
                                            LdH * cos(-EndLOriZ-LdGamma),  LdHs * sin(-EndLOriZ), ...
                                           -LdHs * sin(-EndLOriZ),  LdH * cos(-EndLOriZ+LdGamma)];
        EndPos(:,5:8) = JointPos(1:2,8) + [ LdH * sin(-EndROriZ-LdGamma), -LdHs * cos(-EndROriZ), ... % 右手先端区の字頂点
                                            LdHs * cos(-EndROriZ),  LdH * sin(-EndROriZ+LdGamma); ...
                                            LdH * cos(-EndROriZ-LdGamma),  LdHs * sin(-EndROriZ), ... 
                                           -LdHs * sin(-EndROriZ),  LdH * cos(-EndROriZ+LdGamma)];
        EndTipsPos = Data.data(count, EndETipPOS_Index);
        EndTipsPos = reshape(EndTipsPos, [3, 4]);
        plot(EndPos(1,1:4), EndPos(2,1:4),'-','LineWidth',  4,'Color','b') % 左手区の字描画
        PlotCircle(EndTipsPos(1:2,1), LdD*.5, 'black')                       % 左手先端球描画
        PlotCircle(EndTipsPos(1:2,2), LdD*.5, 'black')                       % 左手先端球描画
        plot(EndPos(1,5:8), EndPos(2,5:8),'-','LineWidth',  4,'Color','b') % 右手区の字描画
        PlotCircle(EndTipsPos(1:2,3), LdD*.5, 'black')                       % 右手先端球描画
        PlotCircle(EndTipsPos(1:2,4), LdD*.5, 'black')                       % 右手先端球描画
        
        hold off
        pause(0.01)
    end
end