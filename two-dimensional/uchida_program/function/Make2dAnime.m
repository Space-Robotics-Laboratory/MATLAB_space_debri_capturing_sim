%アニメ描画，保存

function Make2dAnime(datfile, Parameters)
    % パラメータインポート
    BaseWidth = Parameters.BaseWidth;
    BaseDepth = Parameters.BaseDepth;
    GCenter2MCenter = Parameters.GCenter2MCenter(1:2);
    
    % データインポート
    Data = importdata(datfile, Parameters.Delimiter);
    datanum = size(Data.data(:, :));
    
    figure(101)
    % 描画開始
    for count = 1:datanum(1)
        % ロボベース描画
        BaseCenterPos = Data.data(count, 1:2)';  % 重心位置
        BaseZOri      = Data.data(count, 6);     % z軸周りの角度ラジアン
        Tips = CalcTipsPos(BaseCenterPos - GCenter2MCenter, BaseWidth, BaseDepth, BaseZOri); % ベース頂点計算
        PolyBase = polyshape(Tips(1,:), Tips(2,:));
        plot(PolyBase)
        xlim( [ -0.5, 0.5 ] ); 
        ylim( [ -0.2, 0.8 ] );
        daspect([1, 1, 1]);
        hold on

        % ロボアーム描画
        % ジョイント
        JointPos = Data.data(count, 7:30);
        JointPos = reshape(JointPos, [3,8]);
        plot(JointPos(1,:),   JointPos(2,:),  'o','MarkerSize',10,'Color','r', 'MarkerFaceColor','r')
        % リンク
        plot(JointPos(1,1:4), JointPos(2,1:4),'-','LineWidth',  4,'Color','b') % 左手
        plot(JointPos(1,5:8), JointPos(2,5:8),'-','LineWidth',  4,'Color','b') % 右手
        % エンドエフェクター
        EndEfectPos = [JointPos(:,4); JointPos(:,8)];
        
        plot(Jopi)
        
        hold off
        pause(0.01)
    end
end