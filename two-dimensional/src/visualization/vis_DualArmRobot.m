% 双腕ロボット描画関数 
% 
% 2023.2 akiyoshi uchida
%
% input: R(3*1) Q(3*1), jointP(3*8), enEfP(3*4), enEfQ(3*2), param


function vis_DualArmRobot(roboR0, roboQ0, jointPos, endEffecPos, endEffecOri, param)
    baseWidth = param.robot.baseWidth;
    baseDepth = param.robot.baseDepth;
    basePos = roboR0;
    LdD = param.robot.diameter_endTip;
    gamma = param.robot.endEffector_gamma;
    LdH = param.robot.endEffector_h;
    
    cornerRotMatL = rpy2dc(endEffecOri(:, 1));
    cornerRotMatR = rpy2dc(endEffecOri(:, 2));
    cornerVecL = cornerRotMatL' * [0, -LdH * cos(gamma), 0]';
    cornerVecR = cornerRotMatR' * [0, -LdH * cos(gamma), 0]';

    % ロボベース描画
    baseTips = calc_SquareTips(basePos(1:2,1), baseWidth, baseDepth, roboQ0(3));    % ベース頂点計算
    polyBase = polyshape(baseTips(1,:), baseTips(2,:));                             % 四角描画
    plot(polyBase)
    hold on

    % 図設定
    xlim( [ -0.5, 0.5 ] ); 
    ylim( [ -0.2, 0.55 ] );
    xticks( -0.5:0.1:0.5 );
    yticks( -0.2:0.1:0.8 );
    daspect([1, 1, 1]);

    grid on


    % ロボアーム描画
    % ジョイント
    jointPos = reshape(jointPos, [3,8]);
    if(param.robot.compliantWrist)
        plot(jointPos(1,:),   jointPos(2,:),  'o','MarkerSize',10,'Color','r', 'MarkerFaceColor','r')
    else
        plot(jointPos(1,[1,2,3,5,6,7]), jointPos(2,[1,2,3,5,6,7]), 'o','MarkerSize',10,'Color','r', 'MarkerFaceColor','r')
    end
    
    % リンク
    plot(jointPos(1,1:4), jointPos(2,1:4),'-','LineWidth',  4,'Color','b') % 左手リンク線描画
    plot(jointPos(1,5:8), jointPos(2,5:8),'-','LineWidth',  4,'Color','b') % 右手リンク線描画

    % エンドエフェクター 
    endEffecCornerL = [endEffecPos(:, 1), endEffecPos(:, 1:2) + cornerVecL, endEffecPos(:, 2)]; % 区の字座標 3*4
    endEffecCornerR = [endEffecPos(:, 3), endEffecPos(:, 3:4) + cornerVecR, endEffecPos(:, 4)]; % 区の字座標 3*4
  
    plot(endEffecCornerL(1,:), endEffecCornerL(2,:),'-','LineWidth',  4,'Color','b')    % 左手区の字描画
    plot_circle(endEffecPos(1:2,1), LdD*.5, 'black')                                    % 左手先端球描画
    plot_circle(endEffecPos(1:2,2), LdD*.5, 'black')                                    % 左手先端球描画

    plot(endEffecCornerR(1,:), endEffecCornerR(2,:),'-','LineWidth',  4,'Color','b')    % 右手区の字描画
    plot_circle(endEffecPos(1:2,3), LdD*.5, 'black')                                    % 右手先端球描画
    plot_circle(endEffecPos(1:2,4), LdD*.5, 'black')                                    % 右手先端球描画
