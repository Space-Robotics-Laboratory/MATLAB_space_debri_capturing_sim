% 双腕ロボット描画関数 
% 
% 2023.2 akiyoshi uchida
%
% input: R(3*1) Q(3*1), jointP(3*8), enEfP(3*4), enEfQ(3*2), param


function vis_DualArmRobot(roboR0, roboQ0, jointPos, endEffecPos, endEffecOri_rpy, param)
    baseWidth = param.BaseWidth;
    baseDepth = param.BaseDepth;
    baseHeight = param.BaseHeight;
    baseORI = rpy2dc(roboQ0)';
    m2g = 0;
    basePos = roboR0 + m2g;
    LdD = param.LdD;
    r = LdD*.5;
    
    cornerRotMatL = rpy2dc(endEffecOri_rpy(:, 1))';
    cornerRotMatR = rpy2dc(endEffecOri_rpy(:, 2))';
    cornerVecL = cornerRotMatL * [0, -param.LdH * cos(param.LdGamma), 0]';
    cornerVecR = cornerRotMatR * [0, -param.LdH * cos(param.LdGamma), 0]';

    % ロボベース描画
    [baseVertives, baseFace] = calc_cubic(basePos, baseWidth, baseDepth, baseHeight, baseORI);
    patch('Vertices',baseVertives,'Faces',baseFace, 'FaceColor', param.general.bodyColor)
    hold on

    % 図設定
    xlim( [ -0.5, 0.5 ] ); 
    ylim( [ -0.2, 0.8 ] );
    zlim( [ -0.3, 0.3 ] );
    xticks( -0.5:0.1:0.5 );
    yticks( -0.2:0.1:0.8 );
    zticks( -0.3:0.1:0.3 );
    daspect([1, 1, 1]);

    grid on


    % ロボアーム描画
    % ジョイント
    jointPos = reshape(jointPos, [3,8]);
    plot3(jointPos(1,:),   jointPos(2,:), jointPos(3,:), 'o','MarkerSize',10,'Color','r', 'MarkerFaceColor','r')
    % リンク
    plot3(jointPos(1,1:4), jointPos(2,1:4),jointPos(3,1:4),'-','LineWidth',  4,'Color','b') % 左手リンク線描画
    plot3(jointPos(1,5:8), jointPos(2,5:8),jointPos(3,5:8),'-','LineWidth',  4,'Color','b') % 右手リンク線描画
    % エンドエフェクター 
    endEffecCornerL = [endEffecPos(:, 1), endEffecPos(:, 1:2) + cornerVecL, endEffecPos(:, 2)]; % 区の字座標 3*4
    endEffecCornerR = [endEffecPos(:, 3), endEffecPos(:, 3:4) + cornerVecR, endEffecPos(:, 4)]; % 区の字座標 3*4
  
    plot3(endEffecCornerL(1,:), endEffecCornerL(2,:), endEffecCornerL(3,:), '-','LineWidth',  4,'Color','b')    % 左手区の字描画
    plot_sphere(param.general.sphereParticles, endEffecPos(:,1), r, [1, 1, 1])          % 左手先端球描画
    plot_sphere(param.general.sphereParticles, endEffecPos(:,2), r, [1, 1, 1])          % 左手先端球描画

    plot3(endEffecCornerR(1,:), endEffecCornerR(2,:), endEffecCornerR(3,:),'-','LineWidth',  4,'Color','b')    % 右手区の字描画
    plot_sphere(param.general.sphereParticles, endEffecPos(:,3), r, [1, 1, 1])          % 右手先端球描画
    plot_sphere(param.general.sphereParticles, endEffecPos(:,4), r, [1, 1, 1])          % 右手先端球描画
