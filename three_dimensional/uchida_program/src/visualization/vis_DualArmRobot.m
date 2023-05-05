% 双腕ロボット描画関数 
% 
% 2023.2 akiyoshi uchida
%
% input: R(3*1) Q(3*1), jointP(3*8), enEfP(3*4), enEfQ(3*2), param


function vis_DualArmRobot(roboR0, roboQ0, jointPos, endEffecPos, endEffecOri_rpy, param)
    baseWidth = param.robot.baseWidth;
    baseDepth = param.robot.baseDepth;
    baseHeight = param.robot.baseHeight;
    baseORI = rpy2dc(roboQ0)';
    m2g = 0;
    basePos = roboR0 + m2g;
    L = param.robot.length_endEffector;
    r = param.robot.diameter_endTip*.5;
    
    cornerRotMatL = rpy2dc(endEffecOri_rpy(:, 1))';
    cornerRotMatR = rpy2dc(endEffecOri_rpy(:, 2))';

    endEffecCornerVecL = cornerRotMatL * [0, -L, 0]';
    endEffecCornerVecR = cornerRotMatR * [0, -L, 0]';
    cornerL = endEffecPos(:, 1:3) + endEffecCornerVecL;
    cornerR = endEffecPos(:, 4:6) + endEffecCornerVecR;

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
    jointPos = reshape(jointPos, [3,12]);
    plot3(jointPos(1,:),   jointPos(2,:), jointPos(3,:), 'o','MarkerSize',10,'Color','r', 'MarkerFaceColor','r')
    % リンク
    plot3(jointPos(1,1:6), jointPos(2,1:6),jointPos(3,1:6),'-','LineWidth',  4,'Color','b') % 左手リンク線描画
    plot3(jointPos(1,7:12), jointPos(2,7:12),jointPos(3,7:12),'-','LineWidth',  4,'Color','b') % 右手リンク線描画
    % エンドエフェクター 
    endEffecCornerL = [endEffecPos(:, 1), cornerL(:, 1), jointPos(:, 6) , cornerL(:, 2), endEffecPos(:, 2)]; % 区の字座標 3*5
    endEffecCornerL_rest = [endEffecPos(:, 3), cornerL(:, 3), jointPos(:, 6)];                               % 区の字座標 3*3
    endEffecCornerR = [endEffecPos(:, 4), cornerR(:, 1), jointPos(:, 12), cornerR(:, 2), endEffecPos(:, 5)]; % 区の字座標 3*4
    endEffecCornerR_rest = [endEffecPos(:, 6), cornerR(:, 3), jointPos(:, 12)];                              % 区の字座標 3*3
  
    plot3(endEffecCornerL(1,:), endEffecCornerL(2,:), endEffecCornerL(3,:), '-','LineWidth',  4,'Color','b')    % 左手区の字描画
    plot3(endEffecCornerL_rest(1,:), endEffecCornerL_rest(2,:), endEffecCornerL_rest(3,:), '-','LineWidth',  4,'Color','b')

    plot_sphere(param.general.sphereParticles, endEffecPos(:,1), r, [1, 1, 1])          % 左手先端球描画
    plot_sphere(param.general.sphereParticles, endEffecPos(:,2), r, [1, 1, 1])          % 左手先端球描画
    plot_sphere(param.general.sphereParticles, endEffecPos(:,3), r, [1, 1, 1])          % 左手先端球描画

    plot3(endEffecCornerR(1,:), endEffecCornerR(2,:), endEffecCornerR(3,:),'-','LineWidth',  4,'Color','b')    % 右手区の字描画
    plot3(endEffecCornerR_rest(1,:), endEffecCornerR_rest(2,:), endEffecCornerR_rest(3,:), '-','LineWidth',  4,'Color','b')
    plot_sphere(param.general.sphereParticles, endEffecPos(:,4), r, [1, 1, 1])          % 右手先端球描画
    plot_sphere(param.general.sphereParticles, endEffecPos(:,5), r, [1, 1, 1])          % 右手先端球描画
    plot_sphere(param.general.sphereParticles, endEffecPos(:,6), r, [1, 1, 1])          % 右手先端球描画
