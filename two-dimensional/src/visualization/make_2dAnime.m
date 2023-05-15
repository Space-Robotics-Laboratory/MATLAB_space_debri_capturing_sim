% アニメ描画
% 動画，png保存

function make_2dAnime(datSaver, paths, param)
    % パラメータインポート
    mov_dtime = param.MovDivTime;                              % 動画刻み時間
    dataStruct = datSaver.datStruct;
    
    % force vector scale
    scale = .01;

    % 図定義
    FigureNumber = 101;     % 図番号設定
    figure(FigureNumber);   % 図定義
    picNum = 1;             % 画像保存インデックス
    
    % ファイル名定義
    pngfilename = "AnimeFig";   % pngファイル名
    movfilename = "AnimeMov";   % 動画ファイル名

    % 動画用ファイル定義
    frameRate = 1 / mov_dtime;                                              % フレームレート計算
    video = VideoWriter([paths.movfile, '/', char(movfilename), '.avi']);   % 動画ライター定義
    video.FrameRate = frameRate;                                            % フレームレート設定
    open(video)                                                             % 動画ライターオープン

    % 描画開始
    for count = 1:datSaver.datNum
        % アニメ時間
        anime_time = param.DivTime * count;
        
        % アニメ時間が動画刻み時間で割り切れる時に図を描画，動画に保存
        if rem(anime_time, mov_dtime) == 0
            %%% データインポート
            % robo
            roboR0 = dataStruct.roboR0(count, :)';
            roboQ0 = dataStruct.roboQ0(count, :)';
            jointPos(:, 1:4) = reshape(dataStruct.roboJointLPos(count, :), [3, 4]);
            jointPos(:, 5:8) = reshape(dataStruct.roboJointRPos(count, :), [3, 4]);
            endEffecPos(:, 1:2) = reshape(dataStruct.roboEndEffecLPos(count, :), [3, 2]);
            endEffecPos(:, 3:4) = reshape(dataStruct.roboEndEffecRPos(count, :), [3, 2]);
            endEffecOri(:, 1) = dataStruct.roboEndEffecLOri(count, :)';
            endEffecOri(:, 2) = dataStruct.roboEndEffecROri(count, :)';

            % target 
            targetR0 = dataStruct.targR0(count, :)';
            targetQ0 = dataStruct.targQ0(count, :)';

            % force
            roboForceLeft  = [dataStruct.endTipL1Force(count, :)', dataStruct.endTipL2Force(count, :)'];
            roboForceRight = [dataStruct.endTipR1Force(count, :)', dataStruct.endTipR2Force(count, :)'];
            targetForce = dataStruct.targForce(count, :)';

            % 手先目標位置
            desHandPos = reshape(dataStruct.desHandPos(count, :), [3, 2]);


            %%% 描画
            % ロボ描画
            vis_DualArmRobot(roboR0, roboQ0, jointPos, endEffecPos, endEffecOri, param)
            hold on

            % ターゲット描画
            vis_Target(targetR0, targetQ0, param)

            % ロボ外力描画
            vis_roboForce(endEffecPos, roboForceLeft, roboForceRight, scale)

            % ターゲット外力描画
            vis_targetForce(targetR0, targetForce, scale)

            % 手先目標位置描画
            vis_desiredPos(desHandPos)
            
            hold off

            
            %%% 結果保存
            % 図をVideoWriteに保存
            frame = getframe(figure(FigureNumber));
            writeVideo(video, frame);


            % 図をpng形式で保存
            if rem(count, 500) == 0
                pictureName = sprintf('%s%d.png', pngfilename, picNum);              % png名定義
                saveas(figure(FigureNumber), [paths.pngfile, '/', pictureName]);     % png保存
                picNum = picNum + 1;
            end
        end
    end

    close(video)                % 動画ライタークローズ
end