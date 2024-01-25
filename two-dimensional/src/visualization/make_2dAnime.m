% アニメ描画
% 動画，png保存

function make_2dAnime(datSaver, paths, param)
    % パラメータインポート
    dataStruct = datSaver.datStruct;
    dt_sim = param.general.divTime;
    frameRate = param.general.anime_frameRate; 
    snapShotRate = param.general.snapShot_frameRate;
    
    % force vector scale
    scale = .01;

    % 図定義
    FigureNumber = 101;     % 図番号設定
    fig = figure(FigureNumber);   % 図定義
    fontSize = 22;  % 目盛りのフォントサイズ
    picNum = 1;             % 画像保存インデックス
    
    % ファイル名定義
    pngfilename = "AnimeFig";   % pngファイル名
    movfilename = "AnimeMov";   % 動画ファイル名

    % 動画用ファイル定義
    video = VideoWriter([paths.movfile, '/', char(movfilename), '.avi']);   % 動画ライター定義
    video.FrameRate = frameRate;                                            % フレームレート設定
    open(video)                                                             % 動画ライターオープン

    % 描画開始
    anime_counter = 0;
    for count = 1:datSaver.timer_length
        % アニメ時間ディジタル化
        mov_d_count = fix( 1/ (frameRate * dt_sim) );
        
        % アニメ時間が動画刻み時間で割り切れる時に図を描画，動画に保存
        if rem(count, mov_d_count) == 1
            anime_counter = anime_counter + 1;
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
            set(gca, 'FontSize', fontSize);  % 軸目盛りのフォントサイズを設定
            xlabel('$$ \it{x} \space \rm{[m]} $$', 'Interpreter','latex');
            ylabel('$$ \it{y} \space \rm{[m]} $$', 'Interpreter','latex');
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
            if anime_counter == 1
                frameSize0 = size(frame.cdata);
            end
            frameSize = size(frame.cdata);
            if all(frameSize == frameSize0)
                writeVideo(video, frame);
            else
                disp("failed to save animation")
                break
            end

            % 図をpng形式で保存
            if rem(count, snapShotRate) == 1
                pictureName = sprintf('%s%d.png', pngfilename, picNum);              % png名定義
                % set(fig,'Units','Inches');
                % pos = get(fig,'Position');
                % set(fig,'PaperPositionMode','manual','PaperUnits','Inches','PaperSize',[0.01, pos(4)])
                % 
                % margin = -5;  % 追加の余白サイズを調整（必要に応じて適切な値に変更）
                % paperPos = [margin, margin, pos(3)-2*margin, pos(4)-2*margin];
                % set(gca, 'LooseInset', get(gca, 'TightInset'));
                % 
                % print(fig,[paths.pngfile, '/', 'tny_', pictureName],'-dpng','-r0')
                saveas(fig, [paths.pngfile, '/', pictureName]);     % png保存
                picNum = picNum + 1;
            end
        end
    end

    close(video)                % 動画ライタークローズ
end