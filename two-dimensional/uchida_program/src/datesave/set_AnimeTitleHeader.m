% アニメーション用データファイルヘッダー設定用関数

function AnimeTitle = set_AnimeTitleHeader()
    AnimeTitle  = ["BasePosX","BasePosY","BasePosZ","BaseOriX","BaseOriY","BaseOriZ"];
    for j = 1:8
        for s = ["X", "Y", "Z"]
            AnimeTitle = [AnimeTitle, sprintf("JointPos%d%s",j,s)]; %#ok<AGROW> 
        end
    end
    for LR = ["L", "R"]
        for tip = 1:2
            for s = ["X", "Y", "Z"]
                AnimeTitle = [AnimeTitle, sprintf("ET%s%dPos%s",LR,tip,s)]; %#ok<AGROW> 
            end
        end
    end
    for LR = ["L", "R"]
        for s = ["X", "Y", "Z"]
            AnimeTitle = [AnimeTitle, sprintf("End%sOri%s",LR,s)]; %#ok<AGROW> 
        end
    end
    AnimeTitle = [AnimeTitle, "TargetPosX", "TargetPosY", "TargetPosZ", "TargetOriX", "TargetOriY", "TargetOriZ"];
end