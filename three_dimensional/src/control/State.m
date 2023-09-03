% 接触状況，ターゲット運動状況を保管するクラス．シミュレーションのみで得られる真知であることに注意する
%
% 2023.4 Akiyoshi Uchida
%

classdef State
    properties
        isCapture       % bool ターゲット捕獲状況
        isContact       % bool 1*6 各手先の現時刻接触状況
        wasContact      % bool 1*6各手先の前時刻接触状況
        newContact      % bool 1*6 新たに接触したかどうか
        endContact      % bool 1*6 新たに非接触になったか
        targetSlow      % bool 現時刻でターゲットの角速度がしきい値より小さいか
        comeTargetSlow  % bool 新たにターゲットの角速度がしきい値より小さくなったかどうか
        time            % 各時刻を保存する
    end     
    methods
        % コンストラクター
        function obj = State()
            obj.isCapture  = false;
            obj.isContact  = false(1, 6);
            obj.wasContact = false(1, 6);
            obj.newContact = false(1, 6);
            obj.endContact = false(1, 6);
            obj.targetSlow = false;
            obj.comeTargetSlow = false;
            obj.time.lastContact = inf;
            obj.time.newContact = inf;
            obj.time.comeTargetSlow = inf;
        end
        % 状態更新
        function obj = update(obj, robo, isContact, target, time, param)
            obj.isCapture = judge_isCaptured(robo, target, param);
            obj.wasContact = obj.isContact;
            obj.isContact = isContact;
            obj.newContact = ~obj.wasContact & obj.isContact;
            obj.endContact = obj.wasContact & ~obj.isContact;
            obj.comeTargetSlow = ~obj.targetSlow && abs(target.SV.w0(3))<param.control.switchingTargetAngVel;
            if obj.comeTargetSlow && obj.time.comeTargetSlow == inf
                % 初めて角速度の境界を割った時刻．一度しか更新されない．
                obj.time.comeTargetSlow = time;
            end
            obj.targetSlow = abs(target.SV.w0(3)) <= param.control.switchingTargetAngVel;
            if any(obj.isContact)
                obj.time.lastContact = time;
            end
            if any(obj.newContact)
                obj.time.newContact = time;
            end
        end
    end
end