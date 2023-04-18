% 接触状況，ターゲット運動状況を保管するクラス．シミュレーションのみで得られる真知であることに注意する
%
% 2023.4 Akiyoshi Uchida
%

classdef State
    properties
        isCapture       % bool ターゲット捕獲状況
        isContact       % bool 1*4 各手先の現時刻接触状況
        wasContact      % bool 1*4各手先の前時刻接触状況
        newContact      % bool 1*4 新たに接触したかどうか
        endContact      % bool 1*4 新たに非接触になったか
        targetSlow      % bool 現時刻でターゲットの角速度がしきい値より小さいか
        comeTargetSlow  % bool 新たにターゲットの角速度がしきい値より小さくなったかどうか
        time            % 各時刻を保存する
    end     
    methods
        % コンストラクター
        function obj = State()
            obj.isCapture = false;
            obj.isContact = false;
            obj.wasContact = false;
            obj.newContact = false;
            obj.endContact = false;
            obj.targetSlow = false;
            obj.comeTargetSlow = false;
            obj.time.lastContact = inf;
            obj.time.newContact = inf;
            obj.time.comeTargetSlow = inf;
        end
        % 状態更新
        function obj = update(obj, robo, isContact, target, time, param)
            obj.isCapture = judge_IsCapture(robo, target, param);
            obj.wasContact = obj.isContact;
            obj.isContact = isContact;
            obj.newContact = ~obj.wasContact & obj.isContact;
            obj.endContact = obj.wasContact & ~obj.isContact;
            obj.comeTargetSlow = ~obj.targetSlow && abs(target.SV.w0(3))<param.AngularVelBorder;
            if obj.comeTargetSlow && obj.time.comeTargetSlow == inf
                % 初めて角速度の境界を割った時刻．一度しか更新されない．
                obj.time.comeTargetSlow = time;
            end
            obj.targetSlow = abs(target.SV.w0(3)) <= param.AngularVelBorder;
            if any(obj.isContact)
                obj.time.lastContact = time;
            end
            if any(obj.newContact)
                obj.time.newContact = time;
            end
        end
    end
end