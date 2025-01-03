% 接触状況，ターゲット運動状況を保管するクラス．シミュレーションのみで得られる真知であることに注意する
%
% 2023.4 Akiyoshi Uchida
%

classdef State
    properties
        isDetumbling        % bool ディタンブリングシーケンスかどうか
        tryCapturing           % bool ケージングシーケンスかどうか
        isCapture           % bool ターゲット捕獲状況
        hasCautured         % bool 捕獲成功体験
        isContact            % bool 1*4 各手先の現時刻接触状況
        isBaseContact       % bool ベースとターゲットの接触
        isPinch             % bool 挟み込み
        force_holder_       % isPinch用. -> この辺要改善
        wasContact          % bool 1*4各手先の前時刻接触状況
        newContact          % bool 1*4 新たに接触したかどうか
        endContact          % bool 1*4 新たに非接触になったか
        hasBeyondMaxContactForce % bool 設定した接触力を超えたか
        hasAccidentalContact   % bool 1*4 予期せぬ接触かどうか
        targetSlow          % bool 現時刻でターゲットの角速度がしきい値より小さいか
        targetStop          % bool ターゲットがロボットに対して停止(しきい値以下)
        goneAway            % bool ターゲットを突き飛ばしたか
        comeTargetSlow      % bool 新たにターゲットの角速度がしきい値より小さくなったかどうか
        time                % 各時刻を保存する
    end     
    methods
        % コンストラクター
        function obj = State()
            obj.isDetumbling = false;
            obj.tryCapturing = false;
            obj.isCapture = false;
            obj.hasCautured = false;
            obj.isContact = false;
            obj.isBaseContact = false;
            obj.force_holder_ = zeros(1, 4);
            obj.isPinch = false;
            obj.wasContact = false;
            obj.newContact = false;
            obj.endContact = false;
            obj.hasAccidentalContact = false;
            obj.hasBeyondMaxContactForce = false;
            obj.targetSlow = false;
            obj.targetStop = false;
            obj.comeTargetSlow = false;
            obj.goneAway = false;
            obj.time.lastContact = inf;
            obj.time.newContact = inf;
            obj.time.comeTargetSlow = inf;
        end
        % 状態更新
        function obj = update(obj, controller, robo, isContact, target, time, param)
            obj.isCapture = judge_isCaptured(robo, target, param);
            obj.hasCautured = obj.hasCautured || obj.isCapture;
            obj.isDetumbling = controller.sequenceMode == "detumbling" && ~obj.hasCautured;
            obj.tryCapturing = controller.sequenceMode == "capturing" && ~obj.hasCautured;
            obj.wasContact = obj.isContact;
            obj.isContact = isContact;
            obj.isBaseContact = judge_baseContact(robo, target, param);
            obj.hasBeyondMaxContactForce = obj.hasBeyondMaxContactForce || any(vecnorm(robo.SV.Fe(1:2, :)) >= param.control.maxContactForce);

            % ここだけ不自然．要改善
            [obj.isPinch, obj.force_holder_] = judge_isPinched(obj.force_holder_, robo, param);

            obj.newContact = ~obj.wasContact & obj.isContact;
            obj.endContact = obj.wasContact & ~obj.isContact;
            obj.hasAccidentalContact = obj.hasAccidentalContact || any((controller.phase ~= -1) & obj.isContact);
            obj.comeTargetSlow = ~obj.targetSlow && abs(target.SV.w0(3))<param.control.switchingTargetAngVel;
            if obj.comeTargetSlow && obj.time.comeTargetSlow == inf
                % 初めて角速度の境界を割った時刻．一度しか更新されない．
                obj.time.comeTargetSlow = time;
            end
            obj.targetSlow = abs(target.SV.w0(3)) <= param.control.switchingTargetAngVel;
            [~, target_vb] = tf_s2b([target.SV.R0(1:2, 1);target.SV.Q0(3)], [target.SV.v0(1:2, 1);target.SV.w0(3)], robo);
            obj.targetStop = abs(target_vb(3)) <= param.control.stoppingTargetAngVel && ...
                             vecnorm(target_vb(1:2))   <= param.control.stoppingTargetVel;
            obj.goneAway = vecnorm(target.SV.R0 - robo.SV.R0) >= param.control.reachable;
            if any(obj.isContact)
                obj.time.lastContact = time;
            end
            if any(obj.newContact)
                obj.time.newContact = time;
            end
        end
    end
end