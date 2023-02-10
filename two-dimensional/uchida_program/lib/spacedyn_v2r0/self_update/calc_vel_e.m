% 関節番号で指定した手先（双腕ロボの二股エンドエフェクターでは二つの端球の中点）の速度を求める関数
% 先端リンクの重心速度 -> 手先速度
% 
% 2023.2 Akiyoshi Uchida
%
% 

function VEL_e = calc_vel_e( LP, SV, joints )

n = length(joints);             % 関節数
k = joints(n);                  % 末端リンク
A_i_EE = rpy2dc(LP.Qe(:,k))';   % 手先重心から手先への方向余弦行列

VEL_e = SV.vv(:, k) ...
        + cross( SV.ww(:, k),(A_i_EE*LP.ce(:,k)) );

%%%EOF
