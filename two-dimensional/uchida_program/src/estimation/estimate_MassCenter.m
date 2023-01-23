% ２点観測位置の座標から剛体の重心，速度，角速度を推定する
% 修論再現のためだけのプログラム．終わったら使わない．
% 外力ゼロの場合のみまずは作る．外力ありは知らん．
% 観測点の誤差はないとしてまずやる
% まず２次元の条件でやる
% Q0: ２つの観測点の差分ベクトルの時刻０での値．
% ContactFlag: 1*4, 直方体の確変の接触状態を保持．接触＝１，非接触＝０
% ObesrvedPoints [x1(t) x2(t)
%                 y1(t) y2(t)]

function MassCenter = EstimateMassCenter(ContactFlag, ObservedPoints_t1, ObservedPoints_t2, Q0)
global d_time

    % ContactFlagが全て0の時のみ非接触であるので，推定を計算する
    if ~all(~ContactFlag)
        %contacat
    else
        % not contact
        T2S_1 = ObservedPoints_t1(:,2)-ObservedPoints_t1(:,1);      % 時刻t1における観測点の位置ベクトルの差
        T2S_2 = ObservedPoints_t2(:,2)-ObservedPoints_t2(:,1);      % 時刻t2における観測点の位置ベクトルの差
%        theta_vec_temp = cross( T2S_1, T2S_2);                      % 時刻t1->t2における回転軸 
%        theta_vec = theta_vec_temp / norm(theta_vec_temp);          % 上回転軸の正規化（ベクトルサイズ1）
        theta     = subspace(T2S_1, T2S_2);                         % 上回転軸に対する回転角，二次元ならそのまま回転角
%        R = CalcRotate_from_theta(theta * theta_vec);               % 回転行列の計算
%        MassCenter.w = dc2rpy( R / d_time ) ;                       % 角運動量推定，オイラー角
        MassCenter.w   = [0 0 theta/d_time];                        % 角速度推定
        MassCenter.Qz = acos((T2S_2' * Q0) / (norm(Q0) * norm(T2S_2)));   % 推定角度
    end
end