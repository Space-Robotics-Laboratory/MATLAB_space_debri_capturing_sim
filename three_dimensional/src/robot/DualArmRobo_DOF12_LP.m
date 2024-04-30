function LP = DualArmRobo_DOF12_LP(param)  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% 各リンクパラメータの定義 %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%% リンクの連結関係を定義する %%%%%
% LPは各リンクの初期条件を記述している  LPをデータで渡すときに様々なデータを渡せるように構造体にしている  BB(i) = 根元側で結合しているリンク番号
LP.BB = [ 0 1 2 3 4 5   0 7 8 9 10 11 ];   % ベース(リンク0)からリンク1が生えている  このときリンク1から見た根本側とはベースへの向きのことであるので0が一番左に代入されている

% リンク数
LP.num_q = length( LP.BB );   % lengthは行数と列数の内の大きい方を与える

%%%%% リンクの連結関係を定義する その２ %%%%%
% 対角成分はすべて-1  各列が左からそれぞれリンク1,2を表している  根元側で結合しているリンク番号に対応する行に1を入れる
LP.SS = [ -1  1  0  0  0  0    0  0  0  0  0  0;
           0 -1  1  0  0  0    0  0  0  0  0  0;
           0  0 -1  1  0  0    0  0  0  0  0  0;
           0  0  0 -1  1  0    0  0  0  0  0  0;
           0  0  0  0 -1  1    0  0  0  0  0  0;
           0  0  0  0  0 -1    0  0  0  0  0  0;
           
           0  0  0  0  0  0   -1  1  0  0  0  0;
           0  0  0  0  0  0    0 -1  1  0  0  0;
           0  0  0  0  0  0    0  0 -1  1  0  0;
           0  0  0  0  0  0    0  0  0 -1  1  0;
           0  0  0  0  0  0    0  0  0  0 -1  1;
           0  0  0  0  0  0    0  0  0  0  0 -1 ];
%%%%% リンクの連結関係を定義する その３ %%%%%
% ベース部(リンク0)に連結しているリンクは1  それ以外は0
LP.S0 = [ 1 0 0 0 0 0   1 0 0 0 0 0  ];

%%%%% 手先 (リンク先端) の定義 %%%%%
% 末端リンクならば1
LP.SE = [ 0 0 0 0 0 1  0 0 0 0 0 1 ];

%%%%% 関節の種類 %%%%%
% 回転関節ならば 'R'  直動関節ならば 'P'
LP.J_type = [ 'R' 'R' 'R' 'R' 'R' 'R'    'R' 'R' 'R' 'R' 'R' 'R' ];

%%%%% ベース重心から関節iへの位置ベクトル %%%%%
% 直接連結していないリンクはすべて0 ベースの座標系からみたもの
LP.c0 = [ -param.robot.baseWidth*.5     0 0 0 0 0     param.robot.baseWidth*.5      0 0 0 0 0 ;
           param.robot.baseDepth*.5     0 0 0 0 0     param.robot.baseDepth*.5      0 0 0 0 0 ;
           0                            0 0 0 0 0     0                             0 0 0 0 0 ];

%%%%% ベース部 (リンク0) の質量 %%%%%
LP.m0 = param.robot.mass_base;

%%%%% ベース部 (リンク0) の慣性モーメント %%%%% 
LP.inertia0 = param.robot.inertia_base;

%%%%% 各リンク（ベース以外）の質量 %%%%%
length_links = param.robot.length_links;
mass_bars = length_links .* param.robot.radius_links.^2 * pi * param.robot.rho_link_frame;   % 丸棒質量
mass_links = param.robot.mass_motor + mass_bars;
LP.m = [mass_links, param.robot.mass_endEffector, ...
        mass_links, param.robot.mass_endEffector];

%%%%% リンク系全体の質量 %%%%%
LP.mass = sum( LP.m ) + LP.m0;

%%%%% 各リンク間の座標系の回転関係 %%%%%
% 根元のリンク座標系を回転させる  列はリンク 行はx,y,z軸を表す  例えば下記の意味はリンク1はベースリンク(リンク0)と同じ
% tipsangle1 =  pi/6; tipsangle2 = -pi/6; tipsangle3 =  pi/6; tipsangle4 = -pi/6;
a=pi/2;
LP.Qi = [  0  0  0 -a  a -a     0  0  0 -a  a -a ;
          -a  a  0  0  0  0     a -a  0  0  0  0 ;
           0  0  0  0  0  0     0  0  0  0  0  0 ];
      
  
%%%%% 各リンクの慣性モーメント %%%%%
% 丸棒モデルで計算
inertia_bars  = zeros(3, 3, 5);
inertia_motor_0 = zeros(1, 1, 5);
inertia_bars_0 = zeros(1, 1, 5);
comOffSet_links = .5 * length_links * param.robot.mass_motor ./ mass_links;
inertia_motor_0(:) = (.5 * length_links - comOffSet_links).^2 .* param.robot.mass_motor;
inertia_bars_0(:)  = comOffSet_links.^2 .* mass_bars;
inertia_bars(1, 1, :) = .25 * mass_bars .* param.robot.radius_links.^2 + mass_bars .* length_links.^2 /12;
inertia_bars(2, 2, :) = inertia_bars(1, 1, :);
inertia_bars(3, 3, :) = .5 * mass_bars .* param.robot.radius_links.^2;
inertia_links = inertia_bars + param.robot.inertia_motor; % 慣性足し算
inertia_links(1, 1, :) = inertia_links(1, 1, :) + inertia_bars_0 + inertia_motor_0;
inertia_links(2, 2, :) = inertia_links(2, 2, :) + inertia_bars_0 + inertia_motor_0;

inertia_links = reshape(inertia_links, [3, 15]);
LP.inertia = [inertia_links, param.robot.inertia_endEffector, ...
              inertia_links, param.robot.inertia_endEffector];
%%%%% 各リンク重心から関節への位置ベクトル %%%%%
% 丸棒モデルで計算
com_to_joint_low = -(length_links * .5 + comOffSet_links);
com_to_joint_upp = length_links * .5 - comOffSet_links;

LP.cc = zeros( 3, LP.num_q, LP.num_q );   % 3×6×6の零テンソル(3×6の零行列を6つ作る)
LP.cc(:,1,1) = [ 0 0 com_to_joint_low(1) ]';   % リンク1の重心から根本側の関節(J1)への位置ベクトル
LP.cc(:,1,2) = [ 0 0 com_to_joint_upp(1) ]';   % リンク1の重心からリンク2の関節への位置ベクトル
LP.cc(:,2,2) = [ 0 com_to_joint_low(2) 0 ]';   % リンク2の重心からリンク2の関節への位置ベクトル
LP.cc(:,2,3) = [ 0 com_to_joint_upp(2) 0 ]';   % リンク2の重心からリンク3の関節への位置ベクトル
LP.cc(:,3,3) = [ 0 com_to_joint_low(3) 0 ]';   % リンク3の重心からリンク3の関節への位置ベクトル
LP.cc(:,3,4) = [ 0 com_to_joint_upp(3) 0 ]';   % リンク3の重心からリンク4の関節への位置ベクトル
LP.cc(:,4,4) = [ 0 0 com_to_joint_low(4) ]';   
LP.cc(:,4,5) = [ 0 0 com_to_joint_upp(4) ]';
LP.cc(:,5,5) = [ 0 com_to_joint_low(5) 0 ]';   
LP.cc(:,5,6) = [ 0 com_to_joint_upp(5) 0 ]';
LP.cc(:,6,6) = [ 0 0 -param.robot.length_endEffector * .5 ]' - param.robot.comOffset_endEffector;    

LP.cc(:, 7:12, 7:12) = LP.cc(:, 1:6, 1:6); % 対象
% 末端リンクから端点までの位置ベクトル  末端リンクの重心から手先への位置ベクトル(手先が複数の場合はそれらの中心)  末端部を持たないリンクは0を入れる
% 末端は関節(6, 12)
LP.ce = zeros(3, 12);
LP.ce(:,6) = LP.cc(:, 6, 6);
LP.ce(:,12) = LP.ce(:,6);

% 末端リンクと端点の回転関係  末端リンクの根本と先端の座標系が同一であるということ  記述法はQiと同じ  末端部を持たないリンクは0を入れる
LP.Qe = zeros(3,LP.num_q);
LP.Qe(:, 6)  = [a, 0, 0]';
LP.Qe(:, 12) = [a, 0, 0]';

% EOF