function LP_d = DualArmRobo_LP(param)   %update uchida

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% 各リンクパラメータの定義 %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%基本的にParamSetting.m中で設定する%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%% リンクの連結関係を定義する %%%%%
% LPは各リンクの初期条件を記述している  LPをデータで渡すときに様々なデータを渡せるように構造体にしている  BB(i) = 根元側で結合しているリンク番号
LP_d.BB = [ 0 1 2 3    0 5 6 7 ];   % ベース(リンク0)からリンク1が生えている  このときリンク1から見た根本側とはベースへの向きのことであるので0が一番左に代入されている

% リンク数
%ベースを含まない
LP_d.num_q = length( LP_d.BB );   % lengthは行数と列数の内の大きい方を与える

%%%%% リンクの連結関係を定義する その２ %%%%%
% 対角成分はすべて-1  各列が左からそれぞれリンク1,2を表している  根元側で結合しているリンク番号に対応する行に1を入れる
LP_d.SS =  [  -1  1  0  0     0  0  0  0;
               0 -1  1  0     0  0  0  0;
               0  0 -1  1     0  0  0  0;
               0  0  0 -1     0  0  0  0;
               0  0  0  0    -1  1  0  0;
               0  0  0  0     0 -1  1  0;
               0  0  0  0     0  0 -1  1;
               0  0  0  0     0  0  0 -1 ];

%%%%% リンクの連結関係を定義する その３ %%%%%
% ベース部(リンク0)に連結しているリンクは1  それ以外は0
LP_d.S0 = [ 1 0 0 0    1 0 0 0 ];

%%%%% 手先 (リンク先端) の定義 %%%%%
% 末端リンクならば1
LP_d.SE = [ 0 0 0 1    0 0 0 1 ];

%%%%% 関節の種類 %%%%%
% 回転関節ならば 'R'  直動関節ならば 'P'
LP_d.J_type = [ 'R' 'R' 'R' 'R'    'R' 'R' 'R' 'R' ];

%%%%% ベース重心から関節iへの位置ベクトル %%%%%
% 直接連結していないリンクはすべて0 ベースの座標系からみたもの
zero3 = zeros(3,1);
baseToJoint1 = [-param.robot.baseWidth/2; param.robot.baseDepth/2; 0] - param.robot.comOffset_base;
baseToJoint5 = [ param.robot.baseWidth/2; param.robot.baseDepth/2; 0] - param.robot.comOffset_base;
LP_d.c0 = [ baseToJoint1, zero3, zero3, zero3,   baseToJoint5, zero3, zero3, zero3 ];

%%%%% ベース部 (リンク0) の質量 %%%%%
LP_d.m0 = param.robot.mass_base;%7.70;5.150+2.550

%%%%% ベース部 (リンク0) の慣性モーメント %%%%% 
LP_d.inertia0 = param.robot.inertia_base;
            
%%%%% 各リンク（ベース以外）の質量 %%%%%
LP_d.m = repmat([param.robot.mass_links, param.robot.mass_endEffector], [1,2]);

%%%%% リンク系全体の質量 %%%%%
LP_d.mass = sum( LP_d.m ) + LP_d.m0;

%%%%% 各リンク間の座標系の回転関係 %%%%%
% 根元のリンク座標系を回転させる  列はリンク 行はx,y,z軸を表す.
LP_d.Qi = zeros([3,8]);
    
%%%%% 各リンクの慣性モーメント %%%%%
LP_d.inertia = repmat([param.robot.inertia_links, param.robot.inertia_endEffector], [1,2]);

%%%%% 各リンク重心から関節への位置ベクトル %%%%%
%座標の表現はリンクの根元の座標系から．
% y要素以外全部ゼロ
LP_d.cc = zeros(3,8,8);
centerOfLink2joint_low = [zero3'; -param.robot.length_links * .5; zero3'] - param.robot.comOffset_links;
centerOfLink2joint_up  = [zero3';  param.robot.length_links * .5; zero3'] - param.robot.comOffset_links;
LP_d.cc(:, [1,  10, 19]) = centerOfLink2joint_low;    % [:,n,n] n = 1,2,3 com_link N to joint N
LP_d.cc(:, [37, 46, 55]) = centerOfLink2joint_low;    % [:,n,n] n = 5,6,7 com_link N to joint N
LP_d.cc(:, [9,  18, 27]) = centerOfLink2joint_up;     % [:,n,n-1] n = 2,3,4 com_link N to joint N-1
LP_d.cc(:, [45, 54, 63]) = centerOfLink2joint_up;     % [:,n,n-1] n = 6,7,8 com_link N to joint N-1

length_endEffec = param.robot.endEffector_h * cos(param.robot.endEffector_gamma);
centerOfEE2End   = [0,  length_endEffec * .5, 0]' - param.robot.comOffset_endEffector;
centerOfEE2joint = [0, -length_endEffec * .5, 0]' - param.robot.comOffset_endEffector;
LP_d.cc(:, 4, 4) = centerOfEE2joint;
LP_d.cc(:, 8, 8) = centerOfEE2joint;

% 末端リンクから端点までの位置ベクトル  末端リンクの重心から手先（２つの先端球の中点）への位置ベクトル  末端部を持たないリンクは0を入れる
LP_d.ce = zeros(3, 8);
LP_d.ce(:, 4) = centerOfEE2End;
LP_d.ce(:, 8) = centerOfEE2End;

% 末端リンクと端点の回転関係  末端リンクの根本と先端の座標系が同一であるということ  記述法はQiと同じ  末端部を持たないリンクは0を入れる
LP_d.Qe = zeros([3,8]);

% EOF