function LP_d = DualArm_FourTips_LP_v3()   % 2018/8/15 2018年新テストベッド用に書き換え 長谷

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% 各リンクパラメータの定義 %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%% リンクの連結関係を定義する %%%%%
% LPは各リンクの初期条件を記述している  LPをデータで渡すときに様々なデータを渡せるように構造体にしている  BB(i) = 根元側で結合しているリンク番号
LP_d.BB = [ 0 1 2 3    0 5 6 7 ];   % ベース(リンク0)からリンク1が生えている  このときリンク1から見た根本側とはベースへの向きのことであるので0が一番左に代入されている

% リンク数
LP_d.num_q = length( LP_d.BB );   % lengthは行数と列数の内の大きい方を与える

%%%%% リンクの連結関係を定義する その２ %%%%%
% 対角成分はすべて-1  各列が左からそれぞれリンク1,2を表している  根元側で結合しているリンク番号に対応する行に1を入れる
LP_d.SS = [ -1  1  0  0     0  0  0  0;
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
bl = 0.00;
LP_d.c0 = [ -0.16+bl 0 0 0    0.16+bl 0 0 0;
           0.10946 0 0 0    0.10946 0 0 0;
           0       0 0 0    0       0 0 0 ];

%%%%% ベース部 (リンク0) の質量 %%%%%
LP_d.m0 = 7.695;   % = 5.145+2.550;

%%%%% ベース部 (リンク0) の慣性モーメント %%%%% 
LP_d.inertia0 = [ 1e9 0   0;   % 0の部分がカップリング項(慣性乗積?)
                0   1e9 0;   % 慣性乗積が0ということは、固定軸の周りを回転するということ
                0   0   0.09783069148];
            
nm = 3 / 2;
%%%%% 各リンク（ベース以外）の質量 %%%%%
la_m = 0.570; lb_m = 0.560 * nm / 2; lc_m = 0.560 * nm / 2;
LP_d.m = [ la_m la_m lb_m lc_m    la_m la_m lb_m lc_m ];

%%%%% リンク系全体の質量 %%%%%
LP_d.mass = sum( LP_d.m ) + LP_d.m0;

%%%%% 各リンク間の座標系の回転関係 %%%%%
% 根元のリンク座標系を回転させる  列はリンク 行はx,y,z軸を表す  例えば下記の意味はリンク1はベースリンク(リンク0)と同じ
% tipsangle1 =  pi/6; tipsangle2 = -pi/6; tipsangle3 =  pi/6; tipsangle4 = -pi/6;
LP_d.Qi = [ 0 0 0 0  0 0 0 0;
          0 0 0 0  0 0 0 0;
          0 0 0 0  0 0 0 0 ];
    
%%%%% 各リンクの慣性モーメント %%%%%
la_I = 0.00017771016; lb_I = 0.00067610325 * nm / 2; lc_I = 0.00067610325 * nm / 2;
LP_d.inertia = [1e9 0   0     1e9 0   0     1e9 0   0     1e9 0   0         1e9 0   0     1e9 0   0     1e9 0   0     1e9 0   0;
              0   1e9 0     0   1e9 0     0   1e9 0     0   1e9 0         0   1e9 0     0   1e9 0     0   1e9 0     0   1e9 0;  
              0   0   la_I  0   0   la_I  0   0   lb_I  0   0   lc_I      0   0   la_I  0   0   la_I  0   0   lb_I  0   0   lc_I ];

%%%%% 各リンク重心から関節への位置ベクトル %%%%%
% 初期化
n = 8;      % nはリンク数
LP_d.cc = zeros( 3, n, n );   % 3×6×6の零テンソル(3×6の零行列を6つ作る)

% 定義
t1 = 0.1; 
t2 = 0.03;
t3 = 0.0; 
t4 = -0.01;
te1 = 0.03;

% y要素以外全部ゼロ
LP_d.cc(:,1,1) = [ 0 -0.14641-t1 0 ]';   % リンク1の重心から根本側の関節(J1)への位置ベクトル
LP_d.cc(:,1,2) = [ 0  0.00459 0 ]';   % リンク1の重心からリンク2の関節への位置ベクトル
LP_d.cc(:,2,2) = [ 0 -0.14641-t2 0 ]';   % リンク2の重心からリンク2の関節への位置ベクトル
LP_d.cc(:,2,3) = [ 0  0.00459 0 ]';   % リンク2の重心からリンク3の関節への位置ベクトル
LP_d.cc(:,3,3) = [ 0 -0.03+t3 0 ]';      % リンク3の重心からリンク3の関節への位置ベクトル
LP_d.cc(:,3,4) = [ 0  0.03+t4 0 ]';   % リンク3の重心からリンク4の関節への位置ベクトル
LP_d.cc(:,4,4) = [ 0 -te1 0 ]';       % リンク4の重心からリンク4の関節への位置ベクトル

LP_d.cc(:,5,5) = [ 0 -0.14641-t1 0 ]';
LP_d.cc(:,5,6) = [ 0  0.00459 0 ]';
LP_d.cc(:,6,6) = [ 0 -0.14641-t2 0 ]';
LP_d.cc(:,6,7) = [ 0  0.00459 0 ]';
LP_d.cc(:,7,7) = [ 0 -0.03+t3 0 ]';
LP_d.cc(:,7,8) = [ 0  0.03+t4 0 ]';
LP_d.cc(:,8,8) = [ 0 -te1 0 ]';

% 末端リンクから端点までの位置ベクトル  末端リンクの重心から手先への位置ベクトル(手先が球の場合は球の中心まで)  末端部を持たないリンクは0を入れる
% L = 0.07;
% theta = pi/6;
% te2 = L*sin(theta); te3 = L*cos(theta) - te1;

% 定義
te2 = 0;
te3 = 0;

LP_d.ce = [ 0 0 0 -te2    0 0 0 -te2;   % 手先リンク重心位置=手先位置
          0 0 0  te3    0 0 0  te3;     % 結局ゼロなんかい
          0 0 0  0      0 0 0  0 ];

% 末端リンクと端点の回転関係  末端リンクの根本と先端の座標系が同一であるということ  記述法はQiと同じ  末端部を持たないリンクは0を入れる
LP_d.Qe = [ 0 0 0 0    0 0 0 0;
          0 0 0 0    0 0 0 0;
          0 0 0 0    0 0 0 0 ];

% EOF