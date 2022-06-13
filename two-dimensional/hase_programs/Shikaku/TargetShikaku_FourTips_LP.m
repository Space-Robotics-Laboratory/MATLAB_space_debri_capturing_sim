function LP = TargetShikaku_FourTips_LP()   % □型ターゲット

LP.BB = [];   % BB(i) = 根元側で結合しているリンク番号

LP.num_q = length(LP.BB);

%ベース部(リンク0)に連結しているリンクは1，それ以外は0
LP.S0 = zeros(1,LP.num_q);
LP.S0 = [];

% 対角成分はすべて-1，各列が左からそれぞれリンク1,2,3を表している，根元側で結合しているリンク番号に対応する行に1を入れる
LP.SS = zeros(LP.num_q,LP.num_q);
LP.SS = [];
      
% 末端リンクならば 1
LP.SE = zeros(1,LP.num_q);
LP.SE = [];   % 回転関節ならば 'R', 直動関節ならば 'P'
LP.J_type = zeros(1,LP.num_q);
LP.J_type = [];

LP.m = zeros(1,LP.num_q);
LP.m = [];

% ベースの質量 [kg]
LP.m0 = 3.700 * 1;

LP.mass = sum(LP.m) + LP.m0;

% 根元のリンク座標系を回転させる 列はリンク，行はx,y,z軸を表す
LP.Qi = zeros(3,LP.num_q);

% 末端リンクと端点の回転関係，末端部を持たないリンクは0を入れる
LP.Qe = zeros(3,LP.num_q);

% ベース重心から関節iへの位置ベクトル
% 直接連結していないリンクはすべて0
% ベースの座標系からみたもの
LP.c0 = zeros(3,LP.num_q);

% 各リンク重心から関節への位置ベクトル
LP.cc = zeros(3,LP.num_q,LP.num_q);

LP.ce = zeros(3,LP.num_q);

% ベースの慣性テンソル
tt = 3 * 1;
LP.inertia0 = [ 1e9 0   0;
                0   1e9 0;
                0   0   0.01163261858 * tt ];

LP.inertia = zeros(3,3*LP.num_q);
