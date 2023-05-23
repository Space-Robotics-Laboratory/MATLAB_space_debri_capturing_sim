function LP_t = TargetSquare_LP(Param)   % □型ターゲット

LP_t.BB = [];   % BB(i) = 根元側で結合しているリンク番号

LP_t.num_q = length(LP_t.BB);

%ベース部(リンク0)に連結しているリンクは1，それ以外は0
LP_t.S0 = [];

% 対角成分はすべて-1，各列が左からそれぞれリンク1,2,3を表している，根元側で結合しているリンク番号に対応する行に1を入れる
LP_t.SS = [];
      
% 末端リンクならば 1
LP_t.SE = [];   % 回転関節ならば 'R', 直動関節ならば 'P'
LP_t.J_type = [];
LP_t.m = [];

% ベースの質量 [kg]
rho = Param.target.density;
width = Param.target.width;
depth = Param.target.depth;
mass = rho * width * depth;
LP_t.m0 = mass ;

LP_t.mass = sum(LP_t.m) + LP_t.m0;

% 根元のリンク座標系を回転させる 列はリンク，行はx,y,z軸を表す
LP_t.Qi = zeros(3,LP_t.num_q);

% 末端リンクと端点の回転関係，末端部を持たないリンクは0を入れる
LP_t.Qe = zeros(3,LP_t.num_q);

% ベース重心から関節iへの位置ベクトル
% 直接連結していないリンクはすべて0
% ベースの座標系からみたもの
LP_t.c0 = zeros(3,LP_t.num_q);

% 各リンク重心から関節への位置ベクトル
LP_t.cc = zeros(3,LP_t.num_q,LP_t.num_q);

LP_t.ce = zeros(3,LP_t.num_q);

% ベースの慣性テンソル
LP_t.inertia0 = [1e9 0 0;
                 0   1e9 0;
                 0,    0,  mass * width * depth / 6];

LP_t.inertia = zeros(3,3*LP_t.num_q);
