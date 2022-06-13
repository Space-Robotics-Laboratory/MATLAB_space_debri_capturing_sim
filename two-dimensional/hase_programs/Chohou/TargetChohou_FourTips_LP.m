function LP = TargetChohou_FourTips_LP()   % 〇型ターゲット

% LP.BB = [0];
% LP.SS = [1];
% LP.S0 = [1];
% LP.SE = [1];
% LP.J_type = ['P'];
% LP.m = [mi];
LP.BB = [];% BB(i) = 根元側で結合しているリンク番号

LP.num_q = length(LP.BB);

%ベース部(リンク0)に連結しているリンクは1
%それ以外は0
LP.S0 = zeros(1,LP.num_q);
LP.S0 = [];

% 対角成分はすべて-1
% 各列が左からそれぞれリンク１，２，３を表している
% 根元側で結合しているリンク番号に対応する行に１を入れ
LP.SS = zeros(LP.num_q,LP.num_q);
LP.SS = [];
      
% 末端リンクならば 1
LP.SE = zeros(1,LP.num_q);
LP.SE = [];
% 回転関節ならば 'R'
% 直動関節ならば 'P'
LP.J_type = zeros(1,LP.num_q);
LP.J_type = [];

LP.m = zeros(1,LP.num_q);
LP.m = [];


% ベースの質量 [kg]
LP.m0 = 3.700 * 2;
% LP.m0 = 15.015;
%LP.m0 = 14.925;%これは分セミ実験にて計測したやつ，でもとりあえず上のQtにあわせる
% LP.m0 = mi;

LP.mass = sum(LP.m) + LP.m0;

% LP.Qi = [0 90 0]' * pi / 180;
% LP.Qe = [0 0 0]' * pi / 180;
% LP.c0 = [0.5 0 0]';
% LP.cc(:,1,1) = [0 0 -0.3]';
% LP.ce = [0 0 0.3]';
% 根元のリンク座標系を回転させる 列はリンク，行はx軸y軸z軸を表す
LP.Qi = zeros(3,LP.num_q);

% 末端リンクと端点の回転関係
% 末端部を持たないリンクは0を入れる
LP.Qe = zeros(3,LP.num_q);

%%%%% ベース重心から関節iへの位置ベクトル %%%%%
% 直接連結していないリンクはすべて 0
% ベースの座標系からみたもの
LP.c0 = zeros(3,LP.num_q);

%%%%% 各リンク重心から関節への位置ベクトル %%%%%
LP.cc = zeros(3,LP.num_q,LP.num_q);

LP.ce = zeros(3,LP.num_q);

t = 1.5;
% ベースの慣性テンソル
LP.inertia0 = [ 1e9 0   0;
                0   1e9 0;
                0   0   0.01074964594 * t ];

LP.inertia = zeros(3,3*LP.num_q);
