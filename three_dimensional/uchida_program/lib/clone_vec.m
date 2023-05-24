% 配列を入力回数コピーし，元の要素と隣り合ったものになるような新たな配列を返す.
% 
% 2023.4 akiyoshi uchida
%
% input: [a, b, c, 3] 
% output:[a, a, a, b, b, b, c, c, c]
function res = clone_vec(vec, n)

[i, j] = size(vec);
if i~=1 
    error('you have to input vactor')
end
res_ = repmat(vec, [n,1]);
res = reshape(res_, [1, n*j]);
