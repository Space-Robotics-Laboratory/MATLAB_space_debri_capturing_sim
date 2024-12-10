% ３次元配列からシートごとに列を選択し，元の関数と同サイズの配列を返す関数
%
% 2023.2 akiyoshi uchida
%
% input: A matrix(i*j*k), n vector like (k)
% output:C matrix(i*1*k)
%

function C = pick(A, n)
sA = size(A);
if sA(3) ~= n
    error("n shoud have same size of A's dim3")
elseif size(sA) ~= 3
    error("A should be 3 dimentional array")
end
C = zeros(sA(1), 1, sA(3));
for i = 1:sA(3)
    C(:, 1, i) = A(:, n(i), i);
end