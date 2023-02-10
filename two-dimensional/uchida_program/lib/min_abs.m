% 行列要素を比較し，各要素について絶対値の小さい方を選択した行列を返す．この時，Cの符号はAに従う．
%
% 2023.2 uchida akiyoshi
% 
% input : A n*m, B n*m
% output: C n*m
%

function C = min_abs(A, B)
if size(A) ~= size(B)
    error('A and B should have same size')
end
C = A;
i = abs(A) > abs(B);
C(i) = sign(A(i)) .* abs(B(i));
end