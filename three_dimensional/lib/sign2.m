% 0以上であれば1，0未満であれば-1を返す関数．signの0を除くもの．
%
% 2023.3 akiyoshi uchida
%
function res = sign2(real)

if real < 0
    res = -1;
else
    res = 1;
end
