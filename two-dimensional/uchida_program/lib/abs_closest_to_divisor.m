% 剰余計算をし，除数との絶対値の差がより小さくなる符号を選択する関数
%
% 2023.4 akiyoshi uchida
%
% EXAPLE: input(3, 5)->output(-2)

function result = abs_closest_to_divisor(dividend, divisor)
    mod_result = mod(dividend, divisor);
    if abs(mod_result) > divisor/2
        result = mod_result - sign(mod_result)*divisor;
    else
        result = mod_result;
    end
end
