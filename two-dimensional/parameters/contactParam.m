% 接触係数設定
function cParam = contactParam()
cParam.damp  = 2;%8;%20;           % 接触力減衰係数
cParam.elast = 5000;%9000;%1000;    % 接触力弾性係数
cParam.friction = 0.1;%0.3;         % 接触力摩擦係数

end