% 接触係数設定
%
% 2023.4 akiyoshi uchida

function contactParam = contactParam()

contactParam.elast = 1000;      % 弾性係数
contactParam.damper = 20;       % 減衰係数
contactParam.friction = .3;     % 摩擦係数
end