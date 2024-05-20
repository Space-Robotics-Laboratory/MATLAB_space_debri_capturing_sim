% v = rand(3, 1)
% v = v ./ vecnorm(v) * deg2rad(30)
% C = vec2dc(v)
% qtn = dc2qtn_2(C')
% rad2deg(acos(qtn(4))*2)
% 
% CC = qtn2dc(qtn)'
% 
% CC - C

function err = test_qtn(C)

qtn = dc2qtn_2(C');
CC = qtn2dc(qtn)';

err = C - CC;

end