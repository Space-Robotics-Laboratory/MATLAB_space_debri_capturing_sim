function kakudo = kakudo( A, B, C )

%%%
% A,B,C �̍��W����AB,AC�̃x�N�g�������C�x�N�g��AB����Ƃ����x�N�g��AC�́}��]�p�𒲂ׂ�
%%%

AB = B - A;
AC = C - A;

rot = atan2( AB(2,1), AB(1,1) );

% AB_rot = rpy2dc( 0, 0, -rot )' * AB;
AC_rot = rpy2dc( 0, 0, -rot )' * AC;

kakudo = atan2( AC_rot(2,1), AC_rot(1,1) );   % -�΁`��

end
