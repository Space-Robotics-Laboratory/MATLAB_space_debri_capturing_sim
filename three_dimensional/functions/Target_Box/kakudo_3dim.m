function kakudo = kakudo_3dim( A, B, C )

%%%
% A,B,C の座標からAB,ACのベクトルを作り，ベクトルABを基準としたベクトルACの±回転角を調べる
%%%

AB = B - A;
AC = C - A;

rotz = atan2( AB(2,1), AB(1,1) );

AC_rot = rpy2dc( 0, 0, -rotz )' * AC;

kakudo(3) = atan2( AC_rot(2,1), AC_rot(1,1) );   % -π～π

roty = atan2( AB(1,1), AB(3,1) );

AC_rot = rpy2dc( 0, -roty, 0 )' * AC;

kakudo(2) = atan2( AC_rot(1,1), AC_rot(3,1) );   % -π～π

rotx = atan2( AB(3,1), AB(2,1) );

AC_rot = rpy2dc( -rotx, 0, 0 )' * AC;

kakudo(1) = atan2( AC_rot(3,1), AC_rot(2,1) );   % -π～π

end
