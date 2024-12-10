%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	returns a 3x3 transformation representing a 
%	rotation about the rotation vector theta.
%
%   2023.1 uchida akiyoshi
%
%	See also CX, CY, CZ.

function E0 = vec2dc( theta )

if ( norm(theta)==0 )
   E0 = eye(3);
   
else
   th = norm(theta);
   w = theta ./ th;
   
   E0 =[ cos(th)+w(1)^2*(1-cos(th)) ...
         w(1)*w(2)*(1-cos(th))-w(3)*sin(th) ...
         w(3)*w(1)*(1-cos(th))+w(2)*sin(th);
         
         w(1)*w(2)*(1-cos(th))+w(3)*sin(th) ...
         cos(th)+w(2)^2*(1-cos(th)) ...
         w(3)*w(2)*(1-cos(th))-w(1)*sin(th);
         
         w(3)*w(1)*(1-cos(th))-w(2)*sin(th) ...
         w(3)*w(2)*(1-cos(th))+w(1)*sin(th) ...
         cos(th)+w(3)^2*(1-cos(th)) ];
   
end

%%%EO
