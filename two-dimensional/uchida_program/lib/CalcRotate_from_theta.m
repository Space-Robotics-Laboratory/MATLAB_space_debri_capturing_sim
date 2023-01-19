%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	returns a 3x3 transformation representing a 
%	rotation about the vector theta0.
%
%   2002.2.27 H.Hamano modified to version up
%
%	See also CX, CY, CZ.

function E0 = CalcRotate_from_theta( theta0 )

if ( norm(theta0)==0 )
   E0 = eye(3);
   
else
   th = norm(theta0);
   w = theta0 ./ th;
   
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
