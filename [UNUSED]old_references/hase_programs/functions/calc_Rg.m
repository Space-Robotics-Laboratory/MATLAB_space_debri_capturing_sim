%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The SpaceDyn, a MATLAB toolbox for Space and Mobile Robots.
% (C)1998 The Space Robotics Lab. directed by Kazuya Yoshida,
% Tohoku University, Japan.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	February 4, 1999, Last modification by K.Yoshida
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Calculation of the position of gravity centroid, Rg

function Rg = calc_Rg( LP, SV )
global Ez

 Rm = zeros(3,1);
   
   for i = 1 : LP.num_q
      
      Rm = Rm + LP.m(i) * SV.RR(:,i);
      
   end
   
   Rm = Rm + LP.m0 * SV.R0;
   Rg = Rm / LP.mass;
   
end
