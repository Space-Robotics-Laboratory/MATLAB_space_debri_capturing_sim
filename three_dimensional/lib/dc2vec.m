%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	returns a rotation vector theta representing a 
%	rotation about the 3x3 transformation.
%
%   2023.1 uchida akiyoshi
%
%	See also CX, CY, CZ.
%   tr(R) = 1 + 2cos(th)

function theta = dc2vec(R)
    % th = 0
    tR = trace(R);
    if tR == 3
        theta = zeros(3,1);
    
    % th = pi
    elseif tR == -1
        % 条件を満たすRiiを用いる
        I = eye(3);
        th = pi;
        Rii = diag(R);
        i = find(Rii > -1);                      
        feasible_Rii = Rii(i(1));               
        theta = th / sqrt(2*(1 + feasible_Rii(1))) * (R(:, i(1)) + I(:, i(1)));
    
    % 0 < th < pi
    else
        th = acos(.5*(tR - 1));
        theta_tilde = .5 / sin(th) * (R - R');
        theta = th * inv_tilde(theta_tilde);
    end
end