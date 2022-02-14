%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The SpaceDyn, a MATLAB toolbox for Space and Mobile Robots.
% (C)1998 The Space Robotics Lab. directed by Kazuya Yoshida,
% Tohoku University, Japan.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    version 1 // Oct 4, 1999, Last modification by K.Yoshida
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
% 	QTN2DC	Convert quaternion to direction cosine matrix,
%
%		C0 = QTN2DC( QTN )
%
% Note: Here the "Quaternion" specifically means Euler Symmetric Parameters.
%       See p.414-415 in "Spacecraft Attitude Determination and Control,"
%       Ed. by J. R. Werts, Kluwer Academic Publishers.
%
%   2020.11.16 W. Ribeiro
%

function C0 = qtn2dc( qtn )

% input check-out

	if [4,1]~=size( qtn ) 
        if [1,4]~=size( qtn )
            error('Cannot compute the rotation matrix\n');
        end
	end


% start
	
   
    q0 = qtn(4); 
    q1 = qtn(1); q2 = qtn(2); q3 = qtn(3);
    
    
    C0 = [ 1 - 2*(q2^2 + q3^2)         2*(q1*q2 + q0*q3)        2*(q1*q3 - q0*q2);
           2*(q1*q2 - q0*q3)          1 - 2*(q1^2 + q3^2)       2*(q0*q1 + q2*q3);
           2*(q1*q3 + q0*q2)           2*(q2*q3 - q0*q1)       1 - 2*(q1^2 + q2^2)];

%%% EOF





