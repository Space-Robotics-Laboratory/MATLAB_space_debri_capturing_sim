%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The SpaceDyn, a MATLAB toolbox for Space and Mobile Robots.
% (C)1998 The Space Robotics Lab. directed by Kazuya Yoshida,
% Tohoku University, Japan.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	June 8, 1998, Last modification by K.Yoshida
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%% CALC_GJ	Calculate the Generalized Jacobian.
%
%   CALC_GJ returns the generalized jacobian, GJ (6xn).
%
%   1998 (C)Space Robotics Lab, by Koichi Fujishima
%   2001.9.13 H.Hamano
%   2002.2.27 H.Hamano modified to version up
%
% 2023.1 uchida akiyoshi
% CALC_GJを編集して作成
% 
% CALC_GJ_2ARM returns generalized jacobian of 2 arm robot, GJ2A ( 12*n )

function GJ2A = calc_gj_2arm( DualArmRobo )

LP = DualArmRobo.LP;
SV = DualArmRobo.SV;

% Calculate inertia matrices, HH
HH = calc_hh( LP, SV );

% Find joint connection from the end-link to the 0-th link
jointsL = DualArmRobo.jointsL;
jointsR = DualArmRobo.jointsR;

% calculate Jacobian and inertia matrices
Jm_L_ = calc_je( LP, SV, jointsL );
Jm_L = Jm_L_(:, jointsL);
Jm_R_ = calc_je( LP, SV, jointsR );
Jm_R = Jm_R_(:, jointsR);

[ pe_L, ~ ] = f_kin_e( LP, SV, jointsL );
[ pe_R, ~ ] = f_kin_e( LP, SV, jointsR );

Jb_L = [   eye(3,3) -tilde(pe_L-SV.R0);
         zeros(3,3)         eye(3,3) ];

Jb_R = [   eye(3,3) -tilde(pe_R-SV.R0);
         zeros(3,3)         eye(3,3) ];

Hb = HH(1:6, 1:6);
Hbm_L = HH(1:6, 6+jointsL);
Hbm_R = HH(1:6, 6+jointsR);

% Calculate the Generalized Jacobian
GJ2A = [ Jm_L - Jb_L*(Hb\Hbm_L), -Jb_L*(Hb\Hbm_R)       ;
         -Jb_R*(Hb\Hbm_L)      , Jm_R - Jb_R*(Hb\Hbm_R)];
 


%%% EOF
