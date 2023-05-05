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
% CALC_GGJ_2ARM returns generalized jacobian of 2 arm robot, GJ2A ( 18*n )
%                       base-tip jacobian, Jb = [ Jb_L; Jb_R ]    ( 18*6 )
%                       inertia matrix, HH                        (6+n * &+n)
%
% CALC_GJ_2ARMと違い，ベースの速度まで制御するヤコビアン

function [GGJ2A, Jb, Jm, HH] = calc_ggj_2arm( LP, SV, num_eL, num_eR )

% Calculate inertia matrices, HH
HH = calc_hh( LP, SV );

% Find joint connection from the end-link to the 0-th link
jointsL = j_num(LP, num_eL);
jointsR = j_num(LP, num_eR);

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
HbHbmL = Hb\Hbm_L;
HbHbmR = Hb\Hbm_R;

% Calculate the Generalized Jacobian
GGJ2A =[ -HbHbmL           , -HbHbmR            ;         
         Jm_L - Jb_L*HbHbmL, -Jb_L*HbHbmR       ;
         -Jb_R*HbHbmL      , Jm_R - Jb_R*HbHbmR ];

% base tip jacobian
Jb = [Jb_L; Jb_R];

% joint tip jacobian
Jm = [Jm_L,        zeros(6, num_eR);
      zeros(6,num_eL),  Jm_R];


%%% EOF
