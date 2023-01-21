%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The SpaceDyn, a MATLAB toolbox for Space and Mobile Robots.
% (C)1998 The Space Robotics Lab. directed by Kazuya Yoshida,
% Tohoku University, Japan.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	February 4, 1999, Last modification by K.Yoshida
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 　
% 2023.1 uchida akiyoshi
% 
% 　calc_hhを編集して作成
% 
%    CALC_HH	Calculate the Inertia Matrices H.
%
%    CALC_HH returns the inertia matrices HH (6+n)x(6+n).
% 
%    CALC_HO returns the inertia matrix of base 6*6
% 
%   (C)Space Robotics Lab, by Koichi Fujishima
%   2001.9.12 modified by H.Hamano
%   2002.2.27 H.Hamano modified to version up
%

function H0 = calc_h0( LP, SV )

% Calculation of HH matrix
wE = LP.m0 * eye(3,3);

HH_w = SV.A0*LP.inertia0*SV.A0';
H0 = [         wE  zeros(3,3);
      zeros(3,3)        HH_w ];
   

%%% EOF
