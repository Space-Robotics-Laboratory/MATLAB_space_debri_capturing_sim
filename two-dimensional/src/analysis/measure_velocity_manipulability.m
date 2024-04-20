function [linier_manipulablity, angular_manipulability] = ...
  measure_velocity_manipulability(manipulability_type, LP, SV, num_e)
% A function to measure a manipulability of the selected enf end effector
%   in 2d space
% input:
% manipulability_type: string definition of manipulability
% LP: struct (SpaceDyn) Link Parameter
% SV: struct (SpaceDyn) State Vector
% num_e: int (SpaceDyn) end effector number (defalut L=1, R=2)

joints = j_num(LP, num_e);
Jacobian = calc_je(LP, SV, joints);

linier_indices = 1:3;
angular_indices = 4:6;

J_v = Jacobian(linier_indices, :);
J_w = Jacobian(angular_indices, :);

A_v = J_v * J_v';
A_w = J_w * J_w';

A_v_2d = A_v(1:2, 1:2);
A_w_2d = A_w(1:2, 1:2);

switch manipulability_type
    case 'volume'
        linier_manipulablity = sqrt(det(A_v_2d));
        angular_manipulability = sqrt(det(A_w_2d));
end