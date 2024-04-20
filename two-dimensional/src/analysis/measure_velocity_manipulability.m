function [linier_manipulablity, angular_manipulability] = ...
  measure_velocity_manipulability(num_e, joint_pos, manipulability_type, LP, SV)
arguments
  num_e = 1;
  joint_pos = 'n';
  manipulability_type = 'volume';
  LP = DualArmRobo_LP(set_Param()); % From Matlab_space_debris_capturing_sim
  SV = init_SV_DAR(LP); % From Matlab_space_debris_capturing_sim
end
% A function to measure a manipulability of the selected enf end effector
%   in 2d space
% input:
% num_e: int (SpaceDyn) end effector number (defalut L=1, R=2)
% joint_pos: n*m double (n: time steps, m:joint numbers) joints' position
% manipulability_type: string definition of manipulability
% LP: struct (SpaceDyn) Link Parameter
% SV: struct (SpaceDyn) State Vector

% const
linier_indices = 1:3;
angular_indices = 4:6;

% If joint_pos is set manialy, update SV
if joint_pos == 'n'
  joint_pos = SV.q';
end

joints = j_num(LP, num_e);
joint_pos_size = size(joint_pos);
time_step_lengh = joint_pos_size(1);

% result container
linier_manipulablity = zeros(time_step_lengh, 1);
angular_manipulability = zeros(time_step_lengh, 1);

switch manipulability_type
    case 'volume'
      for time_step = 1:time_step_lengh
        SV.q = joint_pos(time_step, :)';
        SV = calc_aa(  LP, SV );                                        
        SV = calc_pos( LP, SV );
      
        Jacobian = calc_je(LP, SV, joints);
        
        J_v = Jacobian(linier_indices, :);
        J_w = Jacobian(angular_indices, :);
        
        A_v = J_v * J_v';
        A_w = J_w * J_w';
        
        A_v_2d = A_v(1:2, 1:2);
        A_w_2d = A_w(1:2, 1:2);
      
        linier_manipulablity(time_step) = sqrt(det(A_v_2d));
        angular_manipulability(time_step) = sqrt(det(A_w_2d));
      end

end