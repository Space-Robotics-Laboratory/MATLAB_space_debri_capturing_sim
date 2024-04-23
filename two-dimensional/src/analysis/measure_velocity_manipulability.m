function manipulablity = ...
  measure_velocity_manipulability(num_e, joint_pos, manipulability_type, LP, SV, dimension)
arguments
  num_e = 1;
  joint_pos = 'n';
  manipulability_type = 'volume';
  LP = DualArmRobo_LP(set_Param()); % From Matlab_space_debris_capturing_sim
  SV = init_SV_DAR(LP); % From Matlab_space_debris_capturing_sim
  dimension = [1 2 6]; % dimension to be discussed vx, vy, wz
end
% A function to measure a manipulability of the selected enf end effector
%   in 2d space
% input:
% num_e: int (SpaceDyn) end effector number (defalut L=1, R=2)
% joint_pos: n*m double (n: time steps, m:joint numbers) joints' position
% manipulability_type: string definition of manipulability
% LP: struct (SpaceDyn) Link Parameter
% SV: struct (SpaceDyn) State Vector

% If joint_pos is set manialy, update SV
if joint_pos == 'n'
  joint_pos = SV.q';
end

joints = j_num(LP, num_e);
joint_pos_size = size(joint_pos);
time_step_lengh = joint_pos_size(1);

% result container
manipulablity = zeros(time_step_lengh, 1);

switch manipulability_type
    case 'volume'
      for time_step = 1:time_step_lengh
        SV.q = joint_pos(time_step, :)';
        SV = calc_aa(  LP, SV );                                        
        SV = calc_pos( LP, SV );
      
        Jacobian = calc_je(LP, SV, joints);
        
        J = Jacobian(dimension, :);
        
        A = J * J';
      
        manipulablity(time_step) = sqrt(det(A));
      end

end