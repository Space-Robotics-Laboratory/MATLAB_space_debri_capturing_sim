mat_csv = "dat\2024-0514\2024-0514-231950\testRk-dat\savedDat.csv";
mat_tabel = readtable(mat_csv);
mat_ang_vel = [mat_tabel.targetWb_1, mat_tabel.targetWb_2, mat_tabel.targetWb_3];
mat_time = mat_tabel.time;

mj_csv = "../../mujoco_dar_simulation/tutorial/case0/ang_vel.csv";
mj_table = readtable(mj_csv);
mj_time = mj_table.Var1;
mj_ang_vel = [ mj_table.Var2, mj_table.Var3, mj_table.Var4];


diff_ang_vel = mat_ang_vel(2:end,:) - mj_ang_vel(1:end-1,:);

plot(diff_ang_vel)
% plot(mat_time, mat_ang_vel)
% hold on 
% plot(mj_time, mj_ang_vel)
legend("x", "y", "z")
title("Difference between mujoco and spacedyn (dt=0.1)")