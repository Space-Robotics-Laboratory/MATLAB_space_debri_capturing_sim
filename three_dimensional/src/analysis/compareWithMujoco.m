mat_csv = "\dat\2024-0509\2024-0509-225953\testRk-dat\savedDat.csv";
mat_tabel = readtable(mat_csv);
mat_ang_vel = [mat_tabel.targetWb_1, mat_tabel.targetWb_2, mat_tabel.targetWb_3];

mj_csv = "../../mujoco_dar_simulation/tutorial/case0/ang_vel.csv";
mj_table = readtable(mj_csv);
mj_ang_vel = [mj_table.Var1, mj_table.Var2, mj_table.Var3]