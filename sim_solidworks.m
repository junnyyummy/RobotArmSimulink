% run_sim.m

% 设置路径
cd('D:\MyProjects\CubeSat')

% 导入模型
smimport('cubesat_with_arms.step')

% 打开模型
open_system('cubesat_with_arms')

% 运行仿真
sim('cubesat_with_arms')

% 播放动画
smplay('cubesat_with_arms')
