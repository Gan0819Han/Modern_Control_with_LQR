%% --- 现代控制理论课程设计：直流电机位置伺服系统（含状态观测器与鲁棒性分析） ---
%
% 本脚本将完成以下任务:
% 1. 建立理想直流电机的状态空间模型.
% 2. 分析开环系统的稳定性和能控性.
% 3. 使用极点配置法设计状态反馈控制器.
% 4. 使用LQR最优控制法设计状态反馈控制器.
% 5. 仿真并比较两种理想控制器性能.
% 6. (拓展一) 设计状态观测器并仿真其性能.
% 7. (拓展二) 分析控制器在模型参数失配时的鲁棒性.
%
clc;
clear;
close all;

%% ========================================================================
% 第一步: 系统建模与参数定义 (理想名义模型)
% =========================================================================
fprintf('--- 步骤 1: 系统建模 (理想名义模型) ---\n');

% 定义理想的、用于设计的系统物理参数
J = 0.01;   % 转子转动惯量 (kg.m^2)
b = 0.1;    % 粘性摩擦系数 (N.m.s)
K_t = 0.01; % 电机转矩常数 (N.m/A)
K_e = 0.01; % 反电动势常数 (V/rad/s)
R = 1;      % 电枢电阻 (Ohm)
L = 0.5;    % 电枢电感 (H)

% 建立理想状态空间模型 (Nominal Model)
A = [0, 1, 0; 
     0, -b/J, K_t/J; 
     0, -K_e/L, -R/L];
B = [0; 0; 1/L];
C = [1, 0, 0];
D = 0;

sys_ol = ss(A, B, C, D);
fprintf('理想系统状态空间矩阵 A, B, C, D 定义完毕。\n\n');

%% ========================================================================
% 第二步: 开环系统分析
% =========================================================================
fprintf('--- 步骤 2: 开环系统分析 ---\n');
poles_ol = eig(A); fprintf('开环系统极点 (A的特征值):\n'); disp(poles_ol);
Mc = ctrb(A, B); rank_Mc = rank(Mc); fprintf('能控性矩阵的秩: %d\n', rank_Mc);
if rank_Mc == size(A,1), fprintf('结论: 系统完全能控。\n\n'); else, fprintf('结论: 系统不能控。\n\n'); return; end
figure('Name', '开环系统分析'); step(sys_ol); title('开环系统阶跃响应'); grid on; xlabel('时间 (秒)'); ylabel('输出角度 (弧度)');

%% ========================================================================
% 第三步: 控制器设计 - 方法一: 极点配置法
% =========================================================================
fprintf('--- 步骤 3: 极点配置法控制器设计 ---\n');
zeta = 0.707; wn = 4 / (zeta * 1); 
p1 = -zeta*wn + wn*sqrt(1-zeta^2)*1i; p2 = conj(p1); p3 = -5 * zeta * wn; 
poles_desired = [p1; p2; p3]; fprintf('期望配置的控制器极点:\n'); disp(poles_desired);
K1 = place(A, B, poles_desired); fprintf('极点配置法计算得到的增益 K1:\n'); disp(K1);
Nbar1 = -1 / (C * inv(A - B*K1) * B); fprintf('极点配置法的预补偿增益 Nbar1: %.4f\n\n', Nbar1);

%% ========================================================================
% 第四步: 控制器设计 - 方法二: LQR法
% =========================================================================
fprintf('--- 步骤 4: LQR法控制器设计 ---\n');
Q = diag([10, 1, 1]); R_lqr = 0.1;
K2 = lqr(A, B, Q, R_lqr); fprintf('LQR法计算得到的增益 K2:\n'); disp(K2);
Nbar2 = -1 / (C * inv(A - B*K2) * B); fprintf('LQR法的预补偿增益 Nbar2: %.4f\n\n', Nbar2);

%% ========================================================================
% 第五步: 理想控制器性能比较
% =========================================================================
fprintf('--- 步骤 5: 理想控制器性能比较与分析 ---\n');
sys_final_pp = ss(A - B*K1, B * Nbar1, C, D);
sys_final_lqr = ss(A - B*K2, B * Nbar2, C, D);
[y_lqr, t_lqr] = step(sys_final_lqr); [y_pp, t_pp] = step(sys_final_pp, t_lqr);
figure('Name', '理想控制器性能对比'); 
plot(t_pp, y_pp, 'b-', 'LineWidth', 1.5); hold on; plot(t_lqr, y_lqr, 'r--', 'LineWidth', 1.5); 
title('理想全状态反馈：极点配置法 vs LQR法'); legend('极点配置法', 'LQR法', 'Location', 'southeast'); grid on; hold off;
xlabel('时间 (秒)'); ylabel('输出角度 (弧度)');

%% ========================================================================
% 第六步: 拓展一 - 状态观测器设计与仿真
% =========================================================================
fprintf('\n--- 步骤 6: 拓展一 - 状态观测器设计与仿真 ---\n');
% 1. 分析能观性
Mo = obsv(A, C); rank_Mo = rank(Mo); fprintf('能观性矩阵的秩: %d\n', rank_Mo);
if rank_Mo == size(A,1), fprintf('结论: 系统完全能观。\n\n'); else, fprintf('结论: 系统不能观。\n\n'); return; end
% 2. 设计观测器增益 G
poles_obs = [-20; -21; -100]; fprintf('期望配置的观测器极点:\n'); disp(poles_obs);
G_T = place(A', C', poles_obs); G = G_T'; fprintf('计算得到的观测器增益 G:\n'); disp(G);
% 3. 构建包含观测器的闭环系统 (以极点配置控制器K1为例)
n = size(A, 1); A_aug = [A - B*K1, B*K1; zeros(n,n), A - G*C];
B_aug = [B*Nbar1; zeros(n,1)]; C_aug = [C, zeros(1,n)]; D_aug = 0;
sys_aug = ss(A_aug, B_aug, C_aug, D_aug);

%% ========================================================================
% 第七步: 基于观测器的控制器性能分析
% =========================================================================
fprintf('\n--- 步骤 7: 基于观测器的控制器性能分析 ---\n');
% 1. 性能对比图：理想 vs 基于观测器
[y_aug, t_aug] = step(sys_aug, t_lqr);
figure('Name', '最终性能对比：理想 vs 观测器');
plot(t_pp, y_pp, 'b-', 'LineWidth', 2); hold on; 
plot(t_aug, y_aug, 'g--', 'LineWidth', 2);
title('系统输出对比'); legend('理想全状态反馈', '基于观测器的反馈', 'Location', 'southeast'); grid on; hold off;
xlabel('时间 (秒)'); ylabel('输出角度 (弧度)');

% 2. 状态估计效果分析
t = t_lqr; u = ones(size(t)); [~, ~, x_aug] = lsim(sys_aug, u, t);
x_real = x_aug(:, 1:n); e = x_aug(:, n+1:end); x_hat = x_real - e;
figure('Name', '状态估计效果分析');
plot(t, x_real(:,2), 'b-', 'LineWidth', 2); hold on;
plot(t, x_hat(:,2), 'm:', 'LineWidth', 2.5);
title('状态估计效果：角速度 (\omega)'); legend('真实角速度', '估计角速度', 'Location', 'northeast'); grid on; hold off;
xlabel('时间 (秒)'); ylabel('角速度 (rad/s)');

%% ========================================================================
% 第八步: 拓展二 - 定义存在参数摄动的"真实"系统
% =========================================================================
fprintf('\n--- 步骤 8: 拓展二 - 控制器鲁棒性分析 ---\n');
fprintf('    -> 定义存在参数摄动的"真实"系统\n');

% 假设“真实”系统的转动惯量J增加了50%，电阻R增加了20%
J_real = J * 1.5;
R_real = R * 1.2;

% 使用新的参数构建“真实”系统的状态空间矩阵
A_real = [0, 1, 0; 
          0, -b/J_real, K_t/J_real; 
          0, -K_e/L, -R_real/L];
B_real = [0; 0; 1/L];

fprintf('“真实”系统参数: J_real = %.4f, R_real = %.4f\n', J_real, R_real);
fprintf('“真实”系统矩阵 A_real, B_real 定义完毕。\n\n');


%% ========================================================================
% 第九步: 交叉验证仿真与鲁棒性分析
% =========================================================================
fprintf('--- 步骤 9: 交叉验证仿真与鲁棒性分析 ---\n');

% 1. 使用旧控制器(K1, Nbar1)控制"真实"系统
sys_final_robust_pp = ss(A_real - B_real*K1, B_real*Nbar1, C, D);

% 2. 使用旧控制器(K2, Nbar2)控制"真实"系统
sys_final_robust_lqr = ss(A_real - B_real*K2, B_real*Nbar2, C, D);

% 3. 仿真参数失配时的响应
[y_robust_pp, t_robust] = step(sys_final_robust_pp, t_lqr); % 确保时间轴一致
[y_robust_lqr, ~] = step(sys_final_robust_lqr, t_lqr);

% 4. 绘制鲁棒性对比图
% 图5: 极点配置法鲁棒性分析
figure('Name', '鲁棒性分析: 极点配置法');
plot(t_pp, y_pp, 'b-', 'LineWidth', 2); % 理想情况
hold on;
plot(t_robust, y_robust_pp, 'c--', 'LineWidth', 2); % 参数失配情况
title('鲁棒性分析：极点配置法');
xlabel('时间 (秒)');
ylabel('输出角度 (弧度)');
legend('理想情况 (Nominal Model)', '参数失配 (Real Model)', 'Location', 'southeast');
grid on;
yline(1, 'k:', 'LineWidth', 1); % 添加目标值参考线
hold off;

% 图6: LQR鲁棒性分析
figure('Name', '鲁棒性分析: LQR法');
plot(t_lqr, y_lqr, 'r-', 'LineWidth', 2); % 理想情况
hold on;
plot(t_robust, y_robust_lqr, 'm--', 'LineWidth', 2); % 参数失配情况
title('鲁棒性分析：LQR法');
xlabel('时间 (秒)');
ylabel('输出角度 (弧度)');
legend('理想情况 (Nominal Model)', '参数失配 (Real Model)', 'Location', 'southeast');
grid on;
yline(1, 'k:', 'LineWidth', 1); % 添加目标值参考线
hold off;

% 5. 量化性能退化指标
fprintf('\n--- 性能退化量化对比 ---\n\n');
info_pp_ideal = stepinfo(sys_final_pp);
info_pp_robust = stepinfo(sys_final_robust_pp);
info_lqr_ideal = stepinfo(sys_final_lqr);
info_lqr_robust = stepinfo(sys_final_robust_lqr);

fprintf('指标\t\t\t\t理想极点配置\t\t失配极点配置\t\t理想LQR\t\t\t失配LQR\n');
fprintf('-----------------------------------------------------------------------------------------------------\n');
fprintf('调节时间(s)\t\t\t%.4f\t\t\t%.4f\t\t%.4f\t\t\t%.4f\n', info_pp_ideal.SettlingTime, info_pp_robust.SettlingTime, info_lqr_ideal.SettlingTime, info_lqr_robust.SettlingTime);
fprintf('超调量(%%)\t\t\t\t%.4f\t\t\t%.4f\t\t%.4f\t\t\t%.4f\n', info_pp_ideal.Overshoot, info_pp_robust.Overshoot, info_lqr_ideal.Overshoot, info_lqr_robust.Overshoot);
fprintf('稳态值\t\t\t\t%.4f\t\t\t%.4f\t\t%.4f\t\t\t%.4f\n', y_pp(end), y_robust_pp(end), y_lqr(end), y_robust_lqr(end));
fprintf('稳态误差(%%)\t\t\t%.2f\t\t\t%.2f\t\t%.2f\t\t\t%.2f\n', abs(1-y_pp(end))*100, abs(1-y_robust_pp(end))*100, abs(1-y_lqr(end))*100, abs(1-y_robust_lqr(end))*100);
fprintf('-----------------------------------------------------------------------------------------------------\n');
