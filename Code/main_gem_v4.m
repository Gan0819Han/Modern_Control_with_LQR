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

sys_c = ss(A, B, C, D);
fprintf('理想系统状态空间矩阵 A, B, C, D 定义完毕。\n\n');

%% ========================================================================
% 第二步: 开环系统分析
% =========================================================================
fprintf('--- 步骤 2: 开环系统分析 ---\n');
poles_ol = eig(A); fprintf('开环系统极点 (A的特征值):\n'); disp(poles_ol);
Mc = ctrb(A, B); rank_Mc = rank(Mc); fprintf('能控性矩阵的秩: %d\n', rank_Mc);
if rank_Mc == size(A,1), fprintf('结论: 系统完全能控。\n\n'); else, fprintf('结论: 系统不能控。\n\n'); return; end
figure('Name', '开环系统分析'); step(sys_c); title('开环系统阶跃响应'); grid on; xlabel('时间 (秒)'); ylabel('输出角度 (弧度)');

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
poles_obs_c = [-20; -21; -100]; fprintf('期望配置的观测器极点:\n'); disp(poles_obs_c);
G_T = place(A', C', poles_obs_c); G = G_T'; fprintf('计算得到的观测器增益 G:\n'); disp(G);
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


%% ========================================================================
% 第十步: 拓展三 - 系统离散化与离散控制器/观测器设计
% =========================================================================
fprintf('\n--- 步骤 10: 拓展三 - 系统离散化与设计 ---\n');
Ts = 0.01; % 定义采样周期 (s)
sys_d = c2d(sys_c, Ts, 'zoh'); % 将连续系统离散化
Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;
fprintf('系统已离散化，采样周期 Ts = %.3f s\n', Ts);

% 设计离散时间的控制器 (以LQR为例，因其鲁棒性更好)
Kd_lqr = dlqr(Ad, Bd, Q, R_lqr);
fprintf('离散LQR控制器增益 Kd_lqr 计算完毕。\n');

% 设计离散时间的Luenberger观测器，用于对比
% 将连续观测器极点映射到离散域: z = exp(p*Ts)
poles_obs_d = exp(poles_obs_c * Ts);
Gd_T = place(Ad', Cd', poles_obs_d);
Gd_luenberger = Gd_T';
fprintf('离散Luenberger观测器增益 Gd_luenberger 计算完毕。\n\n');

%% ========================================================================
% 第十一步: 定义噪声特性并设计卡尔曼滤波器
% =========================================================================
fprintf('--- 步骤 11: 定义噪声特性并设计卡尔曼滤波器 ---\n');
% 过程噪声协方差 Qn (我们相信状态[角度,角速度,电流]受扰动的程度)
% 数值是可调参数，反映了对模型的不信任程度
Qn = diag([1e-6, 1e-4, 1e-2]); 
% 测量噪声协方差 Rn (我们相信传感器测量的准确度)
% 数值是可调参数，反映了对测量的不信任程度
Rn = 0.1; 

% 使用MATLAB内置函数设计卡尔曼滤波器
% 它会自动计算最优的卡尔曼增益
[kalman_filter, ~, ~] = kalman(sys_d, Qn, Rn);
fprintf('卡尔曼滤波器设计完毕。\n\n');

%% ========================================================================
% 第十二步: 搭建噪声环境下的手动闭环仿真
% =========================================================================
fprintf('--- 步骤 12: 搭建噪声环境下的手动闭环仿真 ---\n');

sim_steps = 1500; % 仿真步数 (1500 * 0.01s = 15s)
t_sim = (0:sim_steps-1) * Ts; % 仿真时间轴
n = size(Ad,1); % 系统阶数

% --- 仿真1: 使用Luenberger观测器 ---
x_true_L = zeros(n, sim_steps);      % 真实状态
x_hat_L = zeros(n, sim_steps);       % Luenberger估计状态
y_out_L = zeros(1, sim_steps);       % 系统输出
u_L = zeros(1, sim_steps-1);       % 控制输入

for k = 1:sim_steps-1
    % 生成噪声
    w = sqrt(Qn) * randn(n,1); % 过程噪声
    v = sqrt(Rn) * randn;     % 测量噪声
    
    % 计算控制输入 (基于上一时刻的估计)
    u_L(k) = -Kd_lqr * x_hat_L(:,k);

    % 更新"真实"系统状态
    x_true_L(:,k+1) = Ad * x_true_L(:,k) + Bd * u_L(k) + w;
    
    % 得到带噪声的测量输出
    y_measured = Cd * x_true_L(:,k+1) + v;
    y_out_L(k+1) = y_measured - v; % 记录不含噪声的真实输出用于绘图
    
    % 更新Luenberger观测器状态
    x_hat_L(:,k+1) = Ad * x_hat_L(:,k) + Bd * u_L(k) + Gd_luenberger * (y_measured - Cd * x_hat_L(:,k));
end

% --- 仿真2: 使用Kalman滤波器 ---
x_true_K = zeros(n, sim_steps);      % 真实状态
x_hat_K = zeros(n, sim_steps);       % Kalman估计状态
y_out_K = zeros(1, sim_steps);       % 系统输出
u_K = zeros(1, sim_steps-1);       % 控制输入

% Kalman滤波器模型有4个输入: [u; y_measured]
kalman_A = kalman_filter.A;
kalman_B = kalman_filter.B;
kalman_C = kalman_filter.C;
kalman_D = kalman_filter.D;

for k = 1:sim_steps-1
    % 生成噪声 (使用相同统计特性，但为独立随机序列)
    w = sqrt(Qn) * randn(n,1);
    v = sqrt(Rn) * randn;
    
    % 计算控制输入
    u_K(k) = -Kd_lqr * x_hat_K(:,k);
    
    % 更新"真实"系统状态
    x_true_K(:,k+1) = Ad * x_true_K(:,k) + Bd * u_K(k) + w;
    
    % 得到带噪声的测量输出
    y_measured = Cd * x_true_K(:,k+1) + v;
    y_out_K(k+1) = y_measured - v;
    
    % 更新Kalman滤波器状态
    kalman_input = [u_K(k); y_measured];
    x_hat_K(:,k+1) = kalman_A * x_hat_K(:,k) + kalman_B * kalman_input;
end

fprintf('两组噪声环境下的仿真完成。\n\n');

%% ========================================================================
% 第十三步: 噪声环境下性能对比与分析
% =========================================================================
fprintf('--- 步骤 13: 噪声环境下性能对比与分析 ---\n');

% 1. 绘制状态估计性能对比图 (角速度 x2)
figure('Name', '状态估计性能对比 (噪声环境)');
plot(t_sim, x_true_K(2,:), 'k-', 'LineWidth', 2.5); % 真实角速度 (以Kalman仿真中的为准)
hold on;
plot(t_sim, x_hat_L(2,:), 'c--', 'LineWidth', 1.5); % Luenberger估计
plot(t_sim, x_hat_K(2,:), 'm-', 'LineWidth', 1.5); % Kalman估计
title('状态估计性能对比：角速度 (\omega)');
xlabel('时间 (秒)');
ylabel('角速度 (rad/s)');
legend('真实值', 'Luenberger 估计', 'Kalman 估计', 'Location', 'northeast');
grid on;

% 2. 绘制最终输出性能对比图
figure('Name', '最终输出性能对比 (噪声环境)');
plot(t_sim, y_out_L, 'c--', 'LineWidth', 1.5);
hold on;
plot(t_sim, y_out_K, 'm-', 'LineWidth', 1.5);
title('最终输出性能对比');
xlabel('时间 (秒)');
ylabel('输出角度 (弧度)');
legend('基于Luenberger控制', '基于Kalman控制', 'Location', 'southeast');
grid on;

% 3. 量化估计误差
rmse_luenberger = sqrt(mean((x_true_L(2,:) - x_hat_L(2,:)).^2));
rmse_kalman = sqrt(mean((x_true_K(2,:) - x_hat_K(2,:)).^2));

fprintf('角速度估计的均方根误差 (RMSE):\n');
fprintf('  - Luenberger 观测器: %.6f\n', rmse_luenberger);
fprintf('  - Kalman 滤波器:    %.6f\n', rmse_kalman);
