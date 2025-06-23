%% --- 现代控制理论课程设计：直流电机位置伺服系统（含多项拓展） ---
%
% 本脚本将完成以下任务:
% 1. 建立理想直流电机的状态空间模型.
% 2. 分析开环系统的稳定性和能控性.
% 3. 设计连续时间的极点配置控制器与LQR控制器.
% 4. 仿真并比较两种理想控制器性能.
% 5. (拓展一) 设计连续时间的Luenberger状态观测器并仿真其性能.
% 6. (拓展二) 分析控制器在模型参数失配时的鲁棒性.
% 7. (拓展三) 在噪声环境下，设计并对比离散Luenberger观测器与卡尔曼滤波器性能.
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

sys_c = ss(A, B, C, D); % Continuous-time system
fprintf('理想连续系统状态空间矩阵 A, B, C, D 定义完毕。\n\n');


%% ========================================================================

% --- 开环分析 ---
fprintf('--- 步骤 2: 开环系统分析 ---\n');
poles_ol = eig(A); fprintf('开环系统极点 (A的特征值):\n'); disp(poles_ol);
Mc = ctrb(A, B); rank_Mc = rank(Mc); fprintf('能控性矩阵的秩: %d\n', rank_Mc);
if rank_Mc == size(A,1), fprintf('结论: 系统完全能控。\n\n'); else, fprintf('结论: 系统不能控。\n\n'); return; end
figure('Name', '开环系统分析'); step(sys_c); title('开环系统阶跃响应'); grid on; xlabel('时间 (秒)'); ylabel('输出角度 (弧度)');

% --- 连续控制器设计 ---
fprintf('--- 步骤 3 & 4: 连续控制器设计 ---\n');
% Pole Placement
zeta = 0.707; wn = 4 / (zeta * 1); 
p1 = -zeta*wn + wn*sqrt(1-zeta^2)*1i; p2 = conj(p1); p3 = -5 * zeta * wn; 
poles_desired_c = [p1; p2; p3];
K_pp_c = place(A, B, poles_desired_c);
Nbar_pp_c = -1 / (C * inv(A - B*K_pp_c) * B);
% LQR
Q = diag([10, 1, 1]); R_lqr = 0.1;
K_lqr_c = lqr(A, B, Q, R_lqr);
Nbar_lqr_c = -1 / (C * inv(A - B*K_lqr_c) * B);

% --- 理想连续控制器性能比较 ---
fprintf('--- 步骤 5: 理想连续控制器性能比较 ---\n');
sys_pp_c = ss(A - B*K_pp_c, B * Nbar_pp_c, C, D);
sys_lqr_c = ss(A - B*K_lqr_c, B * Nbar_lqr_c, C, D);
[y_lqr_c, t_c] = step(sys_lqr_c); [y_pp_c, ~] = step(sys_pp_c, t_c);
figure('Name', '理想控制器性能对比'); 
plot(t_c, y_pp_c, 'b-', 'LineWidth', 1.5); hold on; plot(t_c, y_lqr_c, 'r--', 'LineWidth', 1.5); 
title('理想全状态反馈：极点配置法 vs LQR法'); legend('极点配置法', 'LQR法'); grid on; hold off; xlabel('时间 (秒)'); ylabel('输出角度 (弧度)');

% --- 连续Luenberger观测器设计 ---
fprintf('--- 步骤 6 & 7: 连续Luenberger观测器分析 ---\n');
poles_obs_c = [-20; -21; -100];
G_T = place(A', C', poles_obs_c); G_c = G_T';
n = size(A,1);
A_aug_c = [A - B*K_pp_c, B*K_pp_c; zeros(n,n), A - G_c*C];
B_aug_c = [B*Nbar_pp_c; zeros(n,1)];
C_aug_c = [C, zeros(1,n)];
sys_aug_c = ss(A_aug_c, B_aug_c, C_aug_c, 0);
[y_aug_c, ~] = step(sys_aug_c, t_c);
figure('Name', '最终性能对比：理想 vs 连续观测器');
plot(t_c, y_pp_c, 'b-'); hold on; plot(t_c, y_aug_c, 'g--');
title('系统输出对比'); legend('理想全状态反馈', '基于Luenberger观测器的反馈'); grid on; hold off;

% --- 鲁棒性分析 ---
fprintf('--- 步骤 8 & 9: 鲁棒性分析 ---\n');
J_real = J * 1.5; R_real = R * 1.2;
A_real = [0, 1, 0; 0, -b/J_real, K_t/J_real; 0, -K_e/L, -R_real/L];
B_real = [0; 0; 1/L];
sys_robust_pp = ss(A_real - B_real*K_pp_c, B_real*Nbar_pp_c, C, D);
sys_robust_lqr = ss(A_real - B_real*K_lqr_c, B_real*Nbar_lqr_c, C, D);
figure('Name', '鲁棒性分析: 极点配置法');
[y_robust_pp, t_robust] = step(sys_robust_pp, t_c);
plot(t_c, y_pp_c, 'b-'); hold on; plot(t_robust, y_robust_pp, 'c--');
title('鲁棒性分析：极点配置法'); legend('理想情况', '参数失配'); grid on;
figure('Name', '鲁棒性分析: LQR法');
[y_robust_lqr, ~] = step(sys_robust_lqr, t_c);
plot(t_c, y_lqr_c, 'r-'); hold on; plot(t_robust, y_robust_lqr, 'm--');
title('鲁棒性分析：LQR法'); legend('理想情况', '参数失配'); grid on;


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

% kalman函数需要一个明确定义了噪声输入通道的模型。
% 我们构建一个增广模型，其输入为 [u; w], 其中w是过程噪声。
n = size(Ad, 1);
B_aug_kalman = [Bd, eye(n)]; % B矩阵增广，分别为控制输入u和过程噪声w的输入矩阵
D_aug_kalman = [Dd, zeros(size(Cd,1), n)]; % D矩阵对应增广

% 创建用于卡尔曼滤波器设计的系统模型
sys_for_kalman = ss(Ad, B_aug_kalman, Cd, D_aug_kalman, Ts);

% 使用这个新的增广模型来设计卡尔曼滤波器
[kalman_filter, ~, ~] = kalman(sys_for_kalman, Qn, Rn);
fprintf('卡尔曼滤波器设计完毕。\n\n');

%% ========================================================================
% 第十二步: 搭建噪声环境下的手动闭环仿真
% =========================================================================
fprintf('--- 步骤 12: 搭建噪声环境下的手动闭环仿真 ---\n');

sim_steps = 1500; % 仿真步数 (1500 * 0.01s = 15s)
t_sim = (0:sim_steps-1) * Ts; % 仿真时间轴
n = size(Ad,1); % 系统阶数
r = ones(1, sim_steps); % 单位阶跃参考输入

% --- 仿真1: 使用Luenberger观测器 ---
x_true_L = zeros(n, sim_steps);      % 真实状态
x_hat_L = zeros(n, sim_steps);       % Luenberger估计状态
y_out_L = zeros(1, sim_steps);       % 系统输出
u_L = zeros(1, sim_steps);         % 控制输入

for k = 1:sim_steps-1
    w = sqrt(Qn) * randn(n,1); % 过程噪声
    v = sqrt(Rn) * randn;     % 测量噪声
    
    u_L(k) = -Kd_lqr * x_hat_L(:,k) + Nbar_lqr_c * r(k);
    x_true_L(:,k+1) = Ad * x_true_L(:,k) + Bd * u_L(k) + w;
    y_measured = Cd * x_true_L(:,k) + v; 
    y_out_L(k) = Cd * x_true_L(:,k); 
    x_hat_L(:,k+1) = Ad * x_hat_L(:,k) + Bd * u_L(k) + Gd_luenberger * (y_measured - Cd * x_hat_L(:,k));
end
y_out_L(sim_steps) = Cd * x_true_L(:,sim_steps); 

% --- 仿真2: 使用Kalman滤波器 ---
x_true_K = zeros(n, sim_steps);      % 真实状态
x_hat_K = zeros(n, sim_steps);       % Kalman估计状态
y_out_K = zeros(1, sim_steps);       % 系统输出
u_K = zeros(1, sim_steps);         % 控制输入

% 从kalman_filter对象中提取其状态空间矩阵
kalman_A = kalman_filter.A;
kalman_B = kalman_filter.B;

for k = 1:sim_steps-1
    w = sqrt(Qn) * randn(n,1);
    v = sqrt(Rn) * randn;
    
    u_K(k) = -Kd_lqr * x_hat_K(:,k) + Nbar_lqr_c * r(k);
    x_true_K(:,k+1) = Ad * x_true_K(:,k) + Bd * u_K(k) + w;
    y_measured = Cd * x_true_K(:,k) + v;
    y_out_K(k) = Cd * x_true_K(:,k);
    
    % *** 错误修正 ***
    % kalman滤波器的状态就是对系统状态的最优估计x_hat。
    % 其状态更新方程直接给出下一个最优估计值。
    kalman_input = [u_K(k); y_measured];
    x_hat_K(:,k+1) = kalman_A * x_hat_K(:,k) + kalman_B * kalman_input;
end
y_out_K(sim_steps) = Cd * x_true_K(:,sim_steps);

fprintf('两组噪声环境下的仿真完成。\n\n');

%% ========================================================================
% 第十三步: 噪声环境下性能对比与分析
% =========================================================================
fprintf('--- 步骤 13: 噪声环境下性能对比与分析 ---\n');

% 1. 绘制状态估计性能对比图 (角速度 x2)
figure('Name', '状态估计性能对比 (噪声环境)');
plot(t_sim, x_true_K(2,:), 'k-', 'LineWidth', 2.5);
hold on;
plot(t_sim, x_hat_L(2,:), 'c--', 'LineWidth', 1.5);
plot(t_sim, x_hat_K(2,:), 'm-', 'LineWidth', 1.5);
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
plot(t_sim, r, 'k:', 'LineWidth', 1);
title('最终输出性能对比');
xlabel('时间 (秒)');
ylabel('输出角度 (弧度)');
legend('基于Luenberger控制', '基于Kalman控制', '参考输入', 'Location', 'southeast');
grid on;

% 3. 量化估计误差
rmse_luenberger = sqrt(mean((x_true_L(2,:) - x_hat_L(2,:)).^2));
rmse_kalman = sqrt(mean((x_true_K(2,:) - x_hat_K(2,:)).^2));

fprintf('角速度估计的均方根误差 (RMSE):\n');
fprintf('  - Luenberger 观测器: %.6f\n', rmse_luenberger);
fprintf('  - Kalman 滤波器:    %.6f\n', rmse_kalman);
