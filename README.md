<MARKDOWN>
# Modern Control with LQR
本项目包含使用线性二次调节器(LQR)控制各种系统的MATLAB实现。LQR是一种最优控制方法，能够在给定性能指标下提供最佳控制效果。
## 项目特点
- 针对不同动态系统实现LQR控制器
- 与传统PID控制方法的对比分析
- 使用李雅普诺夫方法进行稳定性分析
- 带有详细注释的MATLAB脚本
- 包含倒立摆和无人机控制等应用示例
## 系统要求
- MATLAB R2020a或更高版本
- 控制系统工具箱(Control System Toolbox)
- (部分示例需要)优化工具箱(Optimization Toolbox)
## 使用方法
1. 克隆本仓库
2. 打开MATLAB并导航至仓库目录
3. 运行所需脚本(如`inverted_pendulum_lqr.m`)
4. 根据需要修改脚本中的参数
## 示例应用
1. **倒立摆控制**：使用LQR实现倒立摆的稳定控制
2. **无人机高度控制**：四旋翼飞行器高度稳定控制
3. **质量-弹簧-阻尼系统**：LQR与PID控制效果对比
## 文件结构
Modern_Control_with_LQR/
├── inverted_pendulum_lqr.m       # 倒立摆LQR控制
├── drone_altitude_control.m      # 无人机高度控制
├── mass_spring_damper/           # 质量-弹簧-阻尼系统
│   ├── lqr_control.m
│   └── pid_control.m
└── utils/                        # 工具函数
├── plot_results.m
└── stability_analysis.m
