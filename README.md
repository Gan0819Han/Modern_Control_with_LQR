# 现代控制理论中的LQR应用  
**使用线性二次调节器（LQR）实现最优控制系统**

[![MATLAB](https://img.shields.io/badge/MATLAB-R2020a+-blue.svg)](https://www.mathworks.com/products/matlab.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

本项目提供多种动态系统的线性二次调节器（LQR）控制实现。LQR是一种最优控制方法，能够在最小化性能指标的前提下提供最佳控制效果，特别适合多变量系统控制。

## ✨ 项目特点
- **多种系统实现**：涵盖经典力学系统与现代控制应用
- **对比分析**：LQR与传统PID控制方法的性能对比
- **稳定性保证**：基于李雅普诺夫方法的稳定性分析
- **详细文档**：完整注释的MATLAB脚本与理论背景说明
- **应用案例**：包含倒立摆、无人机等实际工程案例
- **可视化工具**：直观的结果可视化与性能指标计算

## ⚙️ 系统要求
- **MATLAB R2020a** 或更高版本
- **必需工具箱**：
  - 控制系统工具箱 (Control System Toolbox)
- **可选工具箱**（用于高级示例）：
  - 优化工具箱 (Optimization Toolbox)
  - Simulink®（部分模型仿真）

## 🚀 快速开始
1. 克隆仓库：
   ```bash
   git clone https://github.com/Gan0819Han/Modern_Control_with_LQR.git
   ```
2. 启动MATLAB并导航至项目目录：
   ```matlab
   cd Modern_Control_with_LQR
   ```
3. 运行示例脚本：
   ```matlab
   % 倒立摆控制
   run inverted_pendulum_lqr.m
   
   % 无人机高度控制
   run drone_altitude_control.m
   ```
4. 修改参数（可选）：
   打开脚本文件调整系统参数、权重矩阵等设置

## 📊 应用示例
| 系统 | 文件 | 功能 | 可视化输出 |
|------|------|------|------------|
| **倒立摆控制** | `inverted_pendulum_lqr.m` | 实现倒立摆的稳定控制 | 状态轨迹、控制输入 |
| **无人机高度控制** | `drone_altitude_control.m` | 四旋翼飞行器高度稳定控制 | 高度响应、转子推力 |
| **质量-弹簧-阻尼系统** | `mass_spring_damper/lqr_control.m` | LQR控制实现 | 位移响应、能量消耗 |
|  | `mass_spring_damper/pid_control.m` | PID控制对比 | 性能指标对比图 |

## 📂 文件结构
```plaintext
Modern_Control_with_LQR/
├── core/                        # 核心控制器实现
│   ├── lqr_design.m             # LQR控制器设计函数
│   └── stability_analysis.m     # 李雅普诺夫稳定性分析
├── examples/                    # 应用示例
│   ├── inverted_pendulum_lqr.m  # 倒立摆控制
│   ├── drone_altitude_control.m # 无人机高度控制
│   └── mass_spring_damper/      # 质量-弹簧-阻尼系统
│       ├── lqr_control.m        # LQR控制实现
│       └── pid_control.m        # PID控制实现（对比）
├── utils/                       # 实用工具函数
│   ├── plot_results.m           # 结果可视化
│   ├── system_identification.m  # 系统辨识工具
│   └── performance_metrics.m    # 性能指标计算
├── docs/                        # 文档资源
│   ├── theory_background.pdf    # 理论基础
│   └── parameter_guide.md       # 参数调整指南
└── README.md                    # 项目说明文件
```

## 📝 理论背景
LQR控制器通过最小化二次型性能指标设计最优控制律：
```math
J = \int_{0}^{\infty} \left( \mathbf{x}^T Q \mathbf{x} + \mathbf{u}^T R \mathbf{u} \right) dt
```
其中：
- `Q` = 状态权重矩阵（半正定）
- `R` = 控制输入权重矩阵（正定）
- `x` = 系统状态向量
- `u` = 控制输入向量

## 🤝 贡献指南
欢迎贡献！请遵循以下步骤：
1. Fork 项目仓库
2. 创建特性分支 (`git checkout -b feature/new-control`)
3. 提交修改 (`git commit -am 'Add new control method'`)
4. 推送分支 (`git push origin feature/new-control`)
5. 创建Pull Request

## 📜 许可证
本项目采用 [MIT 许可证](LICENSE)
