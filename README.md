# ICUAS 2027 · 自主无人机竞速

[![tests](https://github.com/E-Goose-Jerry/icuas27-racing/actions/workflows/tests.yml/badge.svg)](https://github.com/E-Goose-Jerry/icuas27-racing/actions/workflows/tests.yml)

ICUAS '27 UAV Competition = **Autonomous Drone Racing**：单架 Crazyflie 级四旋翼 + 前视相机，
全自主完成 起飞 → 按序穿过 7 个 1 m × 1 m 门 → 落地，比完赛/过门精度、用时、安全性。

**这个仓库只做「竞赛层」**：门检测、按序过门的任务状态机、计时与罚时、地面站看板、
仿真编排。飞行控制与轨迹规划**直接用官方 LASER UAV System（LUS）**，不重写。

---

## 为什么这样分层

比赛评分只看「飞完一圈多久、有没有撞」。真正需要自己写的部分只有三块：

| 要解决的问题 | 谁来做 | 在哪 |
|---|---|---|
| 「下一个该穿哪扇门、门算不算数、只剩几圈」 | 本项目 | `racing_core/mission_fsm.py` |
| 「计时 + 每次罚 5 秒」 | 本项目 | `racing_core/timing.py` |
| 「前视相机里那扇门在哪」 | 本项目 | `ros2_ws/.../gate_detector.py` |
| 「怎么飞过去（时间最优轨迹）」 | **官方 LUS** | `planning_manager` + PMM 规划器 |
| 「姿态/推力怎么跟住轨迹」 | **官方 LUS** | `control_manager`（NMPC + INDI） |
| 「PX4 怎么收指令」 | **官方 LUS** | `laser_uav_hw_api` |

所以本项目的节点只做一件事：把「下一扇门」翻译成 LUS 认识的
`/<uav>/planning_manager/trajectory_path` 消息。

---

## 快速开始

### 0. 先跑自检

```bash
cd ~/uav-racing
./tools/preflight.sh
```

它检查：课程文件能不能解析、单元测试过不过、状态目录可不可写、Docker/GPU/X11 在不在。
**全绿才继续。**

### 1. 不需要 ROS 就能玩的部分（1 分钟）

```bash
# 单元测试（纯 stdlib，149 个）
python3 -m unittest discover -s tests -t .

# 地面站看板：用合成比赛回放，不需要仿真器
python3 -m ground_station.server --mock
# 浏览器打开 http://127.0.0.1:8080
```

看板上能看到：T_adj 调整后用时、门进度、俯视地图（目标门高亮）、罚时明细、事件流。
实际效果见 `docs/dashboard-mock.png`。
这些数字全部由**真正会飞的同一套 `racing_core`** 算出来，不是手写 JSON。

### 2. 跑官方仿真（首次构建 40–60 分钟）

```bash
./sim/run_sim.sh build      # 构建 Ubuntu 22.04 + ROS 2 Humble + LUS + PX4 镜像
./sim/run_sim.sh build-race # 在里面编译竞赛包
./sim/run_sim.sh sim        # 起 tmux：Gazebo + PX4 + 估计/控制/规划 + 竞赛节点 + 看板
```

本机是 Ubuntu 24.04 + ROS 2 Jazzy，**LUS 只支持 22.04 + Humble**，
所以容器不是可选项，是唯一能不改官方代码就跑起来的路径。

### 3. 开始一次比赛

```bash
# 看板只读，命令在终端里执行（页面上也印着这几行，方便复制）
ros2 service call /icuas27/race_control icuas27_interfaces/srv/RaceControl \
  "{command: 'start', reason: 'operator'}"
```

之后：起飞 → 实际升到起飞高度后自动开表 → 逐门发航点 → 跑完自动飞到降落点并 `land`。

---

## 目录

```
racing_core/        纯 Python 竞赛逻辑，零第三方依赖，可单测
  geometry.py        向量、门坐标系、线段与门平面求交
  course.py          门/赛道数据模型、JSON 加载、校验
  timing.py          计时、罚时账本、T_adj
  mission_fsm.py     起飞→竞速→降落状态机，产出 Action
  safety.py          数据陈旧预算、越界/过低/过高判定
  protocol.py        地面站 JSON 契约（版本化）
tests/              142 个 unittest（含脚本化跑完整圈）
ros2_ws/src/
  icuas27_interfaces/  msg/srv：门观测、比赛状态、控制服务
  icuas27_racing/      race_mission（任务节点）、gate_detector（门检测）
ground_station/     stdlib HTTP + SSE + 单页看板（无构建、无依赖）
sim/                Dockerfile、非交互 bootstrap、tmux 会话、spawn/planning 配置
course/             赛道 JSON（临时样例 + 规则书形状模板）
tools/              赛道生成、起飞前检查
docs/               架构、官方接口、赛道、复用清单、踩坑、规则要点
```

---

## 验证状态（诚实版）

| 项 | 状态 |
|---|---|
| `racing_core` 单元测试 | ✅ 149 个全过（Linux，Python 3.12） |
| 地面站（mock 回放 + HTTP + SSE） | ✅ 实测可用，含真实 socket 测试 |
| ROS 包 Python 部分单测 | ✅ 22 个通过（`pytest test`） |
| **ROS 节点真跑** | ✅ 已在本机容器里起服务、写状态文件（`sim/smoke_test.sh`） |
| **仿真容器** | ✅ `icuas27-racing:local` 构建成功；官方栈 headless 跑通，里程计 100 Hz |
| **真机 / 穿门飞行** | ❌ 未验证 —— 下一步 |
| **门检测在真实画面上的效果** | ❌ **未验证** —— 颜色阈值必须对着实际画面调 |
| 真机飞行 | ❌ 未做（决赛 2027-06） |

所以：**环境已通，飞行未验**。下一步是 `./sim/run_sim.sh sim`（需要显示器）。

---

## 规则：已发布，但有五处还没定

规则书 **v1.0，2026-10-20**（页眉自称 draft）已经把场地（5 m × 5 m × 2 m、7 个 1 m × 1 m 内空 + 10 cm 边框）、
评分公式（`T_adj = T_elapsed + Σ n·5s`）、时间线写死了。**没给的**是门的具体摆法（只有概念图，
且 §5.2 说地图顺序可能从仿真到实地变化），以及四处 `[TBD]`：实地硬件/算力位置、提交表单、
决赛名额与安全阈值、尝试次数。官方 ROS 接口文档也待发（§6）。

逐条摘录与「和实现的对照」见 [docs/06-规则要点.md](docs/06-规则要点.md)。

## 关键日期

| 时间 | 事项 |
|---|---|
| 2027-01-15 | 报名截止（要填队伍仓库链接） |
| 2027-02-28 | Phase 1 代码提交截止 |
| 2027-04-01 | 公布决赛名单 |
| 2027-06-17 | 现场竞速 |

规则书：<https://uasconferences.com/2027_icuas/wp-content/uploads/ICUAS_2027_UAV_Competition.pdf>

---

## 许可

本项目代码 BSD-3-Clause。官方 LUS 与 laser_crazyflie 各自按其原许可（BSD-3）使用，
本项目**不复制**它们的源码，只在构建时拉取。
