# AGENTS.md — ICUAS 2027 竞速项目：工作区记忆

> 这是一份**工作区级记忆**。任何以本目录（或子目录）为工作区的会话，第一次请求时会自动加载它。
> 它取代"翻历史对话"：读完这份就能接着干。**状态变了就改这里**。

---

## 1. 项目是什么

- 用户：**单人参赛**，目标是拿项目经历、练全栈（机器人 + 后端 + 前端），名次不是首要目标。
- 比赛：**ICUAS '27 = Autonomous Drone Racing**（单机穿门竞速），和 '26 的多机搜救**完全不是一个任务**。
- 主办方：LASER 实验室 / 巴西帕拉伊巴联邦大学（UFPB）。
- 任务：Crazyflie 级四旋翼 + 前视相机，全自主 起飞 → 按序穿 7 个 1 m×1 m 门 → 落地。
- 评分：`T_adj = T_elapsed + Σ n_j × 5 s`（每次不完整/碰撞各罚 5 秒）。
- 规则书：**已发布**，v1.0，**Publication date 20/10/2026**，8 页，页眉自称 `Draft rulebook`。
  场地尺寸（5×5×2 m、7 个 1×1 m 内空 + 10 cm 边框、角落 1×1×0.5 m 起降台）和评分公式都在里面。
  **没给的是门的摆法**（只有概念图，且 §5.2 说地图/障碍顺序可变）。真正 `[TBD]` 只有四处：
  实地硬件与算力位置（§5.2）、提交表单（§6）、决赛名额与安全阈值（§8.2）、尝试次数（§9.2）。
  另有官方 ROS 接口文档待发（§6）。要点摘录：`docs/06-规则要点.md`。
- 关键日期：**2027-01-15 报名截止** → **2027-02-28 Phase 1 提交** → 2027-04-01 出名单 → 2027-06-17 现场竞速。
- ✅ **报名已于 2026-10-30 提交**（用户自己填的 Google 表单）。
- ✅ **仓库已于 2026-10-30 转为 public**（`E-Goose-Jerry/icuas27-racing`），匿名可访问已验过 ——
  主办方点报名表里的链接能直接看到。**不要再加密钥/私有数据进去。**
- 用户语言：**中文**。文档、回复用中文；代码、注释用英文。

## 2. 代码在哪、和别的东西什么关系

| 位置 | 是什么 |
|---|---|
| `~/uav-racing`（本目录） | **本项目唯一真源**。竞赛层全部在这里 |
| 容器内 `/opt/laser_uav_system_ws` | 官方 LUS 工作区，由 `sim/docker/bootstrap_lus.sh` 驱动官方 `install.sh` 的 CI 分支构建 |
| 本地 `~/repos/` 下的另一套无人机栈 | 团队私有仓库，**只读参考**，不复制代码 |
| `~/文档/.../imported/comp-uav-windows` | **上一版 '26 届**，已备份 + 从 GitHub 删除，只作历史 |
| `~/.dsh` 会话数据 | 别动 |

**官方仓库**：

- LUS：<https://github.com/LASER-Robotics/laser_uav_system>（分支 `stable`，BSD-3）
- Crazyflie 补丁：<https://github.com/laser-robotics-research/laser_crazyflie>

**本项目不复制官方源码**，构建时用 git 拉取（`bootstrap_lus.sh`）。理由见 `docs/04-复用清单.md`。

## 3. 当前状态（诚实版）

| 项 | 状态 |
|---|---|
| `racing_core`（几何/赛道/状态机/计时/安全/契约） | ✅ 完成，**148 个单测全过** |
| 地面站（stdlib HTTP + SSE + 看板） | ✅ 完成，**实测可跑**（`--mock` 回放 + 真实 socket 测试） |
| ROS 包（interfaces + racing + 检测器） | ⚠️ 代码完成、Python 部分 22 个测试过，**节点从未在 ROS 下运行** |
| 仿真容器 | ✅ **已构建成功**：`icuas27-racing:local`（19.8 GB），PX4 固件 / MicroXRCEAgent / acados / 35 个 ROS 包齐全 |
| 官方栈 headless 冒烟 | ✅ **通过**：`./sim/smoke_test.sh`，里程计 100.3 Hz，race 节点服务与状态文件正常 |
| 门检测在真实画面上 | ❌ 颜色阈值**完全没调过**，必须对实际画面调 |
| 真机 | ❌ 未开始（决赛 2027-06） |

**一句话：环境通了，还没飞。下一步是 `./sim/run_sim.sh sim`（要屏幕）或先跑 `./sim/smoke_test.sh`。**

看板实拍（mock 回放，第 2 圈，8/21）：`docs/dashboard-mock.png`


## 4. 三个必须知道的"事实"（别再重新挖一遍）

1. **官方 `session.yml` 里那 19 个 Race 航点不是门中心**。第 4、5 个点水平只差 0.52 m、
   高度差 3.35 m —— 两扇水平门不可能这样摆。它们是飞行路径点。
   所以 `course/provisional_sample.json` 的门朝向全是估计值（`yaw_source: "estimated"`）。
2. **`crazyflie_map.world` 里没有门**。里面是 7 个 0.2–0.6 m 的障碍箱（拆 OBJ 算包围盒验证过），
   是演示场，不是场馆。详见 `docs/03-赛道与门.md`。
3. **LUS 只支持 Ubuntu 22.04 + ROS 2 Humble**，本机是 24.04 + Jazzy。
   所以**必须走 Docker**，不要试图往 24.04 里硬装。

## 5. 设计决策（改之前先看理由）

- **不自研规划/控制**：官方 `planning_manager`（PMM 时间最优）+ `control_manager`（NMPC+INDI）
  已经能用，重写等于把有限的精力花在别人已经做好的事上。
- **`racing_core` 零第三方依赖、零 ROS**：因为它是唯一需要（也唯一能够）被单测覆盖的部分。
  **不要往里面 import rospy/numpy/cv2**。
- **地面站用 stdlib 不用 FastAPI**：要能跑在竞赛容器、Windows、手机热点三种环境。
  多一个依赖就多一处装不上。
- **看板只读**：操作走 `ros2 service call`。浏览器标签页永远不该是会让无人机起飞的东西。
- **陈旧预算按平台显式选**（`sim`/`real` 两套），**不做自动探测**。学自团队另一套栈的做法。
- **漏门默认无限重试**。规则书 §3.2：“Missing a gate requires a legal retry… skipping it does not
  count as completion.” 所以 `allow_skip_with_penalty` 默认 **False**，跳过只是测试用的显式开关。
  卡在同一扇门时 `mission.stuck` 为真，节点会打 warning。
- **计时用 ROS 时钟**（`get_clock()`），这样 `use_sim_time` 下仿真时间就是成绩时间。

## 6. 常用命令

```bash
cd ~/uav-racing

./tools/preflight.sh                       # 起飞前自检（全绿才继续）
python3 -m unittest discover -s tests -t . # 148 个单测
python3 -m pytest ros2_ws/src/icuas27_racing/test -q  # 22 个 ROS 包测试
python3 -m ground_station.server --mock    # 看板（无仿真器），:8080
python3 tools/make_courses.py              # 重新生成赛道文件

./sim/run_sim.sh check                     # 环境自检
./sim/run_sim.sh build                     # 构建镜像（慢）
./sim/run_sim.sh build-race                # 容器内编译竞赛包
./sim/run_sim.sh sim                       # 起 tmux 全栈
```

## 7. 协作约定（用户明确表达过）

- **先解释再动手**：大改动前先说方案，或给选项让他挑。
- **会问"你改了什么"** → 每次动完给逐项清单，并说明**没动什么**。
- **要有验证**：跑测试、给真实输出，不要说"应该没问题"。
- **承认错误要明确**：错了就说错，并给出正确结论。
- **命令可复制**：短路径、少参数、一次给全；需要 sudo 的单独标注。
- **⛔ git 完全由用户控制**：AI **不要**执行 `git add` / `commit` / `push` / `rm` / `checkout`。
  改完文件停手，报告"有 N 个未提交改动"。只读命令（`status`/`log`/`diff`）可以跑。
- **谈日期前先 `date`** —— 会话可能跨天，上下文里的"今天"可能是旧的。

## 8. 下一步（按优先级）

| 优先级 | 动作 | 完成判据 |
|---|---|---|
| ~~P0~~ | ~~构建镜像 + 跑通官方栈~~ | ✅ 完成（headless 冒烟全绿，见上） |
| **P0** | 在可视窗口里跑一次：arm → takeoff → 发第一个航点 | 无人机真的离地并按 `trajectory_path` 移动 |
| **P0** | 换成 crazyflie + `crazyflie_map`，确认 `--enable_ai_deck` 真的出图 | `ros2 topic hz /uav1/camera/ai_deck/image_raw` 约 30 Hz |
| **P1** | 用真赛场画面调 `gate_detector` 的 HSV 阈值 | 图片里能稳定框出 1 扇门 |
| **P1** | 端到端跑一圈 `provisional_sample` | 21 次过门全记入，`T_adj` 有数 |
| **P2** | 拿到官方场馆后填 `course/`（改 JSON，不改代码） | 校验通过、仿真里能跑完 |
| ~~P3~~ | ~~报名~~ | ✅ **2026-10-30 已提交**（比截止日早 2.5 个月） |
| **P4** | Phase 1 提交物（2027-02-28）：zip + 构建说明 + 非交互启动 + README | 干净容器里一条命令跑起来 |

## 8.5 报名之后要盯的四件事

1. ~~**仓库可见性**~~ ✅ **已解决**：2026-10-30 转为 public，匿名 GET 返回 200。
   代价：任何人可见 → 提交前必须过一遍"里面有没有密钥/个人信息"。
2. **确认邮件**：填完表单通常会有确认信；没收到就进 WhatsApp 群问（别等到 1 月）。
3. **官方 WhatsApp 群**：<https://chat.whatsapp.com/KRCA3LM7VmQ1iqsK9jcbNd>
   ——`[TBD]` 的四处（提交表单、决赛硬件、安全阈值、尝试次数）和官方接口文档只会在这里澄清。
4. **Phase 1 技术接口文档**：§6 说 topic 名/msg 类型/坐标帧/launch 入口都会写在那份文档里。
   出来后要核对我们的 `/icuas27/*` 命名，需要改的是适配层。

## 9. 未验证清单（别当成已验证）

- [x] ROS 节点在 `rclpy` 下运行（race_mission 已在容器里起服务并写状态文件）
- [x] 仿真容器构建（`icuas27-racing:local`）
- [ ] 真正起飞并穿过第一个门
- [ ] `trajectory_path` 发一次能否让 PMM 一直飞到终点（可能要在飞的过程中重发）
- [ ] `autostart.py` 与 `race_control` 的 start 命令抢解锁时序
- [ ] 门检测在 mono8 324×244 上的表现
- [ ] 3 m/s 限速在 5 m×5 m 场地是否够用（可能想更快，要先改 `agile_planner` 参数）
