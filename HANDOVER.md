# 交接文档 · HANDOVER

> **给下一个会话（或下一个你）**：读完这份就能接着干，不用翻聊天记录。
> 最后更新：**2026-10-30** 晚。写这份的原因：环境已经通了，卡在一个具体的控制问题上，
> 需要换窗口继续。

---

## 一句话现状

**环境全部打通，飞机能离地，但飞不起来。**

但**先看这条**：队友 Dimitri-Yu 已经在原生 Ubuntu 22.04 上跑通过完整短程流程
（起飞 → 虚拟检查点 → 返回 → 落地 → 解除解锁，`IDLE→TAKEOFF→RACE→LANDING→DONE`，
两次成功、2.73 s 仿真时间）。**他的做法和我们的不一样**，而且适配代码还没上传。
所以第一优先级不是调 NMPC，而是**对齐他的路径**。

他用的两条关键做法（见 `对接进度/Dimitri-Yu/2026-10-30.md`）：

1. **`lus_backend=control_manager`** —— 这个版本的官方 PMM 规划器**在 control_manager 内部**，
   他把独立的 `planning_manager` 绕开了（原话："保留独立 planning_manager 的原默认路径，但后者尚未在本机实测"）。
   → 我们卡住的正是那条"未实测"的路径。
2. **仿真真值适配** —— 他遇到的问题是"接地后估计高度向下漂移"，
   和我这边 z 从 1.21 掉到 -600 是**同一个现象**（MEKF 估计发散）。他改用 Gazebo 真值跑通。

**他还确认了**：相机可用（324×244 mono8，两轮各 1302/1276 帧）；
场景里确实是 **7 个门框**；未修改任何控制增益与机体参数。

---

## 30 秒上手

```bash
cd ~/uav-racing

./tools/preflight.sh          # 体检：课程文件/测试/Docker/GPU/X11
./sim/bringup.sh --visual     # 起全套（含 Gazebo 窗口），约 90 秒
```

`bringup.sh` 会自动打好两个上游补丁、按正确顺序启动、并检查生命周期状态。
跑完会打印「怎么起飞 / 怎么看 / 怎么关」。

然后：

```bash
docker exec -it sim bash -c '
  source /opt/ros/humble/setup.bash
  source /opt/icuas27_racing/ros2_ws/install/setup.bash
  ros2 service call /icuas27/race_control icuas27_interfaces/srv/RaceControl \
    "{command: start, reason: operator}"'

# 看状态
docker exec sim cat /tmp/icuas27_race/status.json
```

---

## 已验证 ✅（都有证据，不是"应该没问题"）

| 项 | 证据 |
|---|---|
| 仿真镜像 | `icuas27-racing:local`（19.8 GB），PX4 SITL / MicroXRCEAgent / acados / 35 个 ROS 包齐全 |
| 官方栈 headless | `./sim/smoke_test.sh` 10 项全绿，**里程计 100.3 Hz** |
| 官方栈带窗口 | Gazebo 窗口正常，场地里是 **7 个门框**（`docs/shots/gazebo-arena.png`） |
| 竞赛层编译 | `./sim/run_sim.sh build-race` → 2 packages finished |
| 我的任务节点 | `/icuas27/race_control` 在线，状态文件正常写出 |
| 起飞 | `start` → arm → takeoff 生效，**确实离地到 1.21 m**，计时开跑 |
| 逻辑层测试 | `python3 -m unittest discover -s tests -t .` → **151 个全过** |
| CI | GitHub Actions 三个 job 全绿 |

## 未验证 / 失败 ❌

| 项 | 现象 |
|---|---|
| **悬停** | 起飞机动结束后 **8 秒内从 1.21 m 沉到 0.69 m，随后发散**（z→-600）。**不是悬停失败，是根本没有控制器接管** |
| **轨迹跟踪** | 规划器拒绝一切航点（`fresh control state and odometry in the same frame are required`），原因见下 |
| **过门** | 一次都没穿过 |
| 门检测 | 颜色阈值仍是默认值，**一帧真实画面都没见过** |
| 镜像可复现性 | 当前镜像是容器 `commit` 出来的，**Dockerfile 还没用干净构建验证过** |

---

## ⚠️ 版本不一致（他明确要求核对的第一件事）

| 依赖 | 本仓库镜像 | Dimitri-Yu 验证过的 | |
|---|---|---|---|
| LUS 根仓库 | `b72dbc1bbf6e434f014ab0e93cd2b7c410c992fc` | `ef820aa947dacefe50650872dac72685961ace3a` | ⚠️ **不同** |
| `laser_uav_managers` | `770afe444bdb984cc90809c9c12d3281e408bf16` | `7a1964bdd01d6bda6bcd40abcd1252428ed1431b` | ⚠️ **不同** |
| `laser_uav_controllers` | `fca756ad4c2113077ca4000b3103af787b92eeeb` | — | — |
| `laser_uav_planners` | `8aa841c1676601d73cfaa1873685dd6947a4204c` | — | — |
| `laser_crazyflie` | `05f55a5cff530c12dccfcf17a288d7a5ea02b258` | `05f55a5cff530c12dccfcf17a288d7a5ea02b258` | ✅ 相同 |

我们的镜像是 `git clone -b stable` 当时的最新，他是固定到某个 commit。**管理器的控制代码不同，
这本身就可能解释"他能飞、我们不能"。** 复核版本应当是第一步，而不是继续调参。

## 当前阻塞：控制层（可能是版本/后端选择问题，不一定是算法问题）

两个独立缺陷，详细分析在 **`docs/07-环境与仿真排障日志.md` §14.6**，这里只放结论：

### 缺陷 A：INDI 内环没有电机转速反馈（上游已知）

`control_manager` 订阅 `/uav1/hw_api/motor_speed_estimated`（`MotorSpeedStamped`），
但**这个话题不存在**。上游 README 自己写明：`px4_api` 发的是不带时间戳的 `MotorSpeed`，
**类型不兼容，remap 救不了**。→ INDI 的 drag/thrust 估计失效。

### 缺陷 B：NMPC（acados）第一次求解就失败

```
SQP_RTI: QP solver returned error status 3 QP iteration 10.
multirotor_ode_acados_solve() failed with status 4.
```

单次运行 **3215 次**。后果是死循环：

```
NMPC 失败 → ready_for_tracking=false
        → planning_manager 拒绝航点（control_is_fresh() 要求 ready_for_tracking）
        → 没有参考轨迹 → NMPC 更算不出来
```

### 三条可选路线（**新会话第一件事就是问用户选哪条**）

| 路线 | 做法 | 代价 | 我的建议 |
|---|---|---|---|
| **0. 对齐队友** | 找 Dimitri-Yu 要那份适配（`lus_backend=control_manager` + 真值适配），并把 LUS/managers 固定到他的 commit | 等他把适配传上来；或自己复刻 | **最高优先级**。他已经验证过能飞，我们没验证的是那条路 |
| **A. 调 NMPC** | `R: 1000 → 5`，`N: 20 → 10`，看 QP 是否收敛 | 改 yaml 重启，10 分钟见结果 | 只在路线 0 走不通时试 |
| **B. 换控制模式** | `angular_rates_and_thrust`，绕开逐电机分配 | 要改 `px4_api` 让它订阅该话题 | 动上游，1–2 小时 |
| **C. 绕开 NMPC** | 用 PX4 自带位置/速度控制器（offboard setpoint）+ 自写轨迹跟踪 | 新写控制节点 + 桥 | A 不成就走这条：**竞速比的是稳定飞到门，不是用没用 NMPC** |

参数现状（调参起点）：质量 `0.025 kg`、惯量 `1.6e-5`、`c_thrust=1.28e-8`、
`Q=[40,20,0.3,1,0.3,0.3]`、`R=1000`、`N=20`、`dt=0.05`，模式 `individual_thrust`。

---

## 环境事实（改不动的前提）

| 项 | 值 |
|---|---|
| 宿主机 | Ubuntu 24.04 + ROS 2 Jazzy，**装不了 Humble**，必须走容器 |
| 镜像 | `icuas27-racing:local`（19.8 GB）+ `icuas27-base:local`（5.65 GB，缓存用） |
| 容器内路径 | LUS 在 `/opt/git/laser_uav_system`，工作区 `/opt/laser_uav_system_ws`，本项目挂到 `/opt/icuas27_racing` |
| **`HOME` 必须是 `/opt`** | LUS 脚本写死 `~/git/laser_uav_system` |
| **运行态必须 root** | PX4 要查 passwd（宿主 uid 没记录会 `no matching password record`）；编译态用宿主 uid（否则产物 root 所有删不掉） |
| **`--device /dev/dri` 必需** | 不给的话 gzclient 用软件渲染吃 900% CPU，把物理挤到 1 Hz |
| 环境变量 | 全部集中在 `sim/docker/icuas27_env.sh`，通过 `BASH_ENV` 让每个非交互 shell 自动加载 |
| 磁盘 | 66 GB 可用 |
| 残留容器 | 无（都已清理） |

---

## 已提交的 commit

| SHA | 内容 |
|---|---|
| `53472e4` | 能跑的镜像 + headless 冒烟测试 |
| `a577db7` | 环境与仿真排障日志（13 个坑） |
| `7a35305` | 两份上手文档（agent 版 + 人类版 PDF） |
| `2551356` | 起飞阶段的修复 + 交接文档（变基到队友的 `375a75d` 之上） |
| `375a75d` 等 3 个 | **Dimitri-Yu** 的对接进度文档（他的验证结果就在这里） |

## 本次会话的最后一批改动（**都在本地，等你决定是否提交**）

| 文件 | 改了什么 |
|---|---|
| `racing_core/mission_fsm.py` | 新增 `target_action()`：供"持续重发当前目标"用，**不重置过门判定记忆** |
| `ros2_ws/.../race_node.py` | ① 订阅 `hw_api/diagnostics`（之前 `_last_fcu_at` 永远是 None，离地即误判 `FCU_STALE` 中止）② 加 `fcu_grace`（启动宽限期，避免"还没收到心跳"被当成"心跳过期"）③ 每秒重发当前门目标（规划器会拒绝单次发布） |
| `tests/test_mission_fsm.py` | 两个新测试：重发不改变状态、`target_action` 在非 RACE 返回 None（**151 个测试全过**） |
| `sim/bringup.sh` | **新增**：一键起全套，自动打两个上游补丁 + 按顺序启动 + 校验生命周期 |
| `docs/07` / `AGENTS.md` | 加了 §14.5/§14.6（首飞与控制层分析），并把"稳定悬停"这个**错误表述**改掉 |
| `HANDOVER.md` | **新增**：本文件 |

⚠️ **`takeoff` 请求排队重试还没做完**：现在节点在服务未就绪时仍然会丢请求
（首飞时就是因此重来了一次）。`bringup.sh` 起来后手动发 `start` 通常没事（服务已就绪），
但这是个隐患，记在 `docs/07` §15 待办里。

---

## 别再犯的错（我这一轮犯过的，按痛苦程度排序）

1. **说"稳定悬停"** —— 只看前 6 个采样点就下结论，实际 8 秒后就在下沉。**看数据要看完整段。**
2. **手抄官方安装脚本** —— 漏了 PX4 固件、DDS Agent、acados 三件要命的东西。正解是驱动官方 `install.sh` 的 CI 分支（`GITHUB_ACTIONS=true`）。
3. **`REAL_UAV` 写成裸 `false`** —— 会静默走真机分支：多装 18 个驱动、**跳过 PX4 编译**、还编不过。必须写成带引号的 `"false"`（构建期），运行期要裸值。
4. **`.bashrc` 在容器里等于没有** —— 非交互 shell 直接 return。所有环境变量收进 `BASH_ENV`。
5. **`pkill -f "control_manager"`** —— 把同批 launch 的兄弟节点一起带走了。按 PID 杀。
6. **`ENV` 写在 Dockerfile 顶部** —— 改一行让 500 多个包重装。
7. **`docker commit` 不加 `--change ENTRYPOINT`** —— 镜像配置会被重置。
8. **只看"话题存在"就以为链路通** —— 要看 `ros2 topic info` 的 publisher count 和 `ros2 topic hz`。
9. **`--group px4` 当 gitman 参数** —— 它要位置参数：`gitman install simulation px4 --force`。

全部细节（现象/根因/修法/验证命令）在 `docs/07`，别凭记忆改。

---

## 下一步 checklist

- [ ] **先读 `对接进度/` 目录**（队友在这里写进展），再看本文件
- [ ] **问用户选 0/A/B/C 哪条路线**（默认建议 0：对齐队友）
- [ ] 找 Dimitri-Yu 要适配代码，或复刻 `lus_backend=control_manager` + 真值适配
- [ ] 把 LUS 与 managers 固定到队友验证过的 commit，复核能否复现
- [ ] 试 A：改 `laser_uav_controllers/params/nmpc_controller/crazyflie.yaml` 的 `R` 与 `N`，重启 `control_manager`，看 `QP solver` 是否还报错
- [ ] 判据：看板出现 `state=TRACKING`、`waypoint_count>0`、**高度能稳住 30 秒不下沉**
- [ ] 补 `takeoff` 请求排队重试
- [ ] 起一帧真实相机画面，调 `gate_detector` 的 HSV 阈值（`--enable_ai_deck` 是否出图都还没验证）
- [ ] 跑通一圈：`provisional_sample` 21 次过门
- [ ] Dockerfile 干净构建验一次（Phase 1 要求）
- [ ] 从 `obstacles_crazyrun.obj` 解析真实门位，替换估计赛道

---

## 文档地图

| 想知道什么 | 看哪 |
|---|---|
| **队友做到哪了** | `对接进度/README.md` 与各成员目录（**新会话先看这个**） |
|---|---|
| 项目是什么、为什么这样分层 | `README.md` |
| 规则书要点、`[TBD]` 有哪些 | `docs/06-规则要点.md` |
| 环境/仿真踩过的 15 个坑 | `docs/07-环境与仿真排障日志.md` |
| 逻辑层的坑（状态机/计时/判定） | `docs/05-踩坑与教训.md` |
| 复用了谁的东西、为什么不复用别的 | `docs/04-复用清单.md` |
| 赛道与门的现状 | `docs/03-赛道与门.md` |
| 官方栈话题/服务表 | `docs/02-官方栈接口.md` |
| 新机器从零搭 | `docs/08`（agent 版）/ `docs/pdf/新系统搭建手册.pdf`（人类版） |
| 项目长期记忆 | `AGENTS.md` |
