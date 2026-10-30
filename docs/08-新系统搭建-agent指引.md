# 从零搭建：agent 执行指引

> **读者**：一个刚接手这台机器、没有历史上下文的 AI agent。
> **目标**：把一台**刚装完的 Ubuntu** 变成"能跑 ICUAS 2027 竞速仿真"的状态，并且每一步都**可验证**。
> **原则**：只做本文写的事；每一步都有 `CHECK`，CHECK 不过就停下报告，不要"猜着往下走"。

---

## 0. 硬性约束（先读，违反会造成不可逆损坏）

| 禁止 | 原因 |
|---|---|
| 往 Ubuntu 24.04 上直接装 `ros-humble-*` | Humble 只有 22.04 的包；强行装会污染系统。**必须走 Docker** |
| 执行 `git add` / `commit` / `push` / `rm` / `checkout` | 版本库由人类控制。改完文件就停手，报告"有 N 个未提交改动" |
| 未经人类同意就启动 GUI 程序（Gazebo / gzclient） | 新窗口会抢焦点，打断人类正在做的事 |
| `sudo` 交互式命令 | 需要密码，非交互会挂住。凡需 root 的步骤，输出命令让人类自己执行 |
| `pkill -f <模式>` | 会匹配到执行它的 shell。按 PID 杀，或用 `pkill -f "a[.]b"` 打断自匹配 |
| 把 `.secrets/`、token、私钥写进仓库或日志 | 仓库是 public |

**只读命令随时可跑**：`git status -sb`、`git log --oneline`、`docker ps`、`df -h`。

---

## 1. 前置检查（一条命令判断"这台机器还要不要从头搭"）

```bash
date; uptime -p
lsb_release -ds; uname -r
nproc; free -g | head -2; df -h / | tail -1
command -v docker && docker info >/dev/null 2>&1 && echo "docker OK"
docker images --format '{{.Repository}}:{{.Tag}}' | grep icuas27 || echo "无 icuas27 镜像 → 需要构建"
ls -d ~/uav-racing 2>/dev/null && echo "项目已在" || echo "项目不在 → 需要克隆"
```

判定：

| 输出 | 含义 | 去哪一步 |
|---|---|---|
| 有 `icuas27-racing:local` 且 `~/uav-racing` 存在 | 已就绪 | 直接 §7 冒烟测试 |
| 有项目、无镜像 | 只缺镜像 | §5 构建 |
| 都没有 | 从零 | §2 开始 |

---

## 2. 系统基线

```bash
# 期望：Ubuntu 24.04 / 32 核 / 62 GB / 磁盘剩余 >= 40 GB
nproc; free -g | sed -n 2p; df -h / | tail -1
```

需要满足的下限（低于此值会失败在具体某一步，提前说清楚比事后 debug 便宜）：

| 资源 | 下限 | 不够会怎样 |
|---|---|---|
| 磁盘可用 | **40 GB** | 镜像 19.8 GB + 构建缓存 13 GB，中途 `No space left on device` |
| 内存 | 16 GB | PX4 编译 `cc1plus` 被 OOM kill |
| CPU | 8 核 | 构建时间从 45 分钟涨到 2 小时以上 |

`CHECK`：

```bash
[ "$(df --output=avail -BG / | tail -1 | tr -dc 0-9)" -ge 40 ] && echo "磁盘 OK" || echo "磁盘不足，先清理"
```

**清理建议（需人类同意）**：`docker system df` 看 `RECLAIMABLE`；`docker image prune`、`docker volume prune -f`。

---

## 3. Docker 与 GPU

```bash
command -v docker || echo "需要安装 Docker"
docker info >/dev/null 2>&1 || echo "daemon 未启动 / 用户不在 docker 组"
docker run --rm --gpus all ubuntu:22.04 nvidia-smi -L 2>&1 | head -2
```

- 第 1、2 行有输出 → **停下来**，把 `sudo` 命令给人类执行（安装 Docker、加组、`newgrp docker`）。
- 第 3 行能看到 GPU 型号 → GPU 直通可用。看不到也不阻塞：仿真会退回 CPU 渲染（慢但能跑）。

`CHECK`：

```bash
docker info >/dev/null 2>&1 && echo "DOCKER_OK"
```

---

## 4. 拿到项目

```bash
cd ~ && [ -d uav-racing ] || git clone git@github.com:E-Goose-Jerry/icuas27-racing.git uav-racing
cd ~/uav-racing
git status -sb | head -2
ls
```

期望顶层：`AGENTS.md course docs ground_station pyproject.toml racing_core README.md ros2_ws sim tests tools`

**先读 `AGENTS.md`**：它是这个项目的记忆（当前状态、已踩的坑、下一步）。读完再动手。

`CHECK`：

```bash
cd ~/uav-racing && ./tools/preflight.sh | tail -6
```

`preflight.sh` 的判据是：课程文件能解析、单测全过、状态目录可写、Docker/GPU/X11 在不在。
**`[fail]` 必须先解决；`[warn]` 可以继续但要记录。**

---

## 5. 构建仿真镜像（最慢的一步，45–60 分钟）

```bash
cd ~/uav-racing
nohup ./sim/run_sim.sh build > /tmp/icuas27_build.log 2>&1 &
```

**不要在前台等，也不要加 `timeout`**。用后台 + 轮询：

```bash
tail -f /tmp/icuas27_build.log            # 观察
grep -oE "^#[0-9]+ \[ *[0-9]+/10\]" /tmp/icuas27_build.log | tail -1   # 当前阶段
docker images | grep icuas27               # 唯一的成功判据
```

阶段与耗时（供判断"是不是卡了"）：

| 阶段 | 内容 | 耗时 | 正常表现 |
|---|---|---|---|
| 3/10 | apt 装 500 多个包 | 8–15 分钟 | 一直在 `Get:`，WiFi 慢时会长时间无输出 |
| 5/10 | 官方 `install.sh`（gitman 拉 20+ 仓库） | 5–10 分钟 | `Cloning into ...` |
| 6/10 | **PX4 SITL 编译** | 10–20 分钟 | 大量 `[ 42%] Building CXX` |
| 7/10 | Micro-XRCE-DDS-Agent + acados | 3–6 分钟 | cmake 输出 |
| 8/10 | `colcon build` 35 个 ROS 包 | 10–20 分钟 | `Starting >>> ...` |

**⚠️ 判断构建失败的最快方法**：

```bash
docker images --format '{{.Repository}}:{{.Tag}}' | grep icuas27
```

没有镜像就是失败 —— 即使命令"跑完了"。`docker build` 失败不留镜像，只留缓存层。
**"命令结束"不等于"构建成功"。**

失败后按 `docs/07-环境与仿真排障日志.md` 的编号对照（那份文档就是为这一刻写的）：

| 日志里的报错 | 查 § |
|---|---|
| `Unable to locate package gazebo11` | 07 §1 |
| `Could not find a package configuration file provided by "ament_cmake"` | 07 §2 |
| `AMENT_TRACE_SETUP_FILES: unbound variable` | 07 §3 |
| `Could not find LIVOX_LIDAR_SDK_LIBRARY` / 没编 PX4 | 07 §4 |
| 构建成功但运行没里程计 | 07 §5 |
| `no matching password record` / `/tmp/git/...` 找不到 | 07 §6 |

---

## 6. 编译竞赛包（20 秒）

```bash
cd ~/uav-racing && ./sim/run_sim.sh build-race
```

期望末行：`Summary: 2 packages finished`

`CHECK`：

```bash
docker run --rm icuas27-racing:local bash -c 'ros2 pkg prefix icuas27_racing' | grep -q icuas27 && echo "OK"
```

---

## 7. 冒烟测试（**这就是"搭好了"的判据**）

```bash
cd ~/uav-racing && ./sim/smoke_test.sh
```

必须 10 项全 `[ok]`：

```
[ok] icuas27_racing built
[ok] laser_uav_managers built
[ok] gzserver running
[ok] PX4 SITL process running
[ok] MicroXRCEAgent running (uXRCE-DDS bridge)
[ok] topic /uav1/hw_api/odometry
[ok] topic /uav1/estimation_manager/estimation
[ok] topic /uav1/control_manager/state
[ok] topic /uav1/planning_manager/diagnostics
[ok] odometry: average rate: ~100
[ok] race_control service advertised
[ok] status document written
```

`odometry` 那项是**整条链路的证据**：gzserver → PX4 → uXRCE-DDS → px4_api → estimation_manager，
任何一环断了这里就没有频率。它比"话题存在"可靠得多。

失败时进容器看日志（`--keep` 保留现场）：

```bash
./sim/smoke_test.sh --keep
docker exec -it smoke bash
# 日志：/tmp/gzserver.log /tmp/spawn.log /tmp/px4_api.log /tmp/estimation_manager.log /tmp/race.log
```

---

## 8. 可视仿真（**必须先问人类**）

会弹 Gazebo 窗口并占满屏幕。

```bash
./sim/smoke_test.sh --visual      # 需要 DISPLAY / Xauthority
```

如果 gzclient 起不来，按顺序查（`docs/07` §12）：

1. `Authorization required` → 挂 `~/.Xauthority`
2. `Failed to query drm device` → `--device /dev/dri`
3. `Unable to find shader lib` → `GAZEBO_RESOURCE_PATH` 没设（要 `source /usr/share/gazebo/setup.bash`）

---

## 9. 起飞（**必须有人类在旁边**）

```bash
docker exec smoke bash -c '
  set +u; source /opt/ros/humble/setup.bash; source /opt/laser_uav_system_ws/install/setup.bash; set -u
  ros2 service call /icuas27/race_control icuas27_interfaces/srv/RaceControl \
    "{command: '"'"'start'"'"', reason: '"'"'operator'"'"'}"'
```

会依次：arm → takeoff 到 1 m → 开计时 → 发第一个门的三段航点。
**第一次跑之前确认**：Gazebo 窗口可见、`smoke_test` 全绿、人类在场。

---

## 10. 完成判据（Definition of Done）

全部满足才能说"环境搭好了"：

- [ ] `./tools/preflight.sh` 全绿
- [ ] `python3 -m unittest discover -s tests -t .` → `OK`（149 个）
- [ ] `./sim/run_sim.sh build-race` → `2 packages finished`
- [ ] `./sim/smoke_test.sh` → 10 项全 `[ok]`，里程计约 100 Hz
- [ ] `docker images | grep icuas27-racing` 有镜像
- [ ] 磁盘剩余 ≥ 10 GB

---

## 11. 交接给人类时要报告什么

按这个格式，不要只贴日志：

```
1. 现在能做：<已验证的边界，例如"官方栈 headless 跑通，里程计 100 Hz">
2. 现在不能做：<未验证的部分，例如"还没起飞过">
3. 改了哪些文件：<逐项>
4. 没动什么：<例如"没碰 git，没改系统配置">
5. 待人类决定：<例如"是否开始可视仿真（会占屏）">
6. 有 N 个未提交改动
```

---

## 12. 真出问题时的顺序

1. `docs/07-环境与仿真排障日志.md` —— 先按编号对症状，90% 的坑都在里面
2. `docs/05-踩坑与教训.md` —— 逻辑层的坑
3. `AGENTS.md` —— 当前状态与"下一步"（也可能是"这条已经试过了"）
4. 都没有 → 用 `docker exec` 进容器复现，先确认是**环境**问题还是**代码**问题，再动手
