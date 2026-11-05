# P3D 外部源码修复提案

适用 resources 提交：4aae76618ce819c532accdb8da271aae92aca352。仅供组长审核，启动脚本不自动应用差异，未覆盖当前飞行环境。

根因：P3D实现引用了 gazebo_plugins 包的头文件，本包却已有 laser_gazebo_resources/gazebo_ros_p3d.hpp。应引用自身头文件；旧CMake额外依赖只绕过了错误引用。

验证：移除旧额外include后原源码复现失败，一行源码改正后语法编译通过。从锁定提交git archive导出的隔离源码，仅改这一行，P3dPluginGazebo完整编译链接成功，并发2，无gazebo_plugins额外依赖。静态方块位置(0.25,-0.4,1)加偏移(0,0,2)，实测(0.25,-0.4,3)、frame=world；SIGINT退出时server_exit=0、probe_exit=0。没有PX4或飞行。

失败边界：首次FindMAVLink配置失败，显式指定现有/opt/ros/humble/include后成功。首次SIGTERM退出出现ROS context失效；额外捕获context异常的候选未解决退出超时，因此未采用。最终恢复为仅一行头文件修复并使用SIGINT正常关闭，不能声称插件源码已修复SIGTERM竞争。

完整本地证据在logs/p3d-source-evidence-20261102/。原失败、未采用候选和最终成功分别保留。请组长确认外部仓库维护分支后再接入统一构建，不能盲目应用到其他版本。此提案不证明历史估计器异常、旧spawn挂起、较新planning_manager、实体七门或视觉导航通过。
