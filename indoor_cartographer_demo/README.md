# indoor_cartographer_demo

这是室内自主探索与三维建图项目的 ROS 功能包，负责提供：

- Gazebo 室内场景和地面移动机器人模型；
- Cartographer 二维激光建图配置；
- 三维点云过滤和 OctoMap 配置；
- 自主前沿探索、局部避障和移动小猫动态障碍物；
- 可选 YOLO 猫检测节点。

项目的完整安装、构建、运行命令和话题说明请查看仓库根目录的 [README.md](../README.md)。推荐从仓库根目录启动：

```bash
./run_indoor_mapping_demo.sh scene:=quick slam_mode:=hybrid
```

本功能包不包含飞行器、地空导航或真实机器人底盘驱动。
