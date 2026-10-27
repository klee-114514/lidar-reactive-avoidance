# 激光雷达反应式局部避障

一套可在 MCU、FPGA 或小型嵌入式平台上实现的二维激光雷达局部避障方案。

算法不依赖 ROS、SLAM 或全局地图，主要解决：

- 前方障碍触发；
- 左、右绕行方向选择；
- 按车体尺寸检查候选方向；
- 贴边跟随障碍；
- 回到原任务航线；
- 传感器、转向和电机故障停车。

## 适用平台

- 差速轮式或可原地转向的移动机器人；
- 能提供角度和距离的二维激光雷达；
- 能提供航向的 IMU、陀螺仪、电子罗盘或其他定位模块；
- 能提供实测行进距离的编码器；
- 能执行“走向指定航向”和“原地转向指定航向”的运动控制器。

不要求特定雷达品牌。

算法只要求雷达适配层最终提供 72 扇区距离、正前净空和左右侧净空。

仓库中的 FPGA 示例使用 RPLIDAR A1 协议，但 C 参考控制器与雷达协议无关。

## 完整说明

移植、测量、参数填写和运行顺序全部写在：

- [完整避障设计与移植说明](docs/完整避障设计与移植说明.md)

## 代码

- [可移植 C 参考实现](reference/c/lidar_reactive_avoidance.c)
- [C 接口定义](reference/c/lidar_reactive_avoidance.h)
- [最小调用示例](reference/c/example.c)
- [MCU 适配层模板](platform/mcu-template/lidar_avoidance_adapter.h)
- [车辆标定表](config/vehicle-calibration.csv)
- [配置填写示例](config/lra_vehicle_config.example.h)
- [扫描回放示例](examples/scan-replay)
- [FPGA 示例工程](platform/fpga-gowin/lidar_avoidance.gprj)

## 核心流程

```text
空闲
  -> 检测障碍
  -> 选择方向
  -> 检查车体包络
  -> 原地转向
  -> 小步贴边跟随
  -> 判断已经越过障碍
  -> 返回原航线
  -> 转回原航向
  -> 恢复任务
```

## 使用前提

使用前必须手动测量并填写车辆尺寸、雷达安装位置、最小安全距离和转向参数。

不能直接复制另一台车的参数。

仓库提供扫描回放示例、MCU 适配模板和标定表。

扫描示例和适配模板只作为移植参照。

部署到实际设备前，必须完成参数标定、安全距离设置和独立急停验证。

真实设备必须配置独立急停和电机堵转保护。

## 许可证

项目原创代码使用 MIT License。

正式授权以英文 [LICENSE](LICENSE) 为准。

中文参考译文见 [LICENSE.zh-CN.md](LICENSE.zh-CN.md)。
