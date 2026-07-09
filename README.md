# test

独立的 Windows C++17 工程，使用 wxWidgets、VTK 和 MV3D RGBD SDK，包含机器人模型、点云、相机和网络模块。

## 目录

- `src/`、`include/`：按 `camera`、`net`、`panel`、`point_cloud`、`robot_model` 拆分的源码。
- `tests/`：单元测试。
- `Resource/`：运行时资源。
- `3rd/`：第三方源码。OpenCV、state_machine 和 utils 按要求保留，但当前不默认构建。
- `build_config.ini`：本机 MV3D SDK 路径。

## 构建

在 Git Bash 中执行：

```bash
./build_on_windows.sh Release
```

脚本会按需构建 VTK，然后配置、编译 `test` 并运行测试。生成的程序位于 `build/bin/Release/test.exe`。
