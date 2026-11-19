# LHandPro 1.4.5 模型显示约定

STL 与 YAML 来自已安装的 `/opt/lhandpro/model/DH116`，原文件未改，摘要见 `manifest.json`。用于本机 V3 界面，不是控制或碰撞模型。

2026-11-15 对照厂商程序的模型显示函数核对了两个容易误读的约定：

- `RobotWidget::setModelAngle`（地址 `0x196620`）：按 `drive_axis` 选择 `drive_dir` 的 X、Y 或 Z 分量，把角度乘该分量后加到对应的初始 Euler 分量。三个分量不能相乘。
- `GLView::eulerAnglesToRotationMatrix`（地址 `0x172d90`）：列主序矩阵为 `Rz(-z) Ry(-y) Rx(-x)`。与 scipy 的 `Rotation.from_euler('xyz', -rotation)` 对应。
- `GLView::buildLocalTransform`（地址 `0x1731e0`）：平移、上述旋转、缩放，再沿 `parent_link` 层级组合。

这些是只读核对所得的显示兼容性事实，没有修改或执行厂商控制程序。使用正角 Euler 会让手指朝手背弯曲；把 `drive_dir` 分量相乘还会错误放大拇指远端角度。初版离线截图已暴露问题，交付版已修正。

V3 当量换算仍为六轴 `60/30/80/80/80/80` 度每 10000 eq。模型用于交互和反馈显示，不作为手指接触几何或运动精度标定。
