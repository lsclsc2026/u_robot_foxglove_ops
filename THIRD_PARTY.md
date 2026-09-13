# 第三方与许可说明

本次只保存既有来源，不为整个项目新增开源许可，不改变任何来源的授权条件。私有仓库发布也不代表原作者额外授予再分发或商业使用权。文件级声明、各包 `package.xml` 和既有 LICENSE 应结合阅读。

## 保留的第三方实现

| 路径 | 来源/用途 | 现有许可信息 |
|---|---|---|
| `code/mos_all_test_ws/src/navigation2/` | 完整 Navigation2 本地快照，包含可能的本地修改；保留源码、测试、文档、模型与地图，未替换为上游下载版本 | [根声明](code/mos_all_test_ws/src/navigation2/LICENSE)列有 LGPL-2.1-or-later、Apache-2.0、BSD-3-Clause 及组合，具体以各包和文件为准 |
| `code/mos_all_test_ws/src/navigation2/nav2_mppi_controller/` | MPPI 控制器实现 | 保留 [MIT LICENSE](code/mos_all_test_ws/src/navigation2/nav2_mppi_controller/LICENSE.md)与版权声明 |
| ROS `mos_*`、`lumos_nav` 包 | 本地业务/厂商集成来源，不据名字推断原创归属 | 逐包声明见[许可索引](docs/provenance/package-licenses.tsv) |
| `code/mos_all_test_ws/src/*/weights/` | 视觉与车筐识别模型权重，原位保留 | 未附单独完整模型许可、训练数据授权或训练配方；不推断为通用开源权重 |
| `reference/` | 历史运维、迁移和保护实现 | 保留来源声明；不能认定所有文件均使用同一许可 |

`mos_3d_object`、`mos_cart_fullness`、`mos_grasp_selector`、`mos_route_manager` 的包声明为 `Proprietary`；`mos_coordinator` 仍为 `TODO: License declaration`。`lumos_nav` 声明 MIT，部分控制和流水线包及 `mos_ops_control` 声明 Apache-2.0。包声明不补足未随源提供的版权或完整许可文件，本次没有臆造这些内容。

第三方代码可能包含本地修改；源归档不含可核对的完整 Git 历史，因此不能确认准确上游 commit 或逐项本地补丁。逐文件来源和整理变动见[来源说明](docs/provenance.md)。

## 环境依赖

ROS 2 Humble、Foxglove 客户端/Bridge、OpenCV、cv_bridge、DDS、厂商 Lumos SDK/HAL、SLAM 应用和 Docker 等还受各自许可约束。仓库仅包含来源快照实际保存的文件，不暗示打包了这些依赖的完整发行内容或许可证集合。部署时应沿用目标环境已有依赖及其许可。
