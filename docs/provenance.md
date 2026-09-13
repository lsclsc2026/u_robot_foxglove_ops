# 来源与发布整理清单

## 来源层级

整理日期为 2026-09-13。直接来源是本地只读目录 `/home/unitree/foxglove_handoff_20260913`；其原始索引记录了更早 `/home/shuochen/learning` 工作目录。源目录未被本次整理修改。

| 发布目录 | 原归档中的来源含义 |
|---|---|
| `code/mos_fleet_monitor` | 本地监控工作副本，优先于历史 reference，但不等同已部署实机 |
| `code/mos_all_test_ws` | ROS 源码、依赖和权重快照；原位保留 |
| `reference/mos_ops` | 历史 Git 整理版本 |
| `reference/lumos87_monitor_migration_20260812` | 2026-08-12 迁移快照 |
| `reference/_mos_vision_guard`、`reference/mos_ops_guard` | 历史保护工具 |
| `reference/mos_fleet_monitor` | 历史部署记录；tar.gz 副本未发布 |

地址、机器名和原绝对路径保留用于追溯，新环境使用者需自行配置。没有根据名称或时间戳断言远端部署状态。

## 可审阅清单

- [逐文件发布来源](provenance/published-source-map.tsv)：每个来源文件的发布路径、直接来源、归档 SHA-256、发布 SHA-256 和整理状态。
- [排除文件](provenance/excluded-files.tsv)：9 个历史压缩副本；不删除本地原件，没有排除未压缩源码。
- [大文件清单](provenance/large-files.tsv)：所有至少 1 MiB 的依赖/权重/媒体文件，均保留在原仓库路径；没有源码依赖移往 Release。
- [各包许可声明](provenance/package-licenses.tsv)：从保留的 package.xml 摘录，供定位原文件，不是重新许可。
- [原始来源映射](provenance/original-SOURCE_MAP.tsv)、[原始索引](provenance/original-FILE_INDEX.txt)、[原始校验](provenance/original-SHA256SUMS)：只描述输入归档，不能直接用于本仓库根目录完整性检查。
- [原归档说明](provenance/original-README.md)：完整保留为历史上下文，不是当前安装手册。

## 整理规则

保留业务脚本、ROS 源码、参数默认值、依赖结构、权重、许可证及原有文件模式。新增中文入口与文档、根 `.gitignore`、许可/来源索引；仅在有易混淆部署内容的原有文档顶部追加历史提示。旧索引/校验文件未更新为新版本；其历史语义必须与本次索引区分。

不发布 `.git`、build/install/log、缓存、运行 PID、私钥/密码/token 和压缩重复副本。此次扫描没有发现需要删除的真实认证凭据；不把历史 IP、用户名或机器名误当秘密替换业务默认值。

README 封面和 Release 原速视频由用户提供，媒体信息见 [video-manifest.json](media/video-manifest.json)。Release 只承载演示媒体，权重和第三方网格等仍直接包含在 Git 仓库中。仓库整理版本标签 `v0.1.0-review` 不替换源代码 VERSION。

## 检查范围

只做发布文件尺寸、敏感凭据模式与新增文档相对链接检查；不执行编译、功能测试或机器人操作。来源哈希用于识别复制内容与文档注记，不构成运行正确性证明。旧文档可能含历史外链、绝对本机路径或已排除压缩包名，保留为历史记录。
