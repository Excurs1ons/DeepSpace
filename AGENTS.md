# Repository Guidelines

## 项目定位
`DeepSpace` 是基于 PrismaEngine SDK 的 C++20 航天模拟项目，目标是实现从发射、分级到入轨的高拟真飞行流程。代码贡献的核心原则是：先保证物理正确性，再追求功能扩展与表现层优化。

## 项目结构与模块组织
- `src/main.cpp`：程序入口，负责初始化引擎并运行应用。
- `src/DeepSpaceApp.h`：应用与 `SimulationLayer`，包含主循环中的飞行逻辑。
- `src/core/`：基础类型与常量（如 `Vec3d`、物理常量）。
- `src/physics/`：动力学积分、空气动力学、轨道要素计算。
- `src/environment/`：天体参数与大气模型。
- `src/vessel/`：部件系统、燃料与推力、级间分离、RCS 控制。
- `DESIGN.md`、`DEVELOPMENT.md`：分别记录设计路线与技术演进。

新增功能请优先落在对应子系统目录，不要把业务逻辑直接堆到 `main.cpp`。

## 构建、测试与开发命令
- `cmake -S . -B build -G Ninja`：配置工程并自动下载 PrismaEngine SDK。
- `cmake --build build -j`：并行构建 `DeepSpace`。
- `./build/DeepSpace`（Linux）或 `build\\DeepSpace.exe`（Windows）：运行本地版本。
- `ctest --test-dir build --output-on-failure`：执行自动化测试（新增测试目标后启用）。

若修改 SDK 版本，请同步更新 `CMakeLists.txt` 中版本与下载地址，并在 PR 中说明验证平台。

## 代码风格与命名规范
- 使用 C++20，统一 4 空格缩进。
- 类型、类、方法使用 `PascalCase`；成员变量使用 `m_` 前缀。
- 常量使用 `constexpr` 并集中在 `DeepSpace::Constants`。
- 物理计算统一 `double` + `Vec3d`，避免轨道尺度精度漂移。
- 复杂计算可加简短注释解释公式来源或建模假设，避免“说明显而易见代码”的注释。

## 测试规范
当前仓库尚无独立 `tests/` 目录，贡献时至少完成：
- 全量编译通过。
- 运行烟测，验证起飞、分级、推力响应与遥测输出（Alt/Vel/Ap/Pe）。
- 对物理逻辑改动，提供改动前后关键数值对比（日志或截图）。

新增自动化测试建议：
- 路径：`tests/`。
- 命名：`XxxTests.cpp`（如 `OrbitalElementsTests.cpp`）。
- 接入：在 CMake 注册并通过 CTest 运行。

## 提交与 Pull Request 规范
Git 历史中已出现 `refactor: ...` 与 `Add ...` 两类风格；建议统一使用 `type: summary`：
- 示例：`fix: correct stage fuel routing`。
- 常用类型：`feat`、`fix`、`refactor`、`docs`、`build`。
- 每次提交只做一件事，避免把重构与行为变更混在一起。

PR 需包含：
- 变更目的、影响模块、风险点。
- 构建与运行证据（命令 + 关键输出）。
- 关联 issue/任务；若改动物理行为，附遥测日志或对比截图。

## 配置与仓库卫生
- 不要提交构建产物与下载依赖：`build/`、`PrismaEngine-SDK/`、`*.so`、`*.tar.gz`（见 `.gitignore`）。
- 涉及物理常量、推进参数、气动模型的改动，必须在 `DEVELOPMENT.md` 记录理由与影响范围，便于后续回归。
