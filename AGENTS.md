# Repository Guidelines

## 项目定位
`DeepSpace` 是基于 PrismaEngine SDK 的 C++20 航天模拟项目，当前主线是 `Artemis II` 任务模板与高拟真推进系统（双组元、分级、轨道机动、事件驱动遥测）。

## 项目结构与模块组织
- `src/main.cpp`：程序入口。
- `src/DeepSpaceApp.h`：`SimulationLayer`、任务事件机、自动驾驶、遥测。
- `src/core/`：基础数学与常量。
- `src/physics/`：动力学积分、空气动力学、轨道要素与轨道预判。
- `src/environment/`：行星与大气模型。
- `src/vessel/`：部件、分级、RCS、推进剂路由、节流控制。
- `DESIGN.md`、`DEVELOPMENT.md`：设计路线与技术积累。

## 构建、测试与开发命令
- `cmake -S . -B build -G Ninja`：配置并自动下载支持平台 SDK。
- `cmake --build build -j`：并行构建。
- `./build/DeepSpace`：运行模拟。

注意：Windows x64 SDK 尚未发布。Windows 环境需手动提供 SDK 并传入 `-DPRISMA_SDK_DIR=...`。

## 代码风格与命名规范
- C++20 + 4 空格缩进。
- 类型/类/方法 `PascalCase`，成员变量 `m_` 前缀。
- 物理计算统一 `double` 与 `Vec3d`。
- 推进系统必须显式声明推进剂类型与 O/F 质量比，禁止使用“单油箱总量”近似。
- 任务逻辑集中在独立函数（如 `BuildArtemis2FlightPlan`、`ManageMissionEvents`），避免把流程散落在 `OnUpdate`。

## 测试规范
当前无独立 `tests/` 目录，至少完成：
- 全量构建通过。
- Artemis II 烟测：起飞、Max-Q 节流、主级分离、ICPS 圆轨、Orion 接管。
- 检查遥测：`Ap/Pe`、`q`、`Thrust`、`mdot`、`fuel/ox` 流量、阶段剩余推进剂。

## 提交与 Pull Request 规范
推荐 `type: summary`：`feat:`、`fix:`、`refactor:`、`docs:`、`build:`。
- 每次提交只包含一类逻辑改动。
- PR 必须写明：变更范围、验证方式、关键遥测证据（日志/截图）。
- 涉及飞行计划、推进剂模型、物理参数时，必须同步更新 `DESIGN.md` 与 `DEVELOPMENT.md`。

## 配置与仓库卫生
- 不提交构建产物与下载依赖：`build/`、`PrismaEngine-SDK/`、`*.tar.gz`、`*.so`。
- 新增任务模板时，复用 `PartLibrary` 工厂函数，不在更新循环里动态创建部件。
