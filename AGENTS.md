# Repository Guidelines

## 项目定位
`DeepSpace` 是基于 PrismaEngine SDK 的 C++20 航天模拟项目，当前重点是“真实任务模板 + 高拟真动力学”。最新主线包含 `Artemis II` 飞行计划与双组元推进剂消耗模型。

## 项目结构与模块组织
- `src/main.cpp`：程序入口。
- `src/DeepSpaceApp.h`：`SimulationLayer`、任务编排、自动驾驶、遥测。
- `src/core/`：基础数学与常量。
- `src/physics/`：动力学积分、空气动力学、轨道要素与轨道预判。
- `src/environment/`：行星与大气模型。
- `src/vessel/`：部件、分级、RCS、推进剂路由与推力计算。
- `DESIGN.md`、`DEVELOPMENT.md`：设计路线与技术积累。

## 构建、测试与开发命令
- `cmake -S . -B build -G Ninja`：配置工程并自动下载支持平台的 SDK。
- `cmake --build build -j`：并行构建。
- `./build/DeepSpace`：运行模拟。

注意：Windows x64 SDK 尚未发布。Windows 开发需手动提供本地 SDK 并传入 `-DPRISMA_SDK_DIR=...`。

## 代码风格与命名规范
- C++20 + 4 空格缩进。
- 类型/类/方法 `PascalCase`，成员变量 `m_` 前缀。
- 物理计算统一 `double` 与 `Vec3d`。
- 推进系统必须显式标注推进剂类型与质量比，禁止回退到“单油箱总量”写法。

## 测试规范
当前无独立 `tests/` 目录，至少完成：
- 全量构建通过。
- Artemis II 烟测：起飞、一级耗尽分离、ICPS 点火、近拱点圆轨。
- 检查遥测：`Ap/Pe`、`Thrust`、`mdot`、`fuel/ox` 流量是否连续且合理。

## 提交与 Pull Request 规范
推荐 `type: summary`：`feat:`、`fix:`、`refactor:`、`docs:`、`build:`。
- 一次提交只包含一类逻辑改动。
- PR 必须说明：变更范围、验证方式、遥测证据（日志/截图）。
- 涉及物理参数、任务模板、推进剂模型变更时，必须同步更新 `DESIGN.md` 与 `DEVELOPMENT.md`。

## 配置与仓库卫生
- 不提交构建产物与下载依赖：`build/`、`PrismaEngine-SDK/`、`*.tar.gz`、`*.so`。
- 若新增任务模板，优先以独立构建函数接入（如 `BuildArtemis2FlightPlan`），避免把构型硬编码散落在 `OnUpdate`。
