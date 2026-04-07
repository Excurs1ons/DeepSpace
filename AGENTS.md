# Repository Guidelines

## 项目定位
`DeepSpace` 是基于 Mock 引擎接口的 C++20 航天模拟项目，当前主线是 `Artemis II` 任务模板与高拟真推进系统（双组元、分级、轨道机动、事件驱动遥测）。最终将迁移到真实 PrismaEngine SDK。

## 项目结构与模块组织
- `src/main.cpp`：程序入口（Mock驱动）。
- `src/DeepSpaceApp.h`：`SimulationLayer`、任务事件机、自动驾驶、遥测。
- `src/core/`：基础数学与常量。
- `src/physics/`：动力学积分、空气动力学、轨道要素与轨道预判。
- `src/environment/`：行星与大气模型。
- `src/vessel/`：部件、分级、RCS、推进剂路由、节流控制。
- `src/engine/`：**Mock引擎接口层**（待替换为真实SDK）。
- `src/telemetry/`：遥测日志系统。
- `src/ui/`：HUD界面组件。
- `DESIGN.md`、`DEVELOPMENT.md`：设计路线与技术积累。

## 开发策略：Mock-First 引擎接口

### 核心原则
1. **Mock优先**：所有引擎接口先用Mock实现，确保功能完整后再切换真实SDK
2. **逆推引擎需求**：通过Mock接口的使用方式，反推PrismaEngine需要提供的底层接口
3. **无平台依赖**：Mock实现在任何平台都能编译运行，无需SDL3/OpenGL等依赖

### Mock引擎接口清单
```
src/engine/
├── MockEngine.h          # Mock引擎主类
├── MockInputManager.h    # 键盘/手柄输入
├── MockWindow.h         # 窗口管理（文本模式）
├── MockLogger.h         # 日志输出
├── MockSceneManager.h   # 场景管理（简化版）
├── MockAudioManager.h   # 音频（仅日志输出）
└── MockRenderer.h       # 渲染（占位）
```

### 切换时机
- 项目核心功能完成（发射→入轨→任务事件）
- Mock接口覆盖所有使用场景
- 通过Mock使用方式编写`ENGINE_REQUIREMENTS.md`

## 构建、测试与开发命令
```bash
# Mock模式（默认，无需依赖）
cmake -S . -B build -G Ninja
cmake --build build -j
./build/DeepSpace

# 真实SDK模式（可选）
cmake -S . -B build -G Ninja -DUSE_MOCK_ENGINE=OFF
cmake --build build -j
./build/DeepSpace
```

## 并行开发规范

### 关联性判断标准
| 关联等级 | 定义 | 并行可行性 |
|---------|------|-----------|
| **高关联** | 修改同一文件、同一模块、共享数据结构 | ❌ 禁止并行 |
| **中关联** | 不同文件但同一模块、有接口依赖 | ⚠️ 需协商接口 |
| **低关联** | 不同模块、无共享状态 | ✅ 可并行 |

### 并行开发许可
- ✅ `physics/` 与 `vessel/` 可并行（独立物理域）
- ✅ `telemetry/` 与 `environment/` 可并行（无共享）
- ❌ `ui/` 与 `DeepSpaceApp.h` 不可并行（共享SimulationLayer）
- ❌ `engine/mock/` 与 `simulation/` 不可并行（接口依赖）

### 冲突解决
1. 每个代理开发独立功能模块
2. 接口类定义需先于实现（接口先行原则）
3. 共享数据结构放在 `src/core/`

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
