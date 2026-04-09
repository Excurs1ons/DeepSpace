# DeepSpace 项目设计文档 (Game Design Document)

本设计文档基于《坎巴拉太空计划》(Kerbal Space Program, KSP) 的完整玩法循环，结合"微软飞行模拟级别"的高拟真物理目标，规划 `DeepSpace` 的开发路线。

---

## 1. 核心玩法循环 (Core Gameplay Loop)

KSP 的魅力在于"设计 -> 试错 -> 飞行 -> 探索"的闭环。DeepSpace 继承该循环，并强化真实任务流程。

### 1.1 航天器设计 (VAB / SPH)
- **玩法**：在类似 VAB 的装配界面中，使用模块化部件拼装火箭与上面级。
- **机制**：
  - **基于节点捕捉**：部件通过附着点连接。
  - **实时指标**：TWR、总 Delta-V、CoM、CoT、CoL。
- **DeepSpace 特色**：支持真实推进剂组合（RP-1/LOX、LH2/LOX、MMH/NTO）与任务模板。

### 1.2 任务规划与发射 (Mission Planning & Launch)
- **玩法**：将载具推上发射台，执行自动/手动姿态与分级控制。
- **机制**：
  - **发射时间配置**：可设置具体发射时间（UTC），系统根据发射时间计算太阳位置、发射窗口等。
  - **发射地点选择**：KSC、Baikonur、Wuhan 等发射场，各地有不同的初始气象条件。
  - **天气系统**：温度、湿度、气压、风速、风向等参数影响大气密度、发动机性能。
  - **重力转弯**：在大气层内按高度进行俯仰程序。
  - **动态节流**：Max-Q 区间自动降油门，过峰后恢复。
  - **分级**：推进剂耗尽后分离级间结构并点火上面级。
- **当前任务模板**：`Artemis II`（SLS Block 1 发射 -> ICPS 圆轨 -> Orion 服务舱接管窗口）。

### 1.3 轨道机动与领航 (Orbital Execution & Piloting)
- **机制**：
  - 近拱点圆轨自动点火（可切换）。
  - 轨道预测：短时真空积分预估 Ap/Pe。
  - RCS 平移与姿态稳定用于精调。
  - 任务事件：ICPS 稳定窗口、TEI 准备窗口、Orion 推进系统接管。

### 1.4 探索与经营 (Exploration & Career)
- 后续扩展：月球转移、交会对接、科研与合同体系。

---

## 2. 引擎架构映射 (Engine Architecture Mapping)

### 2.1 物理与环境层 (Physics & Environment)
- `PhysicsBody`：双精度积分与姿态更新。
- `OrbitalMechanics`：轨道根数解算与轨道预判。
- `Planet`/`Atmosphere`：重力、大气压与密度模型（采用公开 ISA 分层大气参数，Mach 由局部声速计算）。
- `WeatherSystem`：天气模拟层，支持从发射时间计算气象条件。

### 2.2 载具系统 (Vessel Systems)
- `Part` / `PartLibrary`：部件模型与任务组件库。
- `EnginePart`：支持燃料+氧化剂双组元质量比（O/F）。
- `FuelTankPart`：按推进剂类型分仓。
- `Vessel::Update`：按 `Stage + PropellantType` 路由推进剂消耗。
- `Vessel::SetStageThrottle`：支持按阶段动态节流。

### 2.3 交互与表现层 (UI & Rendering)
- `SimulationLayer`：任务编排、输入、自动驾驶与遥测输出。
- 遥测新增：总流量、燃料流量、氧化剂流量、阶段推进剂余量、动态压强。

---

## 3. 发射计划与天气系统 (Launch Planning & Weather)

### 3.1 发射时间配置

```yaml
launch:
  datetime: "2026-04-09T18:00:00Z"  # ISO 8601 格式
  timezone: "UTC"
  window:
    start: "2026-04-09T17:00:00Z"
    end: "2026-04-09T22:00:00Z"
  auto_calculate_window: true  # 自动计算最优发射窗口
```

**功能**：
- 根据发射时间计算太阳位置（光照条件）
- 自动计算发射窗口（轨道相位最优时间）
- 发射时间影响初始温度、季节性气候

### 3.2 发射地点配置

```yaml
launch_site:
  name: "Kennedy Space Center"
  location:
    latitude: 28.5721
    longitude: -80.6480
    altitude_m: 3
  timezone: "America/New_York"
  pad: "LC-39A"
```

**支持的发射场**：
| 发射场 | 坐标 | 典型气象 |
|--------|------|----------|
| KSC LC-39A | 28.57°N, 80.65°W | 温暖潮湿 |
| Baikonur | 45.96°N, 63.56°E | 干旱内陆 |
| Jiuquan | 40.96°N, 100.26°E | 沙漠气候 |
| Cape Canaveral | 28.39°N, 80.60°W | 海岸气候 |

### 3.3 天气系统配置

```yaml
weather:
  enabled: true  # 天气影响开关
  real_time_data: false  # 是否使用真实天气API
  parameters:
    temperature_C: 25.0    # 温度 (°C)
    humidity_pct: 70.0     # 相对湿度 (%)
    pressure_hPa: 1013.25  # 气压 (hPa)
    wind_speed_ms: 5.0   # 风速 (m/s)
    wind_direction_deg: 180  # 风向 (°)
    cloud_cover_pct: 30   # 云量 (%)
  variation:
    enabled: true
    random_seed: 12345  # 可复现的随机天气
```

### 3.4 天气对模拟的影响

| 天气参数 | 影响机制 | 影响对象 |
|-----------|----------|----------|
| **温度** | 液氧/液氢蒸发率 | 低温推进剂储箱 |
| **气压** | 大气密度 ρ = P/RT | 动压 q = 0.5ρv² |
| **湿度** | 湿空气密度修正 | 高湿度降低空气密度 |
| **风速** | 侧向力、滚转力矩 | 姿态控制、导航 |
| **云量** | 太阳辐射衰减 | 热力学模型 |

**影响计算示例**：
```cpp
// 湿空气密度修正
double GetAirDensity(double pressure_hPa, double temperature_C, double humidity_pct) {
    double satVapor = 6.112 * exp(17.67 * temperature_C / (temperature_C + 243.5));
    double actualVapor = satVapor * humidity_pct / 100.0;
    double dryAir = (pressure_hPa - actualVapor) * 100.0 / (287.05 * (temperature_C + 273.15));
    double vapor = actualVapor * 100.0 / (461.5 * (temperature_C + 273.15));
    return dryAir + vapor;  // 湿空气密度
}

// 风载影响
Vec3d GetWindForce(double windSpeed, double windDir, double area, double Cd) {
    return 0.5 * airDensity * windSpeed * windSpeed * area * Cd * GetWindDirectionVector(windDir);
}
```

### 3.5 天气禁用模式

当 `weather.enabled = false` 时：
- 使用标准大气模型（ISA）
- 无风条件
- 忽略湿度影响
- 默认温度 15°C (288K)

---

## 4. 数据驱动架构 (Data-Driven Architecture)

### 4.1 配置分层

```
missions/
├── artemis2.conf          # 任务基础配置
├── artemis2_vehicle.yaml   # 箭体架构配置
├── artemis2_flight_plan.yaml  # 飞行计划配置
└── artemis2_weather.yaml   # 天气配置（可选）
```

### 4.2 箭体架构配置 (Vehicle YAML)

```yaml
vehicle:
  name: "SLS Block 1"
  stages:
    - id: 0
      name: "Core Stage + SRB"
      type: "first_stage"
      engines:
        - type: "RS-25"
          count: 4
          position: [0, 0, 0]
        - type: "SRB"
          count: 2
          position: [±3.5, 0, 0]
      tanks:
        - type: "LH2"
          fuelMass: 144000
          dryMass: 9500
        - type: "LOX"
          fuelMass: 840000
          dryMass: 4500
      separator_mass: 1200
    
    - id: 1
      name: "ICPS"
      type: "upper_stage"
      engines:
        - type: "RL10C-2"
          count: 1
      tanks:
        - type: "LH2"
          fuelMass: 25800
        - type: "LOX"
          fuelMass: 142000
    
    - id: 2
      name: "Orion SM"
      type: "service_module"
      persistent: true
      engines:
        - type: "AJ10-190"
          count: 1
      tanks:
        - type: "MMH"
          fuelMass: 4300
        - type: "NTO"
          fuelMass: 4300
```

### 4.3 飞行计划配置 (Flight Plan YAML)

```yaml
flight_plan:
  name: "Artemis II Nominal"
  phases:
    - name: "BOOST"
      duration_s: 126
      target_pitch_deg: 90
      events:
        - time: 0.0
          action: "IGNITE_RS25"
        - time: 3.0
          action: "IGNITE_SRB"
        - time: 126.0
          action: "STAGE_SEPARATION_0"
    
    - name: "CORE_BURN"
      duration_s: 480
      target_ap_km: 200
      events:
        - time: 150.0
          action: "MAXQ_THROTTLE_DOWN"
        - time: 200.0
          action: "MAXQ_THROTTLE_UP"
    
    - name: "ICPS_BURN"
      target_pe_km: 2000
      target_inclination_deg: 28.5
```

---

## 5. 开发里程碑 (Development Roadmap)

- [x] **Milestone 1: 亚轨道与大气物理**
- [x] **Milestone 2: 多级入轨**
- [x] **Milestone 3: 轨道机动与地图视图基础**
  - RCS 姿态控制与圆轨逻辑已接入。
  - 基础轨道预判（Ap/Pe 采样）已实现。
- [ ] **Milestone 4: 数据驱动架构重构**
  - [ ] 箭体架构 YAML 配置化
  - [ ] 飞行计划配置化
  - [ ] 天气系统模块
- [ ] **Milestone 5: 进阶天体力学**
  - 添加月球与跨 SOI 轨迹。
- [ ] **Milestone 6: VAB 载具组装大楼**
  - 计划转为 JSON/YAML 数据驱动。

---

*Last Updated: 2026-04-09*
