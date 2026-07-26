## Plan: 实时展示下一阶段进度

### 目标
在每个遥测输出行下方，追加一行显示当前阶段到下一阶段的转换条件和完成进度：
```
  T+   10.0s  [Launch     ]  alt=      338m  vel=     68m/s  mass= 2083866kg  thr=36827kN
  Next → Ascent: alt 338/1000m [34%]
```

---

### 修改 1: `deepspace/src/simulation.rs` — 添加数据结构和方法

**1a. 添加 `ConditionProgress` 结构体**（在 `TriggerCondition` 定义后，~line 142）：
```rust
#[derive(Debug, Clone)]
pub struct ConditionProgress {
    pub label: String,      // 短标签 "alt", "vel", "T+", "v/v₀", "Q"
    pub current: f64,       // 当前测量值
    pub target: f64,        // 目标阈值
    pub progress: f64,      // 0.0~1.0+ (≥1.0 = 已满足)
    pub is_met: bool,
    pub is_boolean: bool,   // true = 纯布尔条件（MaxQ passed / cutoff fired）
}
```

**1b. 添加 `NextPhaseInfo` 结构体**（同一位置）：
```rust
#[derive(Debug, Clone)]
pub struct NextPhaseInfo {
    pub next_phase: String,
    pub conditions: Vec<ConditionProgress>,
    pub require_all: bool,
}
```

**1c. 在 `MissionControl` 上添加 `compute_next_phase_info()` 方法**（~line 1820，在 `set_phase_name` 前）：
- 遍历 `self.script.phase_transitions` 找到 `from == self.phase_name` 的第一个
- 对每个条件，根据 TriggerType 计算当前值、目标值、完成比例（0.0~1.0+）
- 各 TriggerType 的映射规则：

| TriggerType | 标签 | current | target | progress |
|---|---|---|---|---|
| TimeElapsed | "T+" | mission_time | cond.value | cur/target (target=0 → 1.0) |
| AltitudeAbove | "alt↑" | altitude | cond.value | cur/target |
| AltitudeBelow | "alt↓" | altitude | cond.value | (target-cur)/target ? |
| VelocityAbove | "vel" | velocity | cond.value | cur/target |
| VelocityRatioAbove | "v/v₀" | velocity/v_orbital | cond.value | cur/target |
| TimeSincePhaseAbove | "Δt" | time-phase_entry | cond.value | cur/target |
| DynamicPressureAbove | "Q" | telemetry.Q | cond.value | cur/target |
| MaxqPassed | "MaxQ" | max_q_passed | — | 布尔: 0/1 |
| EngineCutoff | "cutoff" | cutoff_fired | — | 布尔: 0/1 |
| FlagIsTrue/False | param | flag value | — | 布尔: 0/1 |
| ApoapsisAbove | "ap" | ap_from_oe | cond.value | cur/target |
| PeriapsisAbove | "pe" | pe_from_oe | cond.value | cur/target |

- 传入 `(vessel, earth)` 以计算 altitude, velocity, orbital elements
- 返回 `None` 如果没有匹配的转换（最终阶段）

---

### 修改 2: `demo/src/app.rs` — 输出展示

**2a. 在 `run()` 的 `println!` 后追加进度行**（在 `next_print += print_interval;` 前，~line 911）：

```rust
// 显示下一阶段进度
if let Some(info) = self.mission_control.compute_next_phase_info(&self.vessel, &self.earth) {
    let mut parts: Vec<String> = Vec::new();
    for c in &info.conditions {
        if c.is_boolean {
            let status = if c.is_met { "✓" } else { "waiting" };
            parts.push(format!("{} [{}]", c.label, status));
        } else {
            let pct = (c.progress * 100.0).min(99.9);
            parts.push(format!("{} {:.0}/{:.0} [{:.0}%]", c.label, c.current, c.target, pct));
        }
    }
    let sep = if info.require_all { " | " } else { " OR " };
    println!("  Next → {}: {}", info.next_phase, parts.join(sep));
}
```

输出示例：
```
  T+    5.0s  [Launch     ]  alt=       84m  vel=     31m/s  mass= 2213333kg  thr=36683kN
  Next → Ascent: alt↑ 84/1000 [8%]

  T+   50.0s  [Ascent     ]  alt=    13050m  vel=    663m/s  mass= 1035181kg  thr=41003kN
  Next → MaxQ: MaxQ [waiting]

  T+   50.5s  [MaxQ       ]  alt=    13050m  vel=    663m/s  mass= 1035181kg  thr=41003kN
  Next → Orbit: v/v₀ 0.52/0.95 [55%]

  T+  120.0s  [MaxQ       ]  alt=    59782m  vel=    710m/s  mass=  842202kg  thr= 9194kN
  Next → Orbit: v/v₀ 0.74/0.95 [78%]
```

### 涉及文件
- `deepspace/src/simulation.rs` — ~30 行新增
- `demo/src/app.rs` — ~15 行新增

### 测试
```
cargo build -p deepspace-demo && cargo run --bin rocket-sim -- --headless --dt 0.5 --duration 120
cargo test -p deepspace --lib
```