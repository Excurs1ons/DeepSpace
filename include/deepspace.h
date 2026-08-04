/*
 * deepspace.h — DeepSpace 统一世界 C ABI 契约
 *
 * 对应 Rust 实现：deepspace/src/ffi.rs
 * 接入文档：docs/ffi-integration.md（C / C++ / Unity C# / UE C++ 绑定示例）
 *
 * 约定：
 *  - DSWorld 是不透明句柄（*mut World），由 ds_world_create 创建、
 *    ds_world_destroy 释放，必须由同一线程驱动（同 PhysX 默认场景）。
 *  - 所有值类型均为 POD，可 memcpy，跨语言零拷贝。
 *  - 字符串固定缓冲，自动截断并保证 NUL 结尾。
 *  - 错误：返回码（0 = DS_OK，负值 = 错误）+ ds_last_error_message。
 *  - 单线程访问：世界由驱动引擎主线程持有。
 */

#ifndef DEEPSPACE_H
#define DEEPSPACE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 * 不透明句柄
 * ===================================================================== */

typedef struct DSWorld DSWorld;

/* =====================================================================
 * 错误码
 * ===================================================================== */

#define DS_OK               0
#define DS_ERR_NULL         (-1)   /* 空指针 */
#define DS_ERR_NOT_FOUND    (-2)   /* 实体/索引不存在 */
#define DS_ERR_INVALID_ARG  (-3)   /* 参数非法 */
#define DS_ERR_OOM          (-4)   /* 内存不足 */
#define DS_ERR_STATE        (-5)   /* 状态非法 */

/* =====================================================================
 * POD 值类型
 * ===================================================================== */

typedef struct DSVec3 {
    double x;
    double y;
    double z;
} DSVec3;

/* 统一实体类型标签（与 Rust EntityKind 一一对应） */
typedef enum DSEntityKind {
    DSK_Rocket     = 0,
    DSK_Spacecraft = 1,
    DSK_Missile    = 2,
    DSK_Icbm       = 3,
    DSK_Aircraft   = 4,
    DSK_Body       = 5,
} DSEntityKind;

/* 统一实体状态快照（外部引擎每帧读这里渲染） */
typedef struct DSEntityState {
    uint64_t     id;           /* 实体 id（稳定句柄） */
    DSEntityKind kind;         /* 实体类型 */
    DSVec3       position;     /* 世界坐标位置 (m) */
    DSVec3       velocity;     /* 世界坐标速度 (m/s) */
    DSVec3       acceleration; /* 加速度 (m/s²) */
    double       altitude_m;   /* 离地表海拔 (m) */
    bool         alive;        /* 是否存活 */
    char         name[64];     /* 显示名（NUL 结尾） */
    char         status[64];   /* 状态文本（NUL 结尾） */
} DSEntityState;

/* 世界事件类型（与 Rust EventKind 一一对应） */
#define DSEV_Info    0
#define DSEV_Detect  1
#define DSEV_Launch  2
#define DSEV_Hit     3
#define DSEV_Phase   4
#define DSEV_Outcome 5

/* 世界事件（闭环反馈日志） */
typedef struct DSEvent {
    double time;   /* 世界时间 (s) */
    uint32_t kind; /* DSEV_* */
    char text[256]; /* 事件文本（NUL 结尾） */
} DSEvent;

/* 弹道目标配置（新建弹道目标用） */
typedef struct DSBallisticConfig {
    char   name[64];       /* 显示名 */
    DSVec3 position;       /* 初始位置 (m) */
    DSVec3 velocity;       /* 初始速度 (m/s) */
    double mass;           /* 质量 (kg) */
    double ref_area_m2;    /* 弹道系数参考面积 (m²) */
    double cd;             /* 阻力系数 */
    double thrust_n;       /* 发射后推力 (N)，0 = 纯弹道 */
    double thrust_duration_s; /* 推力时长 (s) */
} DSBallisticConfig;

/* 6DOF 航天器配置 */
typedef struct DSSpacecraftConfig {
    char   name[64];
    DSVec3 position;
    DSVec3 velocity;
    double mass;
    double inertia_xx;
    double inertia_yy;
    double inertia_zz;
} DSSpacecraftConfig;

/* 拦截导弹配置（ProNav / APN 制导） */
typedef struct DSInterceptorConfig {
    char   name[64];
    double mass_kg;
    double kill_radius_m;
    double max_mach;
    double max_g_load;
    double nav_constant;
} DSInterceptorConfig;

/* =====================================================================
 * 生命周期
 * ===================================================================== */

/* 创建标准统一世界（地球固定在原点 + 默认大气 + APG-77 雷达）。
 * 等价于 World::default。返回句柄，用 ds_world_destroy 释放。 */
DSWorld *ds_world_create(void);

/* 释放世界。world 必须来自 ds_world_create 且未释放过。 */
void ds_world_destroy(DSWorld *world);

/* 推进一个世界步（N 体 + 航天器 + 弹道 + 拦截 + 探测 + 命中）。 */
int32_t ds_world_step(DSWorld *world);

/* 当前世界时间 (s)。world 为空返回 0 并记录错误。 */
double ds_world_time(DSWorld *world);

/* 最后一条错误消息（线程局部，UTF-8）。
 * buf 为空时返回所需长度；否则写入最多 buf_len-1 字节 + NUL，返回写入字节数。 */
size_t ds_last_error_message(char *buf, size_t buf_len);

/* =====================================================================
 * 实体操作
 * ===================================================================== */

/* 注册 6DOF 航天器，返回实体 id（-1 失败）。cfg 非空。 */
int64_t ds_world_add_spacecraft(DSWorld *world, const DSSpacecraftConfig *cfg);

/* 添加圆轨道卫星（绕世界原点，给定海拔与倾角），返回实体 id。 */
int64_t ds_world_add_satellite(DSWorld *world, const char *name,
                               double altitude_m, double inclination_rad);

/* 添加弹道目标（ICBM / 靶弹），返回实体 id。 */
int64_t ds_world_add_ballistic(DSWorld *world, const DSBallisticConfig *cfg);

/* 发射拦截导弹拦截指定目标，返回拦截弹实体 id（-1 失败）。 */
int64_t ds_world_fire_interceptor(DSWorld *world, uint64_t target_id,
                                  DSVec3 pos, DSVec3 vel);

/* 雷达探测到第一个存活弹道目标 id（无则 -1）。 */
int64_t ds_world_detected_target(DSWorld *world);

/* 实体总数（统一实体表，HUD 遍历用）。 */
size_t ds_world_entity_count(DSWorld *world);

/* 按索引取实体状态（实体表有序，索引 ∈ [0, count)）。
 * 状态拷贝到 out（非空）。越界返回 DS_ERR_NOT_FOUND。 */
int32_t ds_world_entity_at(DSWorld *world, size_t index, DSEntityState *out);

/* 事件流可用事件数（无输出缓冲时的查询）。 */
size_t ds_world_event_count(DSWorld *world);

/* 轮询最近事件：最多写入 max_events 条到 out，返回写入条数。
 * 顺序为新的在前。不消费事件流（多次调用得到同一批）。 */
size_t ds_world_poll_events(DSWorld *world, DSEvent *out, size_t max_events);

#ifdef __cplusplus
}
#endif

#endif /* DEEPSPACE_H */
