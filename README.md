# Runway CP-SAT V0.3 — 20 架航班航路与跑道联合优化

本项目是在 **`runway-cpsat-v2` 基础上直接升级**的版本，并非另起炉灶的新工程。原有的 10 架航班 API、演示程序和回归测试均予以保留。默认的 `runway_demo` 现在演示 **20 架航班的联合优化模型**。

> **安全说明：** 本项目中的所有时间、跑道间隔和限制点间隔均为合成测试数据。本项目是**算法原型**，**不是**可直接投入实际运行的空中交通管理系统。

## 新版模型

- **20 架必须安排的航班：** A01～A10 为进场（ARR），D01～D10 为离场（DEP）。
- **48 个候选方案：** ARR 共 29 个（P1/P2/P3），DEP 共 19 个（Q1/Q2）。
- **2 条混合运行跑道：** R1、R2。ARR 方案联合选择跑道、STAR 和航路限制点；DEP 方案选择跑道及该跑道专属的 VTT。
- **ARR 时间链：** 固定 ENTRY → 所选限制点（FIX）→ CLDT，包含两段按方案激活的区间飞行时间约束。每架航班共用一个 `CLDT` 变量。
- **DEP 时间链：** 固定 READY + 对应方案的最小滑行时间 ≤ MTOT。
- **硬时间窗口：** 支持航班级 NTOT 窗口及可选的方案级窗口。一个时间域可以由多个互不连续的闭区间组成。
- **跑道局部 Circuit：** 不使用某跑道的航班通过自环跳过；实际相邻航班按前后机类型施加有方向的 AA/AD/DA/DD 最小间隔。
- **最近同类型前机：** 每条跑道还建立独立的 ARR-only 和 DEP-only Circuit，避免 A–D–A 序列跳过 AA 等同类型前机保护。
- **共享限制点 Circuit：** X、Y 限制点采用**与候选方案对应的访问节点**，而不是复用航班跑道节点；只有选中的航路才占用相关限制点。
- **条件跨跑道冲突：** 为 A03/P3 与 D03/Q1 设置一项示例性跨跑道冲突，具有不对称的有方向间隔；仅当**两个指定方案都被选中**时才生效。
- **目标函数：** 最小化所有航班的 `max(0, operation_time - reference_time)` 之和。
- **独立校验器：** 不读取 CP-SAT 模型中的 Circuit 弧变量，而是根据最终结果重新检查航段时间、方案选择、时间窗口、跑道邻机、同类型前机、限制点邻机、跨跑道冲突及目标值。

V0.3 中，每个 ARR 候选航路方案至多访问一个共享限制点。**尚未实现**的功能包括：允许重复访问相同限制点的任意复杂航路图、ADA/DAD 三机规则、跑道关闭、真实等待区容量业务语义、实时输入快照加载，以及滚动时域结果发布。这些功能必须基于确认后的业务规则设计，**不会在本版中通过不准确的简化暗中替代**。

## 1. 使用本机安装的 OR-Tools 9.14 编译

环境要求：Linux、CMake ≥ 3.24、支持 C++20 的编译器、已安装的 OR-Tools 9.14 及其传递依赖。Ninja 为可选构建工具。

与你当前的环境一致，以下示例使用 GCC/G++ 13，OR-Tools 安装在 `/usr/local/ortools-9.14`。

```bash
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/usr/local/ortools-9.14 \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++-13
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

也可以使用 CMake Presets：

```bash
cmake --preset ninja-ortools
cmake --build --preset ninja-ortools
ctest --preset ninja-ortools --output-on-failure
```

如果 VS Code 的 CMake 配置命令已经指定编译器和 `CMAKE_PREFIX_PATH`，则不需要额外创建 `.vscode/settings.json`。

## 2. 运行程序

```bash
# 默认运行：20 架 ARR/DEP 航路与跑道联合优化
./build/runway_demo --time-limit 3 --workers 4

# 更充分的搜索，并输出 CP-SAT 原生搜索日志
./build/runway_demo --scenario baseline20 --time-limit 10 --workers 4 --log

# 放宽 A01/P1 的特殊落地时间窗口
./build/runway_demo --scenario route-switch --time-limit 3

# 将 FIX X/Y 的最小间隔由 90 秒提高到 180 秒
./build/runway_demo --scenario fix-conflict --time-limit 3

# 故意构造不可行的 D01 硬窗口；预期为 INFEASIBLE（退出码 3）
./build/runway_demo --scenario infeasible --time-limit 3

# 保留原有的 10 架航班全局序列演示
./build/runway_legacy_demo --time-limit 1 --workers 4
```

演示程序将输出所选方案、跑道、STAR、限制点、限制点通过时刻、CLDT/MTOT、延误、最优界、求解状态和耗时。

**`FEASIBLE` 表示已找到可行解，并不代表已证明全局最优。** 程序不会将 `UNKNOWN`、`INFEASIBLE` 或 `MODEL_INVALID` 的结果作为可发布调度方案。

### 关键验收场景：A01 航路切换

- A01/P1：使用 R1，CLDT 必须位于 `[600,650]`。
- D01/Q1：使用 R1，MTOT 必须位于 `[630,690]`，最早起飞时刻为 630。
- **A01 先运行：** D01 ≥ A01 + AD(120) ≥ 720，超过 NTOT 上限 690。
- **D01 先运行：** A01 ≥ D01 + DA(90) ≥ 720，超过落地上限 650。
- 因此 **A01/P1 不可行**。虽然 P1 的标称总飞行时间短于 P2，模型仍必须为 A01 选择使用 R2 的 P2 方案。
- 一组可独立验证的可行**方案选择组合**是：所有 ARR 选择 P2，所有 DEP 选择 Q1。具体运行时刻仍由求解器计算。

### 硬间隔矩阵（单位：秒；合成测试数据）

| 前机 \\ 后机 | ARR | DEP |
|---|---:|---:|
| ARR | 120（AA） | 120（AD） |
| DEP | 90（DA） | 100（DD） |

基准场景下，FIX X 和 FIX Y 的最小通过间隔均为 90 秒。

两条跑道默认可以独立使用；只有显式配置的跨跑道冲突规则才会施加额外约束。

## 3. 仅编译核心模块和单元测试（不需要 OR-Tools）

```bash
cmake -S . -B build-core -G Ninja -DRUNWAY_WITH_ORTOOLS=OFF
cmake --build build-core
ctest --test-dir build-core --output-on-failure
```

对外公开的领域模型、场景和校验器接口仍使用 **C++11**。只有 `src/cp_sat_scheduler.cpp` 和 `src/joint/cp_sat_joint.cpp` 需要 **C++20**。

## 4. 项目目录

```text
include/runway/model.h              原有 10 航班领域模型
include/runway/scheduler.h          原有 CP-SAT 求解接口
include/runway/joint.h              新增 20 航班领域模型与优化器接口
src/sample_problem.cpp              原有测试数据
src/validator.cpp                   原有校验器
src/cp_sat_scheduler.cpp            原有 CP-SAT 模型
src/joint/sample_joint_problem.cpp  新增 20 航班场景生成器
src/joint/joint_validator.cpp       联合方案独立校验器
src/joint/cp_sat_joint.cpp          方案、时间、跑道、限制点及跨资源冲突建模
app/joint_main.cpp                  新版 runway_demo 入口
app/main.cpp                        保留的 runway_legacy_demo 入口
src/joint/                         联合优化实现
CMakeLists.txt                      C++11 核心与 C++20 求解器适配
CMakePresets.json                   本机构建预设
tests/                              核心和求解器集成测试
```

## 5. 如何安全地扩展

- `JointProblem` 和 `FlightOption` 只包含普通业务数据；OR-Tools 类型不会泄漏到公开的 C++11 头文件中。
- 目前通过修改 `src/joint/sample_joint_problem.cpp` 调整输入场景。通过 JSON 配置和离线快照回放属于后续里程碑，**本版尚未实现**。
- 每新增一项业务硬约束，必须同时扩展 **CP-SAT 模型编译器、独立结果校验器及对应的反例测试**。
- 不要将所有 ARR/DEP 航班错误地视为全局互斥。每个局部 Circuit 只描述其对应的物理资源。
- 同类型最近前机检查与混合跑道邻机检查按设计分开建模。
- 扩展到 140 架航班时，应优先测量**模型构建时间和资源候选弧数量**，再考虑搜索启发式调优。当前原型**尚未证明具有 140 架航班生产环境所需的性能**。

## 6. 当前版本的验证情况

在生成本项目源码时使用的环境中，C++11 的 `runway_core`、`runway_core_test` 和 `runway_joint_core_test` 已成功编译；关闭 OR-Tools 的配置下，**2/2 项核心 CTest 检查通过**。

该环境未安装 OR-Tools C++ SDK，因此**新版 C++20 求解器适配模块及 20 航班完整求解没有在该环境中完成编译和运行验证**。请在已有 `/usr/local/ortools-9.14` 的机器上编译并运行全部 CTest 检查。不能把核心模块测试通过等同于完整求解器测试通过。

所使用的 OR-Tools C++ API 已参考上游 [v9.14 cp_model.h](https://github.com/google/or-tools/blob/v9.14/ortools/sat/cp_model.h) 进行核对，主要包括 `AddCircuitConstraint`、`AddArc`、`AddExactlyOne`、条件线性约束以及 `AddMaxEquality`。
