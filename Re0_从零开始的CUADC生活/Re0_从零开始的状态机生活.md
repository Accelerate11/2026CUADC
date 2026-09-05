# 从 0 写出 CUADC 级 ROS 2 / MAVROS C++ 全自主任务状态机

> **适用对象**：会用电脑和终端，但 C++、ROS 2、MAVROS、无人机任务状态机经验接近 0 的新人。  
> **目标**：不是“看懂一份代码”，而是最终能够自己从空文件开始，逐层写出类似 `cuadc_full_mission_node_3_v11.cpp` 的工程代码，并知道每一层为什么存在、出了问题该查哪里。  
> **基准工程**：本文的工程结构、函数命名、状态组织、视觉时间对齐、桶跟踪、双阶段对准、刚体投放口补偿、舵机 ACK、安全门禁等，均以当前 `cuadc_full_mission_node_3_v11.cpp` 为基准。  
> **教学补充**：C++ 基础、ROS 2 语义、数学推导和独立练习是为了教学加入的背景知识；它们不是原始源码逐字注释。  
> **范围说明**：本文覆盖这份工程实际用到的主要 C++17 语法、STL、ROS 2、MAVROS 和任务算法模式，不等价于完整 ISO C++ 语言手册。

---

## 目录

1. 先建立全局认知：这类程序到底在做什么
2. 学习路线与安全测试顺序
3. C++ 程序最基本的骨架
4. 变量、类型、常量、表达式与作用域
5. 函数：参数、返回值、引用、`const`、静态函数
6. `struct`、`enum class`、`class`：状态机的三块砖
7. STL 容器：`vector`、`deque`、`optional`
8. STL 算法、Lambda、模板和 `auto`
9. 时间：`chrono`、ROS 时间与“稳定时钟”
10. 异步编程：`future` / `shared_future`
11. 第一阶段实践：纯 C++ 状态机
12. ROS 2 节点从零：Node、Timer、Publisher、Subscriber
13. ROS 2 QoS、回调和日志
14. ROS 2 参数系统
15. MAVROS 在架构中的位置
16. 第二阶段实践：订阅里程计、发布虚拟 setpoint
17. 状态机工程化：`tick()`、`enter()`、门禁、超时
18. 坐标系：local、field、body 与固定任务航向
19. 四元数转欧拉角
20. 刚体旋转：`Rz * Ry * Rx`
21. 第三阶段实践：场地坐标与姿态旋转
22. 轨迹层：为什么不能把 setpoint 一次跳 30 m
23. 梯形速度轨迹 `Segment`
24. 近目标限速 `slew_target_toward`
25. 第四阶段实践：在控制台跑一个虚拟航段
26. 飞前门禁与坐标系锁定
27. 视觉时间对齐：为什么“检测结果到达时的位置”是错的
28. 里程计历史、严格夹逼插值与视觉等待队列
29. 第五阶段实践：合成 30 Hz 里程计做时间插值
30. 目标跟踪：从单帧检测到稳定 `BucketTrack`
31. 中值、MAD、指数平滑与数据关联
32. 三桶选二与目标冻结
33. 第六阶段实践：写一个最小桶跟踪器
34. 投放对准：从飞机中心变成“真实投放口”
35. 粗对准、细对准与低空视觉 fallback
36. RELEASE 最终门禁
37. 投放口速度估计与机械脱离延迟
38. 第七阶段实践：刚体投放口补偿
39. MAVROS Service：解锁、起飞、降落
40. 舵机命令、ACK 与“物理状态不确定”
41. 第八阶段实践：异步请求状态机
42. SEARCH 航线和主动边界回收
43. 侦察、返航、落地与显式上锁
44. 故障状态：RETURN_HOME、PILOT_OVERRIDE、ABORT
45. 如何从零逐步拼成完整任务
46. 一份教学版完整骨架
47. 原工程函数总表：每个函数到底负责什么
48. 成员变量如何分类，不再被几百个变量吓住
49. 常见错误与调试方法
50. 新人练习路线与验收标准
51. 继续升级：O4 侦察、12 m/s 非任务区转场应该怎么改
52. 最终心法

---

# 1. 先建立全局认知：这类程序到底在做什么

新人最容易出现的第一个误解是：

> “这个 C++ 程序是不是在直接控制四个电机？”

不是。

这类程序处于**任务层 / 导航层**。它告诉 ArduPilot：

- 我现在希望飞机去哪里；
- 我希望保持什么航向；
- 什么时候开始搜索；
- 什么时候对准；
- 什么时候投瓶；
- 什么情况下返航；
- 什么情况下停止自动任务。

真正的：

- 角速度环；
- 姿态环；
- 位置环；
- 电机混控；
- EKF 状态估计；

主要由 ArduPilot 完成。

可以把系统看成：

```text
                 ┌────────────────────────────┐
                 │      C++ 任务状态机         │
                 │  SEARCH / ALIGN / RELEASE  │
                 └─────────────┬──────────────┘
                               │
             ┌─────────────────┼─────────────────┐
             ↓                 ↓                 ↓
       视觉目标处理        位置 setpoint       舵机命令
       D435i / YOLO        PoseStamped          DO_SET_SERVO
             │                 │                 │
             └──────────┬──────┴─────────────────┘
                        ↓
                     MAVROS
                        ↓
                    ArduPilot
                        ↓
          EKF / Position / Attitude / Motors
```

所以写这种程序，真正要掌握的不是某一个“神奇算法”，而是五件事：

1. **如何组织一个不会乱跳的状态机**
2. **如何可靠接收传感器状态**
3. **如何把坐标和时间对齐**
4. **如何生成合理的目标轨迹**
5. **如何处理异步命令、ACK、超时和故障**

---

# 2. 学习路线与安全测试顺序

不要一上来就把完整代码放真机。

推荐严格按以下层级：

```text
纯 C++ 控制台
    ↓
ROS 2 无飞控节点
    ↓
ROS 2 + 假数据
    ↓
ArduPilot SITL
    ↓
Gazebo
    ↓
MAVROS + SITL
    ↓
台架：无桨 / 无载荷
    ↓
低高度真机
    ↓
完整任务
```

原则：

> **每增加一个真实硬件，就只增加一个新的未知量。**

如果你同时第一次接触：

- C++；
- ROS 2；
- MAVROS；
- 飞控；
- 相机；
- 舵机；
- RTK；

那么出了问题几乎无法定位。

---

# 3. C++ 程序最基本的骨架

最小程序：

```cpp
#include <iostream>

int main()
{
    std::cout << "hello CUADC" << std::endl;
    return 0;
}
```

编译：

```bash
g++ -std=c++17 hello.cpp -o hello
./hello
```

逐行解释。

## 3.1 `#include`

```cpp
#include <iostream>
```

`#include` 是预处理指令。

你可以先简单理解成：

> “我要使用别人已经写好的工具，请把对应声明引进来。”

原工程里常见：

```cpp
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <future>
#include <optional>
#include <string>
#include <vector>
```

分别提供：

- `algorithm`：排序、查找、`clamp` 等；
- `chrono`：时间；
- `cmath`：三角函数、平方根；
- `deque`：双端队列；
- `future`：异步结果；
- `optional`：可能存在，也可能不存在的值；
- `string`：字符串；
- `vector`：动态数组。

ROS 头文件例如：

```cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
```

## 3.2 `main`

```cpp
int main()
```

操作系统启动程序时会进入 `main()`。

ROS 2 版本通常是：

```cpp
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}
```

后面会逐项讲。

## 3.3 分号

C++ 大多数语句以 `;` 结束：

```cpp
int a = 3;
a = a + 1;
```

而代码块：

```cpp
if (...) {
}
```

右花括号后通常不加分号。

但 `struct`、`class`、`enum class` 定义结束要加：

```cpp
struct Point3 {
    double x;
    double y;
    double z;
};
```

---

# 4. 变量、类型、常量、表达式与作用域

## 4.1 常用类型

```cpp
bool enabled = true;
int count = 3;
double height = 1.3;
std::string mode = "GUIDED";
```

原工程还大量使用：

```cpp
std::size_t index = 0U;
int64_t pwm = 1500;
```

### `std::size_t`

用于：

- 容器长度；
- 数组索引；
- ID。

例如：

```cpp
for (std::size_t i = 0U; i < values.size(); ++i) {
}
```

`0U` 的 `U` 表示无符号整数字面量。

### `int64_t`

固定 64 位整数。舵机通道、PWM 参数从 ROS 参数数组读取时经常用它。

---

## 4.2 `const`

```cpp
const double x = 3.0;
```

表示之后不能重新给 `x` 赋值。

原工程常见：

```cpp
const Point3 release_body = ...;
const double distance = ...;
```

意义不仅是“防误改”，更是在告诉读代码的人：

> 这个变量在当前逻辑中是输入事实，不是状态。

---

## 4.3 `constexpr`

```cpp
constexpr double kPi = 3.14159265358979323846;
```

`constexpr` 比 `const` 更强调：

> 这个值可以在编译期确定。

原工程：

```cpp
constexpr double ramp_fraction = 0.20;
```

用于轨迹公式。

---

## 4.4 作用域

```cpp
if (true) {
    double x = 1.0;
}
// 这里不能再访问 x
```

花括号 `{}` 形成作用域。

大型状态机最容易出现的问题之一，就是把所有临时量都变成成员变量。

规则：

- **只在一个函数里使用** → 局部变量；
- **多个 tick 之间必须记住** → 成员变量；
- **固定不变** → `constexpr`；
- **多个函数共享的简单数据结构** → `struct`。

---

# 5. 函数：参数、返回值、引用、`const`、静态函数

先看：

```cpp
double distance_xy(const Point3 & a, const Point3 & b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}
```

这里涉及很多核心语法。

## 5.1 返回类型

最前面的：

```cpp
double
```

表示函数返回一个 `double`。

如果不返回：

```cpp
void publish_setpoint()
{
}
```

---

## 5.2 参数

```cpp
const Point3 & a
```

拆开：

- `Point3`：参数类型；
- `&`：引用，不复制整个对象；
- `const`：函数承诺不修改这个对象；
- `a`：参数名。

对于结构体、字符串、vector，工程里常用：

```cpp
const T &
```

避免不必要复制。

---

## 5.3 值传递

```cpp
double normalize_angle(double value)
```

`double` 很小，直接复制即可。

---

## 5.4 `const` 成员函数

```cpp
bool odom_fresh() const
{
    ...
}
```

函数末尾的 `const` 表示：

> 这个成员函数不应该修改对象的成员状态。

典型用于：

- 查询；
- 计算；
- 判定。

例如：

```cpp
double current_geometry_yaw() const;
bool inside_release_area(...) const;
```

---

## 5.5 `static`

原工程：

```cpp
static double steady_age_s(const SteadyTimePoint & stamp)
```

类里的 `static` 函数不依赖具体对象的 `this`。

还有：

```cpp
static Point3 rotate_body_vector_to_local(...)
```

它只是数学工具。

经验：

> 如果一个函数只使用输入参数，不读取任何成员变量，就考虑 `static`。

---

# 6. `struct`、`enum class`、`class`：状态机的三块砖

## 6.1 `struct`

```cpp
struct Point3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};
```

把相关数据打包。

使用：

```cpp
Point3 p;
p.x = 1.0;

Point3 q{1.0, 2.0, 3.0};
```

原工程用 `struct` 表示：

- `Point3`
- `HeadingSample`
- `NavigationSample`
- `Segment`
- `BucketTrack`
- `PendingServoCommand`
- `PendingVisionDetection`
- `PendingVisionFrame`

判断标准：

> 一个概念需要多个字段共同描述，就建立一个 `struct`。

---

## 6.2 `enum class`

```cpp
enum class State
{
    WAIT_FCU,
    TAKEOFF,
    SEARCH,
    LAND,
    DONE
};
```

它表示“一组有限离散状态”。

比：

```cpp
int state = 3;
```

好得多，因为：

```cpp
State::SEARCH
```

一眼就知道意义。

状态机几乎一定应该用 `enum class`。

---

## 6.3 `class`

简化版本：

```cpp
class Mission
{
public:
    Mission()
    {
    }

private:
    State state_ = State::WAIT_FCU;
};
```

### `public`

外部可以访问。

### `private`

只有类内部成员函数可以访问。

原工程：

```cpp
class VisualDropMissionNode final : public rclcpp::Node
```

表示：

- 定义一个类 `VisualDropMissionNode`；
- 它继承 ROS 2 的 `rclcpp::Node`；
- `final` 表示不希望其他类再继承它。

---

## 6.4 构造函数

```cpp
VisualDropMissionNode()
: Node("visual_drop_mission_node")
{
}
```

名字和类名相同，没有返回类型。

冒号后是初始化列表：

```cpp
: Node("visual_drop_mission_node")
```

先构造父类 `Node`。

---

# 7. STL 容器：`vector`、`deque`、`optional`

这三个是原工程最重要的容器。

## 7.1 `std::vector`

动态数组：

```cpp
std::vector<Point3> route;
route.push_back(Point3{1, 2, 3});
route.push_back(Point3{4, 5, 6});

std::cout << route.size() << std::endl;
```

适合：

- 搜索航点；
- 已知桶；
- 已释放目标；
- 舵机参数；
- 目标 ID。

访问：

```cpp
route[0]
```

注意：越界访问是严重错误。

所以原工程经常先检查：

```cpp
if (index >= route.size()) {
    return;
}
```

---

## 7.2 `std::deque`

双端队列。

```cpp
std::deque<double> samples;
samples.push_back(1.0);
samples.pop_front();
```

适合“滑动历史”。

原工程用来保存：

- 航向历史；
- 里程计历史；
- 待时间对齐视觉帧；
- 直径滤波窗口。

为什么不用 vector？

因为历史队列常常：

```text
尾部追加
头部删除
```

`deque` 对这种操作更自然。

---

## 7.3 `std::optional`

表示：

> 这个值可能还没有。

例如起飞点：

```cpp
std::optional<Point3> home_;
```

没锁定之前：

```cpp
home_.has_value() == false
```

锁定后：

```cpp
home_ = position_;
```

访问：

```cpp
if (home_.has_value()) {
    std::cout << home_->x << std::endl;
}
```

或：

```cpp
Point3 p = home_.value_or(Point3{});
```

它比用：

```cpp
bool have_home;
Point3 home;
```

更不容易把“未初始化”状态搞乱。

原工程中大量 `optional` 用于：

- home；
- active bucket；
- 某个阶段开始时间；
- 稳定计时起点；
- 上一次样本。

---

# 8. STL 算法、Lambda、模板和 `auto`

原工程大量使用 `<algorithm>`。

## 8.1 `std::clamp`

```cpp
double x = std::clamp(value, 0.0, 1.0);
```

小于 0 变 0，大于 1 变 1。

参数加载时非常常见。

---

## 8.2 `std::max` / `std::min`

```cpp
speed = std::max(0.3, speed);
```

意味着：

> 无论参数怎么写，最低不允许低于 0.3。

这是“参数防御”。

---

## 8.3 `std::find_if`

```cpp
auto it = std::find_if(
    tracks.begin(),
    tracks.end(),
    [id](const BucketTrack & t) {
        return t.id == id;
    });
```

Lambda：

```cpp
[id](const BucketTrack & t) {
    return t.id == id;
}
```

可以理解成一个临时小函数。

### `[id]`

把外部变量 `id` 复制进 Lambda。

### `[this]`

允许访问当前类的成员。

### `[this, &local]`

- `this`：当前对象；
- `&local`：按引用捕获外部变量。

---

## 8.4 `std::any_of` / `std::all_of`

```cpp
bool used = std::any_of(
    released.begin(),
    released.end(),
    [&](const Point3 & p) {
        return distance_xy(p, target) < 0.25;
    });
```

- `any_of`：只要一个满足；
- `all_of`：必须全部满足。

---

## 8.5 `std::sort`

```cpp
std::sort(values.begin(), values.end());
```

自定义排序：

```cpp
std::sort(
    tracks.begin(),
    tracks.end(),
    [](const BucketTrack & a, const BucketTrack & b) {
        return a.diameter < b.diameter;
    });
```

这就是“三桶里优先选较小桶”的基础。

---

## 8.6 `std::lower_bound`

原工程的视觉时间对齐使用：

```cpp
std::lower_bound(...)
```

在**已经按时间排序**的导航历史里，找到第一个：

```text
stamp >= 视觉时间
```

的样本。

这比每次从头遍历更明确。

---

## 8.7 erase-remove idiom

```cpp
known_buckets_.erase(
    std::remove_if(
        known_buckets_.begin(),
        known_buckets_.end(),
        predicate),
    known_buckets_.end());
```

这是 C++17 里删除“满足某条件元素”的经典写法。

---

## 8.8 `auto`

```cpp
const auto nav = navigation_sample_at(...);
```

编译器自动推导类型。

适合类型特别长时：

```cpp
auto request = std::make_shared<SomeLongService::Request>();
```

但不要把所有东西都写 `auto`。

新人原则：

> 如果明确类型能提高可读性，就写明确类型；如果类型又长又明显，就用 `auto`。

---

## 8.9 模板

原工程：

```cpp
template<typename T>
T value_at(const std::vector<T> & values,
           std::size_t index,
           const T & fallback)
```

这表示：

> 同一个函数可以处理 `vector<double>`、`vector<int>` 等多种类型。

使用：

```cpp
double duration =
    value_at<double>(release_duration_s_, payload_index_, 0.7);
```

新人前期不需要自己大量写模板，但必须会读。

---

# 9. 时间：`chrono`、ROS 时间与“稳定时钟”

原工程同时存在两种时间概念。

## 9.1 `rclcpp::Time`

ROS 时间：

```cpp
rclcpp::Time stamp;
```

用于：

- 消息时间戳；
- 视觉帧与 odom 对齐。

---

## 9.2 `std::chrono::steady_clock`

原工程：

```cpp
using SteadyClock = std::chrono::steady_clock;
using SteadyTimePoint = SteadyClock::time_point;
```

`steady_clock` 的关键特点：

> 它只保证单调前进，不会因为系统时间校正突然倒退。

非常适合：

- “已经等了几秒”；
- 超时；
- 稳定持续时间；
- 状态进入时间。

例如：

```cpp
static double steady_age_s(const SteadyTimePoint & stamp)
{
    return std::chrono::duration<double>(
        SteadyClock::now() - stamp).count();
}
```

---

## 9.3 为什么不能所有东西都用一个时间

区分：

### “这帧图像什么时候采集？”

需要和 ROS 消息对齐：

```cpp
rclcpp::Time
```

### “这个状态已经持续多久？”

只关心经过时间：

```cpp
steady_clock
```

这是工程里很重要的设计。

---

# 10. 异步编程：`future` / `shared_future`

调用飞控 Service 时，不能这样：

```cpp
send_request();
while (!response) {
    // 卡死
}
```

因为这样会堵住整个状态机。

正确方式是：

```text
发请求
↓
先继续跑 tick
↓
以后每次 tick 看结果是否回来
```

原工程使用：

```cpp
rclcpp::Client<...>::SharedFuture
```

核心模式：

```cpp
future_ = client_->async_send_request(request).future.share();
```

然后：

```cpp
if (future_.valid()) {
    bool ready =
        future_.wait_for(0s) == std::future_status::ready;

    if (ready) {
        auto response = future_.get();
        future_ = {};
    }
}
```

`wait_for(0s)` 表示：

> 现在立刻看一下，绝不阻塞。

这就是非阻塞状态机。

---

# 11. 第一阶段实践：纯 C++ 状态机

先不碰 ROS。

保存为 `fsm_demo.cpp`：

```cpp
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

using Clock = std::chrono::steady_clock;

enum class State
{
    WAIT,
    TAKEOFF,
    SEARCH,
    LAND,
    DONE
};

std::string state_name(State s)
{
    switch (s) {
        case State::WAIT: return "WAIT";
        case State::TAKEOFF: return "TAKEOFF";
        case State::SEARCH: return "SEARCH";
        case State::LAND: return "LAND";
        case State::DONE: return "DONE";
    }
    return "UNKNOWN";
}

int main()
{
    State state = State::WAIT;
    auto state_enter = Clock::now();

    auto enter = [&](State next) {
        std::cout
            << state_name(state)
            << " -> "
            << state_name(next)
            << std::endl;

        state = next;
        state_enter = Clock::now();
    };

    while (state != State::DONE) {
        double age =
            std::chrono::duration<double>(
                Clock::now() - state_enter).count();

        switch (state) {
            case State::WAIT:
                if (age > 1.0) {
                    enter(State::TAKEOFF);
                }
                break;

            case State::TAKEOFF:
                if (age > 2.0) {
                    enter(State::SEARCH);
                }
                break;

            case State::SEARCH:
                if (age > 3.0) {
                    enter(State::LAND);
                }
                break;

            case State::LAND:
                if (age > 2.0) {
                    enter(State::DONE);
                }
                break;

            case State::DONE:
                break;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(50));
    }
}
```

编译：

```bash
g++ -std=c++17 fsm_demo.cpp -o fsm_demo
./fsm_demo
```

### 你必须理解

这里已经出现了完整状态机的核心：

```text
State
state
state_enter
enter()
while/tick
switch
timeout
```

复杂工程只是不断往这套骨架里增加：

- 传感器；
- 门禁；
- 轨迹；
- 视觉；
- 舵机；
- 故障状态。

---

# 12. ROS 2 节点从零：Node、Timer、Publisher、Subscriber

最小 ROS 2 节点：

```cpp
#include <chrono>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class HelloNode : public rclcpp::Node
{
public:
    HelloNode()
    : Node("hello_node")
    {
        timer_ = create_wall_timer(
            500ms,
            [this]() {
                RCLCPP_INFO(
                    get_logger(),
                    "hello");
            });
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<HelloNode>());
    rclcpp::shutdown();
    return 0;
}
```

这里：

```cpp
using namespace std::chrono_literals;
```

让你可以写：

```cpp
50ms
2s
```

而不是：

```cpp
std::chrono::milliseconds(50)
```

---

# 13. ROS 2 QoS、回调和日志

## 13.1 Subscriber

原工程形式：

```cpp
odom_sub_ =
    create_subscription<nav_msgs::msg::Odometry>(
        "/mavros/local_position/odom",
        rclcpp::SensorDataQoS(),
        std::bind(
            &VisualDropMissionNode::odom_callback,
            this,
            std::placeholders::_1));
```

拆开。

### 模板类型

```cpp
nav_msgs::msg::Odometry
```

表示订阅消息类型。

### Topic

```cpp
"/mavros/local_position/odom"
```

### QoS

```cpp
rclcpp::SensorDataQoS()
```

传感器数据通常更关注“最新”，而不是死保每一帧。

### Callback

```cpp
std::bind(
    &Class::callback,
    this,
    std::placeholders::_1)
```

表示：

> 收到一条消息，调用当前对象的 `callback(msg)`。

---

## 13.2 用 Lambda 也可以

教学代码常写：

```cpp
odom_sub_ =
    create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            position_x_ = msg->pose.pose.position.x;
        });
```

两种都应该会读。

---

## 13.3 Publisher

```cpp
setpoint_pub_ =
    create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/setpoint_position/local",
        10);
```

发送：

```cpp
geometry_msgs::msg::PoseStamped msg;
msg.header.stamp = now();
msg.pose.position.x = 1.0;

setpoint_pub_->publish(msg);
```

---

## 13.4 日志

```cpp
RCLCPP_INFO(get_logger(), "x=%.2f", x);
RCLCPP_WARN(get_logger(), "warning");
RCLCPP_ERROR(get_logger(), "error");
```

节流日志：

```cpp
RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    1000,
    "x=%.2f", x);
```

表示最多每 1000 ms 输出一次。

状态机高频 tick 中一定要节流，否则日志本身就能把系统拖慢。

---

# 14. ROS 2 参数系统

原工程先：

```cpp
declare_parameter<double>("takeoff_alt_m", 4.0);
```

再：

```cpp
takeoff_alt_m_ =
    get_parameter("takeoff_alt_m").as_double();
```

## 14.1 为什么参数不直接写死

因为调试时你希望：

```text
不重新编译
只改 YAML
```

就能改变：

- 高度；
- 速度；
- 容差；
- timeout；
- 舵机 PWM。

---

## 14.2 参数加载后要做保护

原工程：

```cpp
fine_align_alt_m_ =
    std::clamp(
        get_parameter("fine_align_alt_m").as_double(),
        0.8,
        coarse_alt_m_);
```

这不是多余。

因为 YAML 可能有人误写：

```yaml
fine_align_alt_m: -5
```

参数防御相当于第二层保险。

---

## 14.3 `config_valid_`

大型任务不能只“读参数”，还要检查组合是否合理。

例如：

```cpp
config_valid_ =
    flight_enable_ &&
    payload_count_ == 2 &&
    servo_channels_[0] == 9 &&
    servo_channels_[1] == 10;
```

这是：

> **配置契约**。

---

# 15. MAVROS 在架构中的位置

可以简单理解：

```text
你的 ROS 2 C++ 节点
        ↓
      MAVROS
        ↓ MAVLink
    ArduPilot
```

在当前工程中主要用到：

### 状态

```text
/mavros/state
```

### 本地里程计

```text
/mavros/local_position/odom
```

### 罗盘航向

```text
/mavros/global_position/compass_hdg
```

### 位置目标

```text
/mavros/setpoint_position/local
```

### Service

```text
/mavros/cmd/arming
/mavros/cmd/takeoff
/mavros/cmd/land
/mavros/cmd/command
```

任务代码并不直接处理 MAVLink 字节包。

MAVROS 已经把它封装成：

- ROS Topic；
- ROS Service。

---

# 16. 第二阶段实践：订阅里程计、发布虚拟 setpoint

**先不要把练习 topic 接到真机控制 topic。**

练习节点：

```cpp
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class PositionPractice : public rclcpp::Node
{
public:
    PositionPractice()
    : Node("position_practice")
    {
        odom_sub_ =
            create_subscription<nav_msgs::msg::Odometry>(
                "/mavros/local_position/odom",
                rclcpp::SensorDataQoS(),
                [this](
                    const nav_msgs::msg::Odometry::SharedPtr msg)
                {
                    x_ = msg->pose.pose.position.x;
                    y_ = msg->pose.pose.position.y;
                    z_ = msg->pose.pose.position.z;

                    RCLCPP_INFO_THROTTLE(
                        get_logger(),
                        *get_clock(),
                        1000,
                        "position=(%.2f, %.2f, %.2f)",
                        x_, y_, z_);
                });

        // 练习时使用自己的 topic，避免误控飞机
        target_pub_ =
            create_publisher<geometry_msgs::msg::PoseStamped>(
                "/tutorial/target",
                10);

        timer_ =
            create_wall_timer(
                std::chrono::milliseconds(100),
                [this]() {
                    geometry_msgs::msg::PoseStamped msg;
                    msg.header.stamp = now();
                    msg.header.frame_id = "map";
                    msg.pose.position.x = x_ + 1.0;
                    msg.pose.position.y = y_;
                    msg.pose.position.z = z_;
                    msg.pose.orientation.w = 1.0;
                    target_pub_->publish(msg);
                });
    }

private:
    double x_ = 0.0;
    double y_ = 0.0;
    double z_ = 0.0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        target_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

你要学会区分：

```text
position_  = 我测到飞机在哪里
target_    = 我希望飞机去哪里
```

这是整个工程最重要的变量语义之一。

---

# 17. 状态机工程化：`tick()`、`enter()`、门禁、超时

原工程主循环：

```cpp
timer_ =
    create_wall_timer(
        50ms,
        std::bind(
            &VisualDropMissionNode::tick,
            this));
```

即大约：

```text
20 Hz
```

## 17.1 `tick()` 应该做什么

推荐顺序：

```text
1. 处理外部异步结果
2. 更新健康状态
3. 发布当前 setpoint
4. 检查全局 timeout
5. 执行当前 State 的 update
```

而不是每个状态自己随便做一堆全局检查。

---

## 17.2 `enter()`

不要到处写：

```cpp
state_ = State::SEARCH;
```

统一写：

```cpp
enter(State::SEARCH);
```

好处：

- 统一记录日志；
- 统一更新时间；
- 统一清理阶段变量；
- 后面容易加 tracing。

简化：

```cpp
void enter(State next)
{
    if (state_ == next) {
        return;
    }

    RCLCPP_INFO(
        get_logger(),
        "State %s -> %s",
        state_name(state_).c_str(),
        state_name(next).c_str());

    state_ = next;
    state_enter_time_ =
        SteadyClock::now();
}
```

---

## 17.3 门禁

例如飞行任务的共同前提：

```cpp
bool flight_gate_ok()
{
    if (!connected_) {
        return false;
    }

    if (!armed_) {
        return false;
    }

    if (!guided_) {
        return false;
    }

    if (!odom_fresh()) {
        return false;
    }

    return true;
}
```

门禁要独立成函数。

不要在每个状态复制：

```cpp
if (!connected || !armed || ...)
```

---

## 17.4 稳定条件不是“某一帧满足”

错误：

```cpp
if (error < 0.2) {
    release();
}
```

正确：

```text
误差进入门限
↓
开始计时
↓
如果中途出门限，计时清零
↓
连续稳定 0.5 s
↓
才通过
```

这就是原工程大量：

```cpp
std::optional<SteadyTimePoint> *_stable_since_;
```

的原因。

---

# 18. 坐标系：local、field、body 与固定任务航向

你需要同时理解三个坐标系。

## 18.1 local

MAVROS 本地位置坐标。

你可以把它理解成任务程序运行时的“世界坐标”。

```cpp
position_.x
position_.y
position_.z
```

---

## 18.2 field

比赛场地坐标。

原工程定义：

```text
field +X = 起飞时机头方向
field +Y = 飞机左侧
field +Z = 向上
```

这样投放区可以写成：

```text
x = 30 ~ 35 m
y = -4 ~ +4 m
```

而不关心今天场地真实朝东、朝西还是朝北。

---

## 18.3 body

机体系。

视觉上游发布：

```text
/perception/drop_buckets_body
```

表示桶相对机体的位置。

投放口外参也是 body：

```text
A1: [ 0.026, -0.065, -0.32 ]
A2: [-0.026,  0.065, -0.32 ]
```

---

# 19. 四元数转欧拉角

Odometry 姿态是四元数：

```cpp
q.x
q.y
q.z
q.w
```

原工程先归一化：

```cpp
double norm = std::sqrt(
    q.x*q.x +
    q.y*q.y +
    q.z*q.z +
    q.w*q.w);
```

再算：

```cpp
roll = atan2(...);
pitch = asin(...);
yaw = atan2(...);
```

你不一定要背公式。

你必须理解两点：

1. 四元数是飞控/ROS 常用姿态表达；
2. 为了工程几何计算，当前代码转成了 roll/pitch/yaw。

---

# 20. 刚体旋转：`Rz * Ry * Rx`

原工程：

```cpp
rotate_body_vector_to_local(
    body,
    roll,
    pitch,
    yaw)
```

采用：

\[
R_{WB}
=
R_z(yaw)
R_y(pitch)
R_x(roll)
\]

顺序不能随便换。

因为矩阵乘法通常：

\[
AB \ne BA
\]

函数展开后，本质是：

```text
先绕 body X 做 roll
再绕新的 Y 做 pitch
再绕世界 Z 做 yaw
```

结果：

```text
机体系向量
↓
世界/local 方向的向量
```

---

# 21. 第三阶段实践：场地坐标与姿态旋转

纯 C++：

```cpp
#include <cmath>
#include <iostream>

constexpr double PI =
    3.14159265358979323846;

struct Point3
{
    double x = 0;
    double y = 0;
    double z = 0;
};

Point3 field_to_local(
    const Point3 & home,
    double mission_yaw,
    double field_x,
    double field_y,
    double z)
{
    double c = std::cos(mission_yaw);
    double s = std::sin(mission_yaw);

    return {
        home.x + c*field_x - s*field_y,
        home.y + s*field_x + c*field_y,
        home.z + z
    };
}

int main()
{
    Point3 home{10, 20, 0};

    // 机头比 local +X 左转 30°
    double yaw = 30.0 * PI / 180.0;

    Point3 p =
        field_to_local(
            home,
            yaw,
            30.0,
            0.0,
            2.2);

    std::cout
        << p.x << " "
        << p.y << " "
        << p.z << std::endl;
}
```

任务：

1. 把 `field_y` 改成 `+3`；
2. 再改成 `-3`；
3. 自己画出“飞机左侧”在 local 里的方向。

如果你画不出来，就还没有真正理解 `field_to_local()`。

---

# 22. 轨迹层：为什么不能把 setpoint 一次跳 30 m

最简单写法：

```cpp
target_ = Point3{30, 0, 4};
```

理论上 ArduPilot 会去。

但问题是：

- 目标突变；
- 飞机可能突然大倾角加速；
- 接近目标时制动依赖飞控自身；
- 不易控制全流程耗时；
- 高速场景更难预测超调。

所以原工程自己在任务层生成：

```text
平滑移动的 setpoint
```

而不是只给终点。

---

# 23. 梯形速度轨迹 `Segment`

结构：

```cpp
struct Segment
{
    Point3 start;
    Point3 end;
    SteadyTimePoint start_time;
    double duration_s = 1.0;
};
```

开始：

```cpp
start_segment(start, end, max_speed);
```

原工程采用：

```text
20% 加速
60% 近似匀速
20% 减速
```

平均速度因子：

```cpp
1.0 - 0.20 = 0.8
```

所以：

```cpp
duration =
    distance /
    (max_speed * 0.8);
```

同时：

```cpp
duration >= min_segment_s_
```

---

# 24. 近目标限速 `slew_target_toward`

对准阶段目标还会随视觉、姿态变化。

不适合重建一个固定 `Segment`。

因此使用：

```cpp
slew_target_toward(desired, max_speed);
```

核心：

```cpp
max_step =
    max_speed * dt;
```

然后沿目标方向每 tick 最多走这么远。

这个函数控制的是：

> **setpoint 的移动速度**

不是直接控制飞机速度。

---

# 25. 第四阶段实践：在控制台跑一个虚拟航段

任务：

- 起点 `(0,0,0)`；
- 终点 `(10,0,0)`；
- 最大 setpoint 速度 `2 m/s`；
- 20 Hz 打印目标位置。

你可以直接从原工程思路实现：

```cpp
double ratio =
    std::clamp(
        elapsed / duration,
        0.0,
        1.0);
```

然后做 20%-60%-20% 的 `progress`。

验收：

```text
开始速度小
中间速度大
末端速度再次变小
最后严格落在 10 m
```

如果你的 setpoint 最后停在 9.8 或 10.2，先别碰无人机。

---

# 26. 飞前门禁与坐标系锁定

原工程不是“FCU connected 就起飞”。

`navigation_ready_to_lock()` 大意要求：

```text
配置有效
FCU connected
未 armed
odom 新鲜
compass 新鲜
NAV30 ready
heading 稳定
vision ready
位置稳定
飞机静止
```

然后：

```cpp
lock_frame();
```

锁定：

```cpp
home_ = position_;
mission_yaw_ = mean_heading();
locked_odom_yaw_ = current_odom_yaw_;
```

### 为什么锁两种 yaw

`mission_yaw_`：

> 稳定罗盘定义的任务绝对方向。

`locked_odom_yaw_`：

> 记录锁定瞬间 odom yaw 的零偏基准。

后续几何 yaw：

\[
yaw_{geometry}
=
yaw_{mission}
+
(yaw_{odom} - yaw_{odom,lock})
\]

这样既：

- 不依赖 odom 的绝对零点；
- 又保留飞行中的实际小偏航变化。

---

# 27. 视觉时间对齐：为什么“检测结果到达时的位置”是错的

假设：

```text
10.000 s 相机采集
10.100 s YOLO完成
10.120 s ROS收到
```

飞机以：

```text
2 m/s
```

移动。

120 ms 里移动：

\[
2 \times 0.12 = 0.24m
\]

如果你用 10.120 s 的飞机位置去解释 10.000 s 的图像：

> 系统会凭空产生约 24 cm 动态误差。

因此正确做法是：

```text
视觉采集时间
     ↓
查历史 odom
     ↓
恢复采集瞬间的飞机位姿
     ↓
再做 body -> local
```

---

# 28. 里程计历史、严格夹逼插值与视觉等待队列

原工程保存：

```cpp
std::deque<NavigationSample>
    navigation_history_;
```

每个样本：

```cpp
struct NavigationSample
{
    rclcpp::Time stamp;
    Point3 position;
    double roll;
    double pitch;
    double odom_yaw;
};
```

## 28.1 严格夹住

视觉时间 `t` 必须满足：

```text
odom_before.time < t < odom_after.time
```

不允许：

- 最近邻替代；
- 向未来外推；
- 向过去外推。

---

## 28.2 插值

位置：

\[
p(t)
=
p_0
+
r(p_1-p_0)
\]

其中：

\[
r=
\frac{t-t_0}{t_1-t_0}
\]

角度不能普通直接减，因为有 `-π / +π` 环绕，所以原工程使用：

```cpp
normalize_angle(
    before +
    ratio * normalize_angle(after - before))
```

---

## 28.3 为什么视觉帧先进入 pending queue

视觉帧到达的瞬间，可能“后一帧 odom”还没到。

所以不是立刻丢弃。

流程：

```text
视觉到达
↓
放 pending_vision_frames_
↓
等最多约 0.15 s
↓
后续 odom 到来
↓
再次尝试严格夹逼
```

这叫：

> 为因线程调度 / ROS 到达顺序造成的轻微乱序留缓冲。

---

# 29. 第五阶段实践：合成 30 Hz 里程计做时间插值

先写纯 C++ 版本。

构造：

```text
t=0.000 x=0
t=0.033 x=0.033
t=0.066 x=0.066
```

查询：

```text
t=0.050
```

期望：

```text
x≈0.050
```

然后故意制造：

```text
t=0.000
t=0.100
```

如果最大 gap 设 0.050 s：

```text
必须拒绝
```

这个练习通过后，你才真正理解“30 Hz 时间硬同步”。

---

# 30. 目标跟踪：从单帧检测到稳定 `BucketTrack`

单帧检测不可靠。

你可能看到：

```text
frame 1: 桶直径 0.19
frame 2: 桶直径 0.22
frame 3: 漏检
frame 4: 桶直径 0.20
```

所以建立：

```cpp
struct BucketTrack
{
    std::size_t id;
    Point3 local;
    Point3 body;
    double diameter;
    double confidence;
    double position_deviation;
    double diameter_deviation;
    std::size_t confirmations;
    ...
};
```

这表示：

> “我认为这些连续检测属于同一个现实桶。”

---

# 31. 中值、MAD、指数平滑与数据关联

## 31.1 指数平滑

位置：

\[
x_{new}
=
(1-\alpha)x_{old}
+
\alpha x_{measured}
\]

原工程位置：

```text
alpha = 0.25
```

小 alpha：

- 更稳；
- 反应慢。

大 alpha：

- 更灵敏；
- 更抖。

---

## 31.2 中值

直径保存一个窗口：

```cpp
std::deque<double> diameter_samples;
```

排序后取中间值。

中值比平均值更不怕异常值。

例如：

```text
0.20
0.21
0.19
0.70   ← 错检
0.20
```

平均值会被拉偏，中值影响小。

---

## 31.3 MAD

MAD：

\[
median(|x_i - median(x)|)
\]

它衡量：

> 这组数据自己有多稳定。

不是只看一个直径值。

---

## 31.4 数据关联

对于每个已有 track 和当前 detection，算：

```text
位置差
直径差
```

只有：

```text
position_delta <= gate
diameter_delta <= gate
```

才允许匹配。

代价：

```cpp
position_delta / position_gate
+
diameter_delta / diameter_gate
```

然后按 cost 从小到大贪心关联。

---

# 32. 三桶选二与目标冻结

竞赛里三个桶直径不同。

当前逻辑：

```text
搜索途中：
    尽量等 3 个稳定独立桶

3 个齐：
    按直径排序
    选较小 2 个

完整搜索航线结束：
    如果始终只有 2 个可靠桶
    接受这 2 个
```

之后保存：

```cpp
selected_target_ids_
selected_target_positions_
selected_target_diameters_
```

并：

```cpp
target_selection_locked_ = true;
```

### 为什么冻结

否则可能：

```text
第一瓶投完
↓
视觉重新排序 / ID 重建
↓
第二瓶投到了错误桶
```

冻结目标把：

> “感知阶段的决策”

和：

> “执行阶段的动作”

分开。

---

# 33. 第六阶段实践：写一个最小桶跟踪器

输入模拟：

```cpp
struct Detection
{
    double x;
    double y;
    double diameter;
};
```

连续生成：

```text
桶 A 真值：x=1.0, y=2.0, d=0.15
桶 B 真值：x=3.0, y=2.0, d=0.20
桶 C 真值：x=5.0, y=2.0, d=0.25
```

每帧加入：

```text
±0.05 m 位置噪声
±0.01 m 直径噪声
```

要求：

1. 用位置 gate 建 track；
2. confirmations >= 3 才可靠；
3. 最后排序选较小两个。

这是把真实视觉问题先变成“纯数据问题”。

---

# 34. 投放对准：从飞机中心变成“真实投放口”

旧思路：

```text
飞机中心
   ↓
桶中心
```

新思路：

```text
真实出瓶口
   ↓
桶中心
```

两个投放口 body 外参：

```text
A1 [ +0.026, -0.065, -0.32 ]
A2 [ -0.026, +0.065, -0.32 ]
```

关键公式：

\[
p_{release}
=
p_{vehicle}
+
R_{WB}r_{release}^{B}
\]

希望：

\[
p_{release,xy}
=
p_{bucket,xy}
\]

所以反解飞机中心：

\[
p_{vehicle,xy}^{desired}
=
p_{bucket,xy}
-
(R_{WB}r_{release}^{B})_{xy}
\]

这就是：

```cpp
desired_vehicle_pose_for_release_target()
```

---

# 35. 粗对准、细对准与低空视觉 fallback

## 35.1 粗对准

高度：

```text
1.7 m
```

目标：

- 快速靠近；
- 允许较宽容误差；
- 已经使用真实投放口几何。

稳定后保存：

```cpp
coarse_alignment_target_local_
```

---

## 35.2 细对准

高度：

```text
1.3 m
```

速度：

```text
0.25 m/s
```

正常要求：

```text
飞机中心相对补偿目标 ≤ 0.25 m
真实投放口误差       ≤ 0.12 m
高度误差             ≤ 0.08 m
连续稳定             ≥ 0.45 s
```

---

## 35.3 低空视觉丢失

不是立即返航。

先等待一个 grace：

```text
0.8 s
```

如果仍无视觉，但最后可信目标已经满足 RELEASE 较宽门限：

```text
允许用冻结目标继续
```

这是：

> 感知可靠性和任务成功率之间的工程折中。

---

# 36. RELEASE 最终门禁

进入 RELEASE 后：

```text
桶坐标冻结
```

但飞机目标不冻结。

每 tick：

```cpp
direct_release_pose_ =
    desired_vehicle_pose_for_release_target(
        frozen_bucket,
        fine_alt);
```

因为 roll/pitch/yaw 可能一直变化。

最终要求同时满足：

```text
identity_ok
area_ok
geometry_ok
height_ok
motion_ok
attitude_ok
```

具体包括：

### 身份

仍然是原来的任务目标。

### 区域

- 桶在合法投放区域；
- 当前投放口也在；
- 预测脱离点也在。

### 几何

```text
center_xy         ≤ 0.35 m
release_point_xy  ≤ 0.20 m
predicted_xy      ≤ 0.20 m
```

### 高度

```text
≤ 0.15 m
```

### 速度

```text
水平 ≤ 0.15 m/s
垂直 ≤ 0.10 m/s
```

### 姿态

```text
|roll|  ≤ 8°
|pitch| ≤ 8°
角速度  ≤ 10°/s
```

全部连续稳定：

```text
0.5 s
```

才发舵机。

---

# 37. 投放口速度估计与机械脱离延迟

代码还估算：

```cpp
release_point_velocity_local_m_s_
```

思路：

```text
上一次投放口位置
当前投放口位置
时间差 dt
↓
速度
```

再做指数平滑。

预测：

\[
p_{predicted}
=
p_{release}
+
v_{release}
\Delta t_{detach}
\]

其中：

```text
Δt_detach
```

是：

> 命令发出到瓶子真正物理脱离的延迟。

当前默认：

```text
0.0 s
```

所以结构已经有，预测暂未真正产生提前量。

---

# 38. 第七阶段实践：刚体投放口补偿

写纯 C++：

```cpp
Point3 release_body{
    0.026,
    -0.065,
    -0.32
};
```

输入姿态：

```text
roll  = 0°
pitch = 0°
yaw   = 0°
```

算投放口。

然后：

```text
roll = 8°
```

重新算。

观察：

> 虽然 body 外参里的 X/Y 没变，但 `-0.32 m` 的 Z 力臂会投影成水平偏移。

再写：

```cpp
desired_vehicle =
    target_bucket - rotated_release_offset;
```

这就是完整对准数学核心。

---

# 39. MAVROS Service：解锁、起飞、降落

ROS Service 与 Topic 不同。

Topic：

```text
持续发布 / 持续订阅
```

Service：

```text
请求
↓
响应
```

原工程：

```cpp
arm_client_ =
    create_client<mavros_msgs::srv::CommandBool>(
        "/mavros/cmd/arming");
```

请求：

```cpp
auto request =
    std::make_shared<
        mavros_msgs::srv::CommandBool::Request>();

request->value = true;
```

发送：

```cpp
arm_future_ =
    arm_client_
        ->async_send_request(request)
        .future.share();
```

---

## 39.1 为什么请求要限频

原工程：

```cpp
bool request_allowed() const
{
    return steady_age_s(last_request_time_) >= 1.0;
}
```

否则 20 Hz tick 可能每秒向飞控发 20 次：

```text
ARM
ARM
ARM
ARM
...
```

这不是可靠，而是制造通信风暴。

---

## 39.2 CommandTOL 起飞期间为何不发位置 setpoint

当前工程明确：

```text
TAKEOFF 阶段 publish_setpoint_ = false
```

使用：

```text
/mavros/cmd/takeoff
```

起飞。

达到约：

```text
90% takeoff_alt
```

后再开始位置 setpoint。

核心思想：

> 不让两个不同的“上升控制意图”同时抢飞机。

---

# 40. 舵机命令、ACK 与“物理状态不确定”

舵机使用：

```text
MAV_CMD_DO_SET_SERVO
command = 183
```

参数：

```text
param1 = channel
param2 = PWM
```

例如：

```text
SERVO9
1200 -> stowed
1500 -> release
```

## 40.1 为什么必须保存 PendingServoCommand

请求发出去以后不能立刻认为动作成功。

保存：

```cpp
struct PendingServoCommand
{
    ServoPurpose purpose;
    std::size_t payload;
    SteadyTimePoint sent;
    SharedFuture future;
};
```

随后 `check_servo_results()` 处理。

---

## 40.2 ACK timeout 不等于“舵机没动”

这是很重要的工程常识。

情况可能是：

```text
命令真正到达飞控
飞控执行
舵机动作
但是 ACK 在通信中丢了
```

所以：

```text
ACK 超时
```

不能安全地解释成：

```text
“没有释放”
```

原工程采取更保守语义：

> **物理释放状态不确定。**

于是：

- 不把 payload 计为完成；
- 尝试回收舵机；
- 进入安全结束流程。

---

# 41. 第八阶段实践：异步请求状态机

不连接飞控，先写一个 mock：

```cpp
enum class CommandState
{
    IDLE,
    SENT,
    ACCEPTED,
    TIMEOUT
};
```

模拟：

```text
t=0 发送
t=0.8 返回 ACK
```

和：

```text
t=0 发送
t=4.0 仍无 ACK
```

要求：

- 主循环始终 20 Hz；
- 不能 `sleep(4s)` 等结果；
- timeout 后恢复成可重试状态。

如果你会写这个，ROS Service 的异步逻辑就不神秘了。

---

# 42. SEARCH 航线和主动边界回收

投放区：

```text
约 5 m × 8 m
```

原工程建立 7 条搜索 lane。

边界不是直接飞到 ±4 m，而是：

```text
搜索横向主动内缩
```

外侧大约：

```text
field_y ≈ ±3.10 m
```

同时还有：

```text
soft limit = ±3.60 m
```

如果真实飞机位置超过：

```text
±3.60 m
```

优先进入回收：

```text
停止继续搜索
↓
生成向内 recovery_target
↓
回到安全范围
↓
恢复当前 lane
```

这说明：

> 规划安全边界和实际状态安全边界应该是两层。

---

# 43. 侦察、返航、落地与显式上锁

基准 v11 仍是：

```text
两瓶完成
↓
RECON_TRANSIT
↓
RECON_DESCEND
↓
RECON_SCAN 6 点拍 RGB
↓
RECON_RETURN_CLIMB
↓
RECON_RETURN
↓
LAND
↓
DISARM
```

侦察函数和投放函数是独立模块。

这很好，因为后续你可以把：

```text
D435i 6 点拍照
```

替换成：

```text
O4 单趟人工图传侦察
```

而不用重写投放状态机。

---

## 43.1 落地确认

当前工程不只看一个“landed”枚举。

它用 odom：

```text
相对高度很低
水平速度很低
垂直速度很低
```

连续稳定一段时间。

这是典型：

> **用多个物理量确认一个不可直接完全信任的离散状态。**

---

## 43.2 显式 DISARM

落地后：

```text
LAND
↓
landing confirmation
↓
DISARM
```

然后发：

```cpp
CommandBool
value = false;
```

而不是“假设飞控自己会锁”。

---

# 44. 故障状态：RETURN_HOME、PILOT_OVERRIDE、ABORT

## 44.1 RETURN_HOME

任务中已经起飞，但任务本身失败。

例如：

- 搜索失败；
- 对准失败；
- 视觉故障但还能导航。

就回：

```text
home 上空
↓
LAND
```

---

## 44.2 PILOT_OVERRIDE

如果飞手切出 GUIDED：

```text
自动节点必须让出控制权
```

关键原则：

> 人工接管是最高优先级，不允许节点又偷偷切回 GUIDED。

如果此时舵机处于释放动作，还要尽可能安全收回。

---

## 44.3 ABORT

主要用于还没正常起飞时：

```text
配置错误
飞前门禁异常
```

未 armed：

```text
直接终止
```

已经 armed：

```text
转 LAND
```

---

# 45. 如何从零逐步拼成完整任务

真正推荐的开发顺序不是按照源文件从上往下写。

而是下面 13 个里程碑。

## Milestone 1：纯 C++ 数据结构

只写：

```text
Point3
distance_xy
normalize_angle
```

通过控制台测试。

---

## Milestone 2：纯 C++ 状态机

状态：

```text
WAIT -> TAKEOFF -> LAND -> DONE
```

全靠时间模拟。

---

## Milestone 3：ROS 2 Node + 20 Hz Timer

只打印：

```text
tick
```

---

## Milestone 4：订阅 FCU State / Odom

只观测，不控制。

验收：

```text
connected
armed
mode
x/y/z
```

都能正确打印。

---

## Milestone 5：发布安全测试 setpoint

先 SITL / Gazebo。

实现：

```text
起飞后一个固定点
```

---

## Milestone 6：轨迹 Segment

实现：

```text
A -> B -> C
```

不要视觉。

---

## Milestone 7：field 坐标

起飞锁定：

```text
home
mission_yaw
```

然后：

```text
field(10,0)
```

无论飞机初始朝向如何，都应该是“机头前方 10 m”。

---

## Milestone 8：飞前门禁

加入：

```text
odom fresh
heading stable
stationary
```

---

## Milestone 9：视觉 body -> local

先只显示：

```text
桶 local 坐标
```

不要飞向桶。

拿已知地面位置验证。

---

## Milestone 10：时间对齐

加入：

```text
navigation_history
pending frame
strict interpolation
```

比较：

```text
未对齐版本
对齐版本
```

动态飞行时误差差异。

---

## Milestone 11：Track + target lock

只打印：

```text
track id
diameter
confirmations
selected target
```

不投放。

---

## Milestone 12：ALIGN

先：

```text
只对准
不动作舵机
```

在日志里验证：

```text
release_point_error
center_error
roll/pitch
```

---

## Milestone 13：Servo + RELEASE gate

最后才接真实投放机构。

---

# 46. 一份教学版完整骨架

这不是完整比赛代码，而是让新人理解“大型节点是如何拼起来的”。

```cpp
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

using SteadyClock =
    std::chrono::steady_clock;
using SteadyTimePoint =
    SteadyClock::time_point;

struct Point3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

enum class State
{
    WAIT_FCU,
    WAIT_NAV,
    WAIT_GUIDED,
    TAKEOFF,
    TRANSIT,
    LAND,
    DONE,
    ABORT
};

class MissionNode final
    : public rclcpp::Node
{
public:
    MissionNode()
    : Node("mission_tutorial")
    {
        state_sub_ =
            create_subscription<
                mavros_msgs::msg::State>(
                "/mavros/state",
                10,
                [this](
                    const mavros_msgs::msg::State::SharedPtr msg)
                {
                    fcu_state_ = *msg;
                });

        odom_sub_ =
            create_subscription<
                nav_msgs::msg::Odometry>(
                "/mavros/local_position/odom",
                rclcpp::SensorDataQoS(),
                [this](
                    const nav_msgs::msg::Odometry::SharedPtr msg)
                {
                    position_ = {
                        msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z
                    };

                    have_odom_ = true;
                    last_odom_ =
                        SteadyClock::now();
                });

        setpoint_pub_ =
            create_publisher<
                geometry_msgs::msg::PoseStamped>(
                "/tutorial/setpoint",
                10);

        state_enter_ =
            SteadyClock::now();

        timer_ =
            create_wall_timer(
                50ms,
                std::bind(
                    &MissionNode::tick,
                    this));
    }

private:
    double age_s(
        const SteadyTimePoint & t) const
    {
        return
            std::chrono::duration<double>(
                SteadyClock::now() - t)
                .count();
    }

    bool odom_fresh() const
    {
        return
            have_odom_ &&
            age_s(last_odom_) < 1.0;
    }

    void enter(State next)
    {
        state_ = next;
        state_enter_ =
            SteadyClock::now();
    }

    void publish_setpoint()
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = "map";

        msg.pose.position.x =
            target_.x;
        msg.pose.position.y =
            target_.y;
        msg.pose.position.z =
            target_.z;

        msg.pose.orientation.w = 1.0;

        setpoint_pub_->publish(msg);
    }

    void tick()
    {
        if (publish_enabled_) {
            publish_setpoint();
        }

        switch (state_) {
            case State::WAIT_FCU:
                if (fcu_state_.connected) {
                    enter(State::WAIT_NAV);
                }
                break;

            case State::WAIT_NAV:
                if (odom_fresh()) {
                    home_ = position_;
                    target_ = position_;
                    publish_enabled_ = true;
                    enter(State::WAIT_GUIDED);
                }
                break;

            case State::WAIT_GUIDED:
                if (fcu_state_.mode == "GUIDED") {
                    // 教学版省略真实 ARM / TAKEOFF service
                    enter(State::TAKEOFF);
                }
                break;

            case State::TAKEOFF:
                // 教学版只演示状态逻辑
                target_ = {
                    home_->x,
                    home_->y,
                    home_->z + 2.0
                };

                if (position_.z >
                    home_->z + 1.8)
                {
                    enter(State::TRANSIT);
                }
                break;

            case State::TRANSIT:
                target_ = {
                    home_->x + 5.0,
                    home_->y,
                    home_->z + 2.0
                };

                if (std::hypot(
                        position_.x - target_.x,
                        position_.y - target_.y)
                    < 0.5)
                {
                    enter(State::LAND);
                }
                break;

            case State::LAND:
                publish_enabled_ = false;

                // 教学版此处应调用 landing service
                if (age_s(state_enter_) > 3.0) {
                    enter(State::DONE);
                }
                break;

            case State::DONE:
                rclcpp::shutdown();
                break;

            case State::ABORT:
                rclcpp::shutdown();
                break;
        }
    }

    State state_ =
        State::WAIT_FCU;

    mavros_msgs::msg::State
        fcu_state_;

    Point3 position_;
    Point3 target_;

    std::optional<Point3> home_;

    bool have_odom_ = false;
    bool publish_enabled_ = false;

    SteadyTimePoint
        last_odom_;
    SteadyTimePoint
        state_enter_;

    rclcpp::Subscription<
        mavros_msgs::msg::State>::SharedPtr
        state_sub_;

    rclcpp::Subscription<
        nav_msgs::msg::Odometry>::SharedPtr
        odom_sub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr
        setpoint_pub_;

    rclcpp::TimerBase::SharedPtr
        timer_;
};

int main(
    int argc,
    char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<MissionNode>());

    rclcpp::shutdown();

    return 0;
}
```

你应该能够指出它与正式工程之间还缺：

```text
参数
航向锁定
轨迹生成
MAVROS service
视觉
时间同步
track
target plan
ALIGN
RELEASE
舵机 ACK
recon
failsafe
landing confirmation
```

如果你能逐项回答“为什么缺”和“该放哪里”，就开始具备自己写大型任务节点的能力。

---

# 47. 原工程函数总表：每个函数到底负责什么

下面按职责把基准 v11 的主要函数全部梳理一遍。

## 47.1 基础数学

| 函数 | 作用 |
|---|---|
| `normalize_angle` | 把弧度角归一化到周期连续形式 |
| `normalize_degrees` | 把角度归一化到 0~360° |
| `distance_xy` | 计算平面距离 |
| `distance_xyz` | 计算三维距离 |
| `value_at<T>` | 安全读取 vector 的第 index 项，越界时 fallback |
| `vector3_at` | 从扁平 double 数组读取第 N 个三维向量 |

## 47.2 参数

| 函数 | 作用 |
|---|---|
| `declare_parameters` | 注册所有 ROS 参数及默认值 |
| `load_parameters` | 读取、限幅、覆盖可靠性 profile，并建立配置契约 |

## 47.3 ROS 回调

| 函数 | 作用 |
|---|---|
| `nav30_ready_callback` | 接收导航 30 Hz 健康状态 |
| `recon_capture_ack_callback` | 接收侦察照片保存 ACK |
| `state_callback` | 更新 FCU 模式/armed，并处理 GUIDED 人工接管 |
| `odom_callback` | 更新位置、速度、姿态、角速度并写导航历史 |
| `extended_state_callback` | 保存 ExtendedState 诊断信息 |
| `compass_callback` | 更新罗盘和航向稳定样本 |
| `bucket_callback` | 验证视觉时间戳并把视觉帧放入等待队列 |

## 47.4 时间同步

| 函数 | 作用 |
|---|---|
| `navigation_sample_at` | 用 before/after odom 严格插值视觉采集时刻导航状态 |
| `steady_age_s` | 计算稳定时钟经过秒数 |
| `process_pending_vision_frames` | 等候后续 odom，处理/拒绝 pending 视觉帧 |
| `process_time_aligned_vision_frame` | 把通过时间同步的视觉检测转为 local 并进入 tracking |
| `reset_visual_target_memory_after_clock_rebase` | ROS 视觉时钟回跳时清历史和目标 |

## 47.5 视觉健康

| 函数 | 作用 |
|---|---|
| `vision_health_ready` | heartbeat + time alignment 都正常 |
| `vision_heartbeat_fresh` | 视觉 topic 最近是否仍有消息 |
| `vision_time_alignment_fresh` | 最近是否有成功完成时间对齐的视觉 |
| `vision_ready_for_takeoff` | 起飞前视觉消息数和新鲜度门禁 |
| `state_requires_vision` | 当前仅 SEARCH 强制持续视觉 |
| `monitor_visual_health` | SEARCH 视觉短断时悬停等待，超时 failsafe |
| `begin_visual_failsafe` | 标记任务失败并返航/降落 |

## 47.6 坐标与姿态

| 函数 | 作用 |
|---|---|
| `geometry_yaw_from_odom` | 任务绝对 yaw + odom 相对 yaw 漂移 |
| `current_geometry_yaw` | 当前几何 yaw |
| `rotate_body_vector_to_local` | `Rz*Ry*Rx` 旋转机体系向量 |
| `body_to_local` | 视觉 body 坐标 + 采集时刻飞机位姿 → local |
| `local_to_body_current` | 当前 local 点反变换到 body |
| `field_to_local` | 任务场地坐标 → local |
| `local_to_field` | local → 任务场地坐标 |

## 47.7 投放口几何

| 函数 | 作用 |
|---|---|
| `release_point_local_current` | 当前真实投放口 local 坐标 |
| `desired_vehicle_pose_for_release_target` | 已知桶坐标和当前姿态，反算飞机中心应去哪里 |
| `desired_release_pose` | active bucket 的包装函数 |
| `release_error` | 当前投放口到桶的 XY 误差 |
| `reset_release_point_velocity_estimator` | 清投放口速度估计器 |
| `update_release_point_velocity_estimator` | 用位置差估计投放口速度并低通 |
| `predicted_release_point_local` | 按机械延迟预测脱离时投放口位置 |

## 47.8 投放区域和目标身份

| 函数 | 作用 |
|---|---|
| `inside_drop_area` | 点是否在投放区（可带 extra） |
| `inside_release_area` | 点是否在最终释放安全内缩区 |
| `bucket_already_used` | 某位置是否接近已投目标 |
| `target_id_already_used` | 某 mission ID 是否已投 |
| `released_position` | 检测是否落在已释放目标附近 |
| `active_target_identity_valid` | active bucket 是否仍符合冻结任务目标身份 |

## 47.9 Track 与目标计划

| 函数 | 作用 |
|---|---|
| `median` | 中值 |
| `median_absolute_deviation` | MAD |
| `smooth_track` | 位置/直径/置信度平滑并更新 confirmations |
| `merge_frame_detections` | detection ↔ known track 数据关联 |
| `track_ready_for_selection` | track 是否达到可选质量 |
| `try_lock_target_plan` | 优先 3 桶选 2；航线结束可 2 桶直接用 |
| `best_bucket` | 返回当前 payload 对应的已选目标 |
| `try_reacquire_active_target_from_frame` | ID 重建时按位置/直径/排他门限重绑定 |
| `accepting_visual_targets` | SEARCH/COARSE/FINE 接收视觉，RELEASE 冻结 |
| `frozen_target_for_payload` | 构造第二瓶的冻结任务目标 |

## 47.10 飞前健康与坐标锁定

| 函数 | 作用 |
|---|---|
| `odom_fresh` | odom 是否新鲜 |
| `compass_fresh` | 罗盘是否新鲜 |
| `extended_state_fresh` | ExtendedState 是否新鲜 |
| `on_ground_reported` | 诊断型 ON_GROUND 判断 |
| `nav30_ready_fresh` | 30 Hz 导航锁是否新鲜 |
| `mean_heading` | 对航向样本做圆均值 |
| `heading_stable` | 航向窗口是否都在允许波动内 |
| `navigation_ready_to_lock` | 起飞前总门禁 |
| `lock_frame` | 锁 home、mission yaw、odom yaw baseline，并构建航线 |

## 47.11 航线

| 函数 | 作用 |
|---|---|
| `build_search_route` | 建立 7 lane 搜索航线 |
| `build_recon_route` | 基准 v11 建立 6 点侦察航线 |
| `start_segment` | 建立一个有时长的梯形速度航段 |
| `sample_segment` | 按当前时间采样航段 setpoint |
| `segment_complete` | 时间到且真实飞机进入终点半径 |
| `slew_target_toward` | 对动态目标做最大速度限制 |

## 47.12 搜索

| 函数 | 作用 |
|---|---|
| `start_search` | 初始化搜索阶段和第一个 segment |
| `update_search_lateral_guard` | 超出 field_y 软边界时先拉回 |
| `update_search` | 锁桶、推进航点、整条航线结束后的 2 桶 fallback |

## 47.13 对准与释放

| 函数 | 作用 |
|---|---|
| `retry_alignment_or_return` | 单次重试或失败返航 |
| `fine_alignment_visual_valid` | 细对准实时视觉是否新鲜且未偏离粗参考 |
| `finish_fine_alignment` | 冻结最后可信桶坐标并进入 RELEASE |
| `update_alignment` | 1.7 m 粗对准 |
| `update_fine_alignment` | 1.3 m 细对准和低空视觉 fallback |
| `update_release` | 最终几何、区域、速度、姿态门禁和舵机动作 |
| `finish_payload_release` | 记录已投目标；第一瓶后直接切第二冻结目标；第二瓶后进侦察 |

## 47.14 侦察

| 函数 | 作用 |
|---|---|
| `publish_recon_photo_mode` | 通知视觉进程切换 PHOTO_ONLY |
| `request_recon_photo` | 发布第 N 个侦察拍照触发 |
| `start_recon_phase` | 两瓶完成后启动侦察 |
| `update_recon_transit` | 高速转场到侦察入口 |
| `update_recon_descend` | 下降进入第一拍照点 |
| `update_recon_scan` | 推进 6 点并触发拍照 |
| `start_recon_return_climb` | 扫描完成后原地爬升 |
| `update_recon_return_climb` | 到返航高度后启动高速回家 |
| `update_recon_return` | 到 HOME 上空后进入 LAND |

## 47.15 返航和落地

| 函数 | 作用 |
|---|---|
| `start_return_home` | 故障返航航段 |
| `update_return` | 故障返航结束后 LAND |
| `relative_altitude` | 当前高度相对 home |
| `landing_candidate` | odom 低高度低速度候选 |
| `reset_landing_confirmation` | 清落地稳定计时 |
| `update_landing_confirmation` | 连续落地确认 |
| `landing_confirmation_ready` | DISARM 前再次确认 |

## 47.16 舵机

| 函数 | 作用 |
|---|---|
| `initialize_servos_if_ready` | 未解锁时把两个舵机初始化到 stowed |
| `send_servo` | 发送 DO_SET_SERVO 异步请求 |
| `check_servo_results` | 处理 initialize/release/stow ACK 和超时 |
| `handle_release_abort_stow` | failsafe/人工接管后持续尝试安全收回 |

## 47.17 飞控命令

| 函数 | 作用 |
|---|---|
| `publish_setpoint` | 发布 `target_` 和锁定 yaw |
| `request_allowed` | 统一 1 Hz 请求限频 |
| `mark_request` | 记录最近请求时间 |
| `request_takeoff` | 发 CommandTOL 起飞 |
| `request_land` | 发 CommandTOL 降落 |
| `request_arm` | 满足安全门禁后自动解锁 |
| `request_disarm` | 落地确认后显式上锁 |
| `check_service_results` | 非阻塞检查 arm/takeoff/land ACK 和 timeout |

## 47.18 全局控制和故障

| 函数 | 作用 |
|---|---|
| `tick` | 20 Hz 总调度器 |
| `flight_gate_ok` | 飞行中共同安全门禁 |
| `mark_mission_failure` | 保存首个终止原因 |
| `abort_or_land` | armed 则 LAND，否则 ABORT |
| `is_automatic_flight_state` | 判断是否处于自动飞行流程 |
| `enter` | 统一状态切换和阶段变量初始化 |
| `state_name` | State → 字符串 |
| `main` | 初始化 ROS、spin 节点、退出 ROS |

---

# 48. 成员变量如何分类，不再被几百个变量吓住

大型 C++ 状态机最吓新人的不是函数，而是：

```text
一大坨成员变量
```

不要逐个死记。

分组。

## 48.1 当前事实

```text
fcu_state_
position_
current_roll_
current_pitch_
current_odom_yaw_
horizontal_speed_m_s_
vertical_speed_m_s_
```

它们回答：

> “飞机现在是什么状态？”

---

## 48.2 任务命令

```text
target_
direct_release_pose_
segment_
```

回答：

> “现在要飞机干什么？”

---

## 48.3 生命周期状态

```text
state_
payload_index_
search_index_
recon_index_
```

回答：

> “任务走到哪里了？”

---

## 48.4 安全/流程 flag

```text
publish_setpoint_
mission_started_
takeoff_sent_
release_started_
return_sent_
servos_initialized_
mission_failed_
```

回答：

> “某个一次性动作到底发生到哪一步？”

---

## 48.5 可选状态

```text
home_
active_bucket_
*_stable_since_
*_start_time_
```

回答：

> “这个信息现在是否存在？”

---

## 48.6 历史/缓存

```text
heading_samples_
navigation_history_
pending_vision_frames_
known_buckets_
```

回答：

> “为了时间、滤波、跟踪，需要记住过去什么？”

---

## 48.7 配置参数

```text
*_speed_m_s_
*_timeout_s_
*_tolerance_m_
*_alt_m_
```

回答：

> “工程策略的可调数字是什么？”

这样看，几百个变量其实只有七类。

---

# 49. 常见错误与调试方法

## 49.1 在 callback 里直接推进任务

错误：

```cpp
odom_callback(...)
{
    if (...) {
        state_ = State::RELEASE;
    }
}
```

问题：

- callback 顺序不可控；
- 状态转换分散；
- 难 debug。

更好的原则：

```text
callback 负责更新事实
tick 负责做决策
```

极少数高优先级事件（如人工切出 GUIDED）可以直接触发 override。

---

## 49.2 用系统当前时间解释旧图像

症状：

```text
飞机越快
桶 local 坐标越偏
```

先检查：

```text
视觉 timestamp
odom history
strict interpolation
```

---

## 49.3 把目标 ID 当作现实物体的永久身份

视觉 track ID 是软件状态。

低空尺度变化后可能重建。

所以要：

```text
ID + 位置 + 直径 + 其他目标排他
```

共同重关联。

---

## 49.4 只看飞机中心，不看机构外参

有投放口、相机、RTK 天线等物理偏置时：

```text
“飞控坐标到了”
```

不等于：

```text
“载荷机构到了”
```

任何高精度任务必须先问：

> 我的控制目标到底是机体哪个物理点？

---

## 49.5 发送命令 = 认为成功

错误：

```cpp
send_servo();
payload_done = true;
```

正确：

```text
send
↓
pending
↓
ACK
↓
物理动作保持时间
↓
stow ACK
↓
任务完成
```

---

## 49.6 在 tick 里阻塞

不要：

```cpp
sleep(2);
wait();
while (...);
```

状态机应该：

```text
每 50 ms 很快跑完一次
```

所有“等待”都写成：

```text
记录时间
以后 tick 再看
```

---

## 49.7 状态切换后忘了清旧变量

典型：

```text
上一次 stable_since_
```

如果不 reset，新状态可能一进入就误判稳定时间已经满足。

所以 `enter()` 是统一清理的好地方。

---

## 49.8 YAML 改了但参数没变

当前基准工程有：

```text
rescue_reliability_profile
```

开启时会强制覆盖一批投放参数。

调参时要同时检查：

```text
declare 默认值
YAML
load_parameters 限幅
profile 强制覆盖
```

最终真正生效的是最后一层。

---

# 50. 新人练习路线与验收标准

不要用“我看懂了”作为验收。

要用：

> “我能不能从空文件重新写出来？”

## Level 1：C++ 基础

必须自己写：

- `Point3`
- `distance_xy`
- `enum class`
- `switch`
- `optional`
- vector 循环
- Lambda 排序

---

## Level 2：ROS 2

必须自己写：

- Node；
- 20 Hz Timer；
- 一个 Subscriber；
- 一个 Publisher；
- 参数；
- throttle 日志。

---

## Level 3：状态机

自己实现：

```text
WAIT
TAKEOFF
WAYPOINT
LAND
DONE
```

所有等待非阻塞。

---

## Level 4：坐标和轨迹

自己实现：

- field ↔ local；
- yaw 锁定；
- Segment；
- slew target。

---

## Level 5：视觉数学

自己实现：

- odom history；
- strict timestamp interpolation；
- body → local。

---

## Level 6：tracking

自己实现：

- track ID；
- confirmations；
- EMA；
- median；
- 3 选 2。

---

## Level 7：投放几何

自己推导：

\[
p_{release}
=
p_{vehicle}
+
Rr
\]

并能解释：

> 为什么 `z=-0.32 m` 在飞机倾斜时会影响 XY。

---

## Level 8：工程可靠性

自己实现：

- ACK；
- timeout；
- retry；
- failsafe；
- stable_since；
- manual override。

达到 Level 8，才算真正能维护当前这种任务代码。

---

# 51. 继续升级：O4 侦察、12 m/s 非任务区转场应该怎么改

这是基于当前讨论的下一阶段任务，不属于基准 v11 已实现内容。

当前 v11 仍是：

```text
D435i PHOTO_ONLY
6 个侦察拍照点
1.2 m
```

如果改为 DJI O4 人工图传侦察，建议从软件架构上做：

## 51.1 不要只改速度参数

还应删除/修改：

```text
6 点照片 ACK 成功条件
request_recon_photo()
PHOTO_ONLY 依赖
RECON_DESCEND 的 6 点流程
DONE 中 recon_photo_saved_ids_ 数量要求
```

---

## 51.2 把侦察变成一个连续任务段

更合适的结构：

```text
RECON_TRANSIT
↓
RECON_SCAN_SINGLE_PASS
↓
RECON_RETURN
```

状态越少越好。

不要为了“看起来完整”保留已经不需要的状态。

---

## 51.3 非任务区 12 m/s

应该区分：

```text
正常比赛高速转场
故障返航
```

是否都 12 m/s。

工程上通常建议：

```text
正常高速段：12 m/s
任务区：按识别/投放要求降速
故障返航：保留更保守速度
```

如果最终比赛策略明确要求所有非任务区都 12 m/s，再统一调整对应参数。

---

## 51.4 修改后一定重新检查飞控侧限制

任务代码里的：

```text
max_speed = 12
```

表达的是任务层 setpoint 轨迹峰值。

真机能否达到还取决于：

- ArduPilot 速度限制；
- 加速度限制；
- 最大倾角；
- 动力余量；
- 载荷；
- 风；
- 高度保持能力。

所以：

> **任务层速度参数不是物理速度保证。**

---

# 52. 最终心法

如果只记住十句话，记这些：

1. **callback 更新事实，tick 做决策。**
2. **measurement 和 command 永远分开：`position_` ≠ `target_`。**
3. **状态切换统一走 `enter()`。**
4. **任何等待都不要阻塞，用 timestamp + future。**
5. **任何“成功”都尽量等 ACK 或物理条件确认。**
6. **图像必须用采集时刻的飞机姿态解释。**
7. **目标检测不是目标身份，Track 才接近任务身份。**
8. **高精度控制先问清楚控制的是飞机哪个物理点。**
9. **安全不是一个 if，而是门禁、稳定时间、timeout、retry、fallback、override 的组合。**
10. **完整大代码不是一次写出来的，是十几个可独立验证的小系统逐层拼出来的。**

真正掌握以后，你看 `cuadc_full_mission_node_3_v11.cpp` 不应该再看到“一千多行复杂代码”，而应该看到：

```text
基础数学
+ ROS IO
+ 参数
+ 状态机
+ 坐标系
+ 轨迹
+ 时间同步
+ tracking
+ 目标计划
+ 几何对准
+ 异步命令
+ failsafe
```

每一层都可以单独测试、单独替换、单独升级。

这才是从“会改别人代码”走到“能自己设计无人机任务软件”的分界线。

---

# 附录 A：建议的 ROS 2 功能包依赖

如果把教学版逐渐扩展到基准工程，`package.xml` / `CMakeLists.txt` 至少会涉及：

```text
rclcpp
geometry_msgs
nav_msgs
std_msgs
mavros_msgs
```

示意 `CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.8)
project(cuadc_mission_tutorial)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

add_executable(
  mission_node
  src/mission_node.cpp
)

target_compile_features(
  mission_node
  PUBLIC cxx_std_17
)

ament_target_dependencies(
  mission_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  mavros_msgs
)

install(
  TARGETS mission_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

编译：

```bash
colcon build --packages-select cuadc_mission_tutorial
source install/setup.bash
```

---

# 附录 B：推荐的源码目录拆分方式

当前单文件适合比赛快速迭代和集中审查；新人真正学会后，可以考虑拆成：

```text
include/cuadc/
    types.hpp
    geometry.hpp
    trajectory.hpp
    tracker.hpp
    mission_state.hpp

src/
    geometry.cpp
    trajectory.cpp
    tracker.cpp
    mission_node.cpp
```

但不要为了“架构漂亮”过早拆文件。

判断标准：

> 一个模块是否已经有稳定接口、独立测试价值，并且主文件阅读负担已经明显增加？

如果没有，就先保持简单。

---

# 附录 C：学习时推荐自己打印的关键日志

飞前：

```text
FCU connected
armed
mode
odom age
nav30
heading variation
vision heartbeat
vision aligned
servo initialized
```

搜索：

```text
state
search index
field x/y
known tracks
track id
confirmations
diameter
confidence
deviation
```

对准：

```text
target id
center error
release point error
height error
visual age
roll
pitch
yaw delta
```

释放：

```text
identity_ok
area_ok
geometry_ok
motion_ok
attitude_ok
stable time
servo pending
servo ACK
```

返航/落地：

```text
distance home
relative altitude
horizontal speed
vertical speed
landing stable time
armed
```

日志的目的不是“越多越好”。

而是：

> **每一个状态转换，都能从日志回答“为什么发生”。**
