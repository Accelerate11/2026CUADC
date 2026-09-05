# CUADC C++：从零基础到完整任务状态机

## · C++17 / ROS 2 Humble / MAVROS

> **给谁读**：能使用电脑，但没有系统学习过C++的人；也适合会改代码、却不清楚引用、生命周期和异步语义的队员。
> **学习目标**：从一个可编译的main出发，逐步建立阅读和编写CUADC任务程序所需的语言、标准库与软件组织能力。
> **写作边界**：不是ISO C++全语言百科；重点覆盖基准工程实际使用的C++17知识及必要先修。原工程事实、语言背景、教学简化与改进分别说明。
> **安全边界**：前51章可离线学习；配套纯C++例程无硬件控制接口。ROS练习使用隔离教学话题和加法服务，不提供真实起飞/投放入口。
> **日期**：2026-09-05。原教程与原v11文件均保留，本书不覆盖它们。

## 先看这一页：怎样使用这份长教程

原稿按工程模块快速介绍了很多概念；这次改为按知识依赖展开。不要从模板回调或完整节点开读。先把“声明与执行”“变量与对象”“值与引用”“拥有与借用”分清，再看ROS长类型。

本书中标为**完整例程**的代码都有对应文件、构建命令和实际捕获输出；标为**语法片段/源码摘录**的代码用于局部讲解，不承诺单独复制就能编译。故意错误的例子有明确标记，不能当作可运行参考。

练习顺序固定为：读需求 → 手算/预测 → 编译运行 → 比较结果 → 做一处变更 → 给变更补测试。暂时不会的前置概念应回到对应章节，而不是继续背更长的一行。

| 你目前的情况 | 从哪里开始 | 暂时不要做什么 |
|---|---|---|
| 没学过C++ | 第1—12章、例程01—06 | 不接飞控、不看完整节点 |
| 会if/for，但引用和类薄弱 | 第13—28章 | 不先写多线程或复杂模板框架 |
| 会基础C++，读不懂ROS长类型 | 第29—44章，再到第52章 | 不把API调用当语言关键字 |
| 能写ROS节点，想维护任务代码 | 第45—64章及源码函数索引 | 不把教学简化程序直接部署真机 |

**配套目录提示**：下面编译命令假定当前工作目录是ZIP解压后、含根CMakeLists.txt的目录；单独复制某个例程时可改用相应本地文件路径。

## 可点击目录


**第一篇：先让一个小程序真正运行起来**

- [第1章：先分清我们在学什么，再决定从哪里开始](#ch01)
- [第2章：文件、终端、编辑器、编译器，分别是什么](#ch02)
- [第3章：第一个程序，逐个符号讲清楚](#ch03)
- [第4章：程序为什么会失败——预处理、编译、链接与运行](#ch04)

**第二篇：用数据和控制流程描述任务**

- [第5章：变量、声明、初始化、赋值，不是一回事](#ch05)
- [第6章：类型、取值范围、`size_t` 与有符号/无符号](#ch06)
- [第7章：整数除法、浮点误差、字面量与单位转换](#ch07)
- [第8章：输入、输出、字符串，先从可靠读取一个数开始](#ch08)
- [第9章：运算符与表达式，把每个条件读成一句话](#ch09)
- [第10章：`if`、`else` 与提前返回——把错误入口挡在外面](#ch10)
- [第11章：循环不是“程序卡住”，关键在于每次推进了什么](#ch11)
- [第12章：第一次阶段实践——写一个离线任务条件检查器](#ch12)

**第三篇：函数、引用、指针与数据结构**

- [第13章：函数从零——谁调用谁，值从哪里来，执行到哪里回去](#ch13)
- [第14章：函数声明、定义、重载、默认参数与接口设计](#ch14)
- [第15章：引用与参数传递——把 `const T&` 讲透](#ch15)
- [第16章：指针、地址、解引用与空值——不跳过任何一层](#ch16)
- [第17章：作用域、生命周期、`const`、`constexpr`、`static` 与命名空间](#ch17)
- [第18章：数组、字符串、`char** argv` 与字符串视图](#ch18)
- [第19章：`struct` 与 `enum class`——第一次建立任务的数据语言](#ch19)
- [第20章：第二次阶段实践——离线航点检查器](#ch20)

**第四篇：对象、类、资源与所有权**

- [第21章：`class`、对象、成员函数与 `this`](#ch21)
- [第22章：构造函数与初始化列表——冒号后面到底发生什么](#ch22)
- [第23章：析构、RAII、异常、`noexcept`——资源怎样可靠收尾](#ch23)
- [第24章：复制、移动、左值右值与 Rule of Zero](#ch24)
- [第25章：`unique_ptr`——一个资源只有一个拥有者](#ch25)
- [第26章：`shared_ptr`、`weak_ptr` 与 ROS 的 `SharedPtr`](#ch26)
- [第27章：继承、虚函数、`override`、`final` 与组合](#ch27)
- [第28章：函数指针、成员函数指针、`std::function` 和 `std::bind`](#ch28)

**第五篇：读懂现代 C++ 的类型、容器和算法**

- [第29章：`auto`、类型别名、`decltype` 与长类型拆解](#ch29)
- [第30章：Lambda 逐个符号拆解——捕获、参数、返回、生命周期](#ch30)
- [第31章：模板从最简单函数开始——类型参数不是运行参数](#ch31)
- [第32章：`std::array` 与 `std::vector`——长度、容量、元素和生命周期](#ch32)
- [第33章：`deque`、队列、历史窗口与移动后删除](#ch33)
- [第34章：`optional`、`pair`、`tuple`、`variant`——不要用一个零代表所有未知](#ch34)
- [第35章：迭代器与半开区间——理解 `begin()`、`end()` 和失效](#ch35)
- [第36章：查找和判定算法——从手写循环到 `find_if`、`all_of`、`lower_bound`](#ch36)
- [第37章：排序、比较器、删除与归约——不要把容器算法当黑盒](#ch37)
- [第38章：map、set、哈希容器与复杂度——什么时候不用 vector](#ch38)

**第六篇：把语言知识变成可验证的工程代码**

- [第39章：数学工具、有限值、范围检查与“安全默认值”的边界](#ch39)
- [第40章：文件、流、解析与日志——读到字符串不等于读到合法数值](#ch40)
- [第41章：从一个 `.cpp` 到多文件工程——头文件、ODR 与 CMake](#ch41)
- [第42章：调试、测试、警告与 Sanitizer——怎样证明修改没有破坏旧行为](#ch42)
- [第43章：`chrono` 从时间单位讲到时间点——为什么不能拿所有时间互相减](#ch43)
- [第44章：异步从零——`promise`、`future`、线程、锁与 ROS 回调](#ch44)

**第七篇：把已经学会的 C++，组合成可测试的任务零件**

- [第45章：状态机不是 `switch` 的别名，而是跨调用保存进度](#ch45)
- [第46章：连续稳定门——把布尔值、时间和 `optional` 串起来](#ch46)
- [第47章：从 `Point3` 到投放口几何，先理解“数据代表什么”](#ch47)
- [第48章：轨迹函数，按公式一步一步实现](#ch48)
- [第49章：导航历史与时间插值，把容器和算法真正用起来](#ch49)
- [第50章：跟踪与目标冻结——从“一次测量”变成“任务对象”](#ch50)
- [第51章：阶段大实践——完整但不接飞机的任务状态机](#ch51)

**第八篇：现在才进入 ROS 2——把框架调用读回普通 C++**

- [第52章：一个 ROS 节点，究竟何时构造、何时运行](#ch52)
- [第53章：消息、发布和订阅的类型，逐层解开](#ch53)
- [第54章：参数不是全局变量——声明、加载、验证和生效](#ch54)
- [第55章：执行器、回调组和对象寿命，为什么程序会卡住](#ch55)
- [第56章：ROS异步服务，完整示范pending请求的生命周期](#ch56)
- [第57章：MAVROS只是换了消息与服务类型，不能省掉业务判断](#ch57)
- [第58章：阅读4345行v11的正确顺序，不是从第1行一路硬啃](#ch58)

**第九篇：带着语言知识逐段阅读正式源码**

- [第59章：源码精读一——移动队列元素、查找轨迹与忽略返回值](#ch59)
- [第60章：源码精读二——粗对准、细对准与RELEASE门禁](#ch60)
- [第61章：失败处理、返航与原版侦察，别把日志当物理事实](#ch61)
- [第62章：重构时先保留行为，再谈模块漂亮](#ch62)
- [第63章：常见错误字典——从症状倒查C++知识点](#ch63)
- [第64章：学习安排与总验收——不是读完多少行，而是能独立解释多少行为](#ch64)
- [附录A：同一个符号为什么有不同意思](#appendix-a)
- [附录B：标准库与ROS常用接口速查](#appendix-b)
- [附录C：与原版教程相比，哪些地方明确澄清或改变了](#appendix-c)
- [附录D：配套纯C++核心模块完整文件](#appendix-d)
- [附录E：单元测试不是注释——完整测试文件](#appendix-e)
- [附录F：故意写错，再读懂编译器](#appendix-f)
- [附录G：实践索引与知识点覆盖](#appendix-g)
- [附录H：基准v11函数与实际行号索引](#appendix-h)
- [附录I：如何读“看起来最复杂的一行”](#appendix-i)
- [附录J：来源、版本与验证边界](#appendix-j)

---

# 第一篇：先让一个小程序真正运行起来

<a id="ch01"></a>
# 第 1 章：先分清我们在学什么，再决定从哪里开始

## 1.1 目标不是背代码，而是建立四种能力

学习这份教材的终点，是能够从空文件写出一个可以解释、可以测试、可以维护的任务程序。看到 `const Point3&` 时，你应当知道它怎样传递数据；看到 `[this]` 时，应当知道谁必须继续活着；看到 `future.wait_for(...)` 时，应当知道程序到底有没有在等待；看到 `State::RELEASE` 时，应当能说明它为何进入、何时离开，以及异常时如何退出。

这里有四个不同层次，不能同时混学：

| 层次 | 要解决的问题 | 例子 |
|---|---|---|
| C++ 语言 | 怎样表达数据、操作与生命周期 | 变量、函数、引用、类、模板 |
| C++ 标准库 | 怎样使用通用工具 | `vector`、`optional`、`chrono`、`future` |
| ROS 2 框架 | 多个程序怎样交换消息和调度回调 | Node、Topic、Timer、Service |
| CUADC 业务 | 怎样搜索、对准、投放、侦察和退出 | 状态机、目标记忆、稳定门禁 |

原教程较早把四层写在一起。例如，刚接触函数的读者就要同时面对 `Point3`、`const`、引用和 `std::hypot`。本版先把每一种机制单独做成小实验，之后再组合。**前 44 章的主要任务是学习 C++，不是配置飞机。**

## 1.2 只需要一条主线

```text
认识文件和编译器
→ 变量、表达式、分支、循环
→ 函数、引用、指针、作用域
→ 结构体、类、构造、析构、所有权
→ 可调用对象、Lambda、模板、容器、算法
→ 多文件工程、测试、时间、异步
→ 纯 C++ 任务组件与模拟状态机
→ ROS 2 外壳与 v11 源码阅读
```

第一次学习，不要从最后的 ROS 完整类开始向前猜。每篇结束做一个能够运行的程序，再进入下一篇。最后的函数速查和源码索引用于复习，不是初学顺序。

## 1.3 这份教材中的三种材料

**已有材料**是你上传的旧 C++ 教程，以及 `cuadc_full_mission_node_3_v11(1).cpp`。涉及实际函数行为时，以这份上传源码为依据，不把此前讨论的 O4、高速转场等设想当成既有代码。

**教学补充**是本版新增的语言说明、内存示意、标准库解释、练习和重构方法。它们不是从原代码中“发现”的新功能。

**教学改写**是为了演示某个概念而重新写的小程序。它可以比 v11 更简单，也可能有额外的输入检查；凡行为不同之处会明确说明。它不能替代经过验证的比赛节点。

## 1.4 什么是“会了”

“读的时候觉得懂”不等于“能写”。每一阶段至少完成三件事：不看答案复写一个小程序；修改一个输入并预测结果；故意破坏一个条件，解释错误发生在哪一层。能解释失败，比只跑通一次更有价值。

整个配套工程默认只操作本地内存和教学文件。ROS 练习使用 `/cpp_tutorial/...` 命名空间，不提供真实 MAVROS 解锁、起飞或舵机发送程序。**不要为了试例程而把这些话题重映射到真机控制话题。**

**材料依据**：旧教程 [B0]、上传 v11 [B1]。语言范围采用 C++17；后续标准功能会标明不属于本书默认编译环境。

<a id="ch02"></a>
# 第 2 章：文件、终端、编辑器、编译器，分别是什么

## 2.1 一个 `.cpp` 文件只是一份文本

文件扩展名 `.cpp` 告诉人和工具“这里通常放 C++ 源代码”。它不是可执行程序。把文件重命名成 `fly` 并不会让它变成程序，给源文件增加执行权限也不会替代编译。

编辑器负责写文本；编译器负责检查 C++ 规则并产生机器代码；终端负责把你输入的命令交给操作系统。VS Code 是编辑器，`g++` 是编译器，Bash 是一种命令解释器。一个工具可以调用另一个工具，但它们不是同一件事。

## 2.2 先掌握最少的目录操作

以下是终端命令，不要写进 `.cpp` 文件：

```bash
mkdir -p ~/cuadc_cpp_basics
cd ~/cuadc_cpp_basics
pwd
ls
```

`mkdir -p` 创建目录；`cd` 改变当前工作目录；`pwd` 显示当前目录；`ls` 查看文件。`~` 表示当前用户的主目录。你在终端运行 `./hello` 时，`./` 表示从当前目录找文件，不是 C++ 语法。

文件名带空格或括号时应加引号：

```bash
wc -l 'cuadc_full_mission_node_3_v11(1).cpp'
```

不要把复制过程中附带的行号、`$` 提示符、Markdown 的三个反引号一起粘进文件。文件应使用纯文本、UTF-8 编码，扩展名不要误存成 `.cpp.txt`。

## 2.3 准备编译工具

Ubuntu 的教学环境可以通过包管理器安装构建工具。下面只安装本地开发工具，不刷写或配置飞控：

```bash
sudo apt update
sudo apt install build-essential cmake gdb python3
g++ --version
cmake --version
```

`sudo` 是以管理员权限执行系统操作，不需要用它运行你的普通 C++ 练习。安装命令属于环境准备，源码语义以 `-std=c++17` 为准。其他系统也可以学习同一套 C++，只是工具安装和可执行文件形式不同。

## 2.4 最值得记住的一条编译命令

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -g hello.cpp -o hello
./hello
```

| 部分 | 意义 |
|---|---|
| `g++` | 调用 C++ 编译驱动程序 |
| `-std=c++17` | 使用 C++17 语言模式，不依赖编译器默认版本 |
| `-Wall -Wextra -Wpedantic` | 打开常用诊断；不表示发现所有可能错误 |
| `-g` | 生成调试信息，供调试器使用 |
| `hello.cpp` | 输入源文件 |
| `-o hello` | 输出程序命名为 `hello` |
| `./hello` | 执行编译得到的程序 |

**编译成功后再运行。** 更稳妥的写法是用 `&&` 连接：

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic hello.cpp -o hello && ./hello
```

这是 Shell 的条件执行，不是 C++ 的 `&&` 表达式。如果新版本编译失败，而目录里保留着上一次的 `hello`，直接运行可能误以为新代码生效。

**小练习**：在不同目录运行 `./hello` 会怎样？答案：操作系统在不同的当前目录查找，可能提示文件不存在；这与 C++ 语法没有关系。

参考：[R01]、[R02]。

<a id="ch03"></a>
# 第 3 章：第一个程序，逐个符号讲清楚

<a id="ex01"></a>
**完整例程 01：`examples/01_hello.cpp`**

```cpp
#include <iostream>

int main()
{
    std::cout << "Hello, CUADC!\n";
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/01_hello.cpp -o build/ex_01_hello && \
  ./build/ex_01_hello
```

**本次运行捕获的标准输出**：

```text
Hello, CUADC!
```

## 3.1 `#include <iostream>`

`#include` 是预处理指令，作用是让这个翻译单元能够使用指定头文件提供的声明及相关定义。这里的 `iostream` 提供标准输入输出流相关接口。

尖括号不是“小于、大于”；在这条预处理指令里它们包围头文件名。后面写自己的头文件通常使用双引号，例如 `#include "geometry.hpp"`。尖括号和双引号的搜索细节由工具链配置决定，初学时理解为“标准或外部头文件”和“本工程头文件”即可。

`#include` 行后不要随手加 `;`。头文件提供的是编译时需要的信息；链接库的问题在第 4、41 章另讲。

## 3.2 `int main()`

`main` 是普通宿主环境下程序的入口函数名。`int` 表示函数返回整数；`()` 表示这里没有显式列出的参数；紧随其后的 `{...}` 是函数体。操作系统启动程序所需的运行库初始化仍然存在，不应把 `main` 理解成机器执行的第一条指令。

此时只需把 `main` 当成“我的程序从这里开始执行”。不要写 `void main()`。以后 ROS 版本的 `main(int argc, char** argv)` 只是多了命令行参数，并没有换一种语言。

## 3.3 `std::cout << "Hello, CUADC!\n";`

`std` 是标准库使用的命名空间名称。`::` 是作用域解析符，所以 `std::cout` 表示“标准库命名空间中的 cout”。`cout` 是标准输出流对象。

这里的 `<<` 是流插入运算，意为把右侧内容写入左侧流。它也可以连续使用：

```cpp
// 放在 main 函数体内的语法片段。
std::cout << "height=" << 1.3 << " m\n";
```

这不是把三个东西做二进制左移。C++ 允许同一运算符根据操作数类型表示不同操作，后面称为运算符重载。

双引号包围的是字符串字面量。`\n` 表示换行字符，不是两次输出反斜杠和字母 n。常见转义只需先记 `\n`、`\t`、`\\`、`\"`。中文文本可放在字符串或注释中，但程序标识符建议使用稳定、清楚的英文名。

## 3.4 `return 0;` 和分号

`return 0;` 结束 `main`，向调用环境报告成功状态。非零退出码通常用于报告失败，但每个具体数字的含义由程序约定。返回值不是“让飞机高度变成零”，也不是打印零。

分号结束一条语句。花括号组织代码块。缩进有助于人阅读，但一般不改变 C++ 语义。以下写法虽然难看，表达同样的程序：

```cpp
// 不推荐的排版示例。
int main(){return 0;}
```

类、结构体和枚举定义结束需要 `};`，普通函数定义结束通常只写 `}`。不要机械地给所有右花括号加分号。

## 3.5 注释不是程序命令

```cpp
// 单行注释：到行末结束。
/* 多行注释：
   可以跨行，但不能按普通方式嵌套。 */
```

注释可以过时，所以阅读实际工程时，必须检查注释与执行语句是否一致。比如“10 Hz”的旧注释不一定等于定时器真实周期。

**练习**：把输出分成两行，分别写 `CUADC` 和 `offline exercise`。再删掉一句的分号，观察编译错误指向哪里，然后恢复。错误信息有时在下一行暴露，但根因在前一行。

参考：C++17 `[basic.start.main]`、`[lex.comment]`、`[iostream.objects]`，见 [R03]。

<a id="ch04"></a>
# 第 4 章：程序为什么会失败——预处理、编译、链接与运行

## 4.1 从文本到进程的四步

```text
.cpp 和头文件
    ↓ 预处理：展开 include、条件编译等
翻译单元
    ↓ 编译与汇编：检查语法、类型，产生目标代码
.o 目标文件
    ↓ 链接：把各目标文件和库中需要的定义连接起来
可执行文件
    ↓ 运行
进程及其内存、文件、线程等资源
```

`g++ hello.cpp -o hello` 通常替你串起了中间步骤，所以你不一定看到 `.o`。需要观察时可以分开执行：

```bash
g++ -std=c++17 -E hello.cpp -o hello.ii
g++ -std=c++17 -c hello.cpp -o hello.o
g++ hello.o -o hello
```

`-E` 只做预处理，输出可能很长；`-c` 产生目标文件而不进行最终链接。

## 4.2 四种错误，不要混着查

| 现象 | 所在层 | 优先检查 |
|---|---|---|
| `iostream: No such file` 或找不到项目头文件 | 预处理/环境 | 工具链是否正确、include 路径 |
| `expected ';'`、类型不匹配 | 编译 | 当前行与前一行、声明、参数类型 |
| `undefined reference to ...` | 链接 | 有没有函数定义、是否加入相应 `.cpp` 或库 |
| 程序启动后崩溃、结果错误、一直等待 | 运行 | 生命周期、边界、输入数据、状态条件 |

“没有报编译错”只说明满足了编译器能检查的规则。访问已销毁对象、读数组之外的内存、数据竞争，都可能成功编译。

## 4.3 什么是未定义行为

未定义行为不是“系统默认返回一个随机值”，而是 C++ 标准不对该执行结果提出要求。你不能据一次运行正常推断下次也正常。初学阶段最常见的是未初始化标量读取、数组越界、悬空引用解引用、某些整数溢出，以及多线程中的未同步读写。

不要用“我这台电脑能跑”作为反驳。后面会用警告、测试、调试器和 Sanitizer 帮助发现这些问题，但没有任何单一工具能证明无人机任务安全。

## 4.4 一个重要习惯：每次只增加一个难点

先让 `hello.cpp` 成功，再加变量；变量成功再加分支。出现 200 行模板错误时，先看**第一条属于你自己代码的错误位置**，不要从最后一行开始乱改。

**本篇验收**：能区分源码和程序；能从终端编译；能解释 `#include`、`main`、`std::cout`、分号和返回码；能把失败归到环境、编译、链接或运行之一。

参考：[R01]、C++17 `[lex.phases]`、`[intro.execution]`，见 [R03]。

# 第二篇：用数据和控制流程描述任务

<a id="ch05"></a>
# 第 5 章：变量、声明、初始化、赋值，不是一回事

## 5.1 一个变量最少包含哪些信息

```cpp
// 函数体内的语法片段。
double height_m = 1.3;
```

`double` 是类型，告诉编译器这是浮点数；`height_m` 是名字；`1.3` 是初始值。声明告诉编译器名字及其类型；这里同时是定义，因为它创建了对象；初始化是对象创建时确定初始状态。

之后写：

```cpp
height_m = 1.7;
```

这是赋值：对象已经存在，改变它保存的值。等号在这里不表示数学上的永久相等关系。`height_m = height_m + 0.4;` 是先计算右侧，再把结果交给左侧。

## 5.2 三种常见初始化写法

```cpp
int count = 2;       // 复制初始化，不等于一定发生一次额外复制。
int other(2);        // 直接初始化。
int third{2};        // 列表初始化。
double height{};    // 对这个 double，初始化为 0.0。
```

初学时普通数值可以使用 `=`；定义聚合数据常用 `{}`。列表初始化还会拒绝某些窄化转换，例如 `int n{1.3};` 不能用来悄悄丢弃小数。

不要把所有花括号都叫“代码块”。`if (...) { ... }` 的花括号包围语句；`Point3{1,2,3}` 的花括号包围初始化数据。看它所在的位置才能判断用途。

## 5.3 未初始化不代表零

```cpp
// 错误示例，不要运行。
int count;
// std::cout << count; // 对该局部 int 的未初始化读取没有可依赖的结果。
```

不同存储期和不同类型有不同初始化规则。最实用的习惯是：**在定义时就给普通局部数值一个明确初值。** 不要依赖调试器恰好显示的零。

同时，零并不总是合法的“未知状态”。高度为零可以是地面位置，不能自动表示“没有高度”。后面用 `optional` 单独表达“没有值”。

## 5.4 名字需要携带业务单位

`height_m` 比 `h` 明确；`timeout_s` 比 `timeout` 明确；`angle_rad` 和 `angle_deg` 不应混用。尾部下划线，如 `position_`，是项目用来识别成员变量的命名习惯，不是 C++ 强制规定，也不意味着线程安全。

<a id="ex02"></a>
**完整例程 02：`examples/02_values.cpp`**

```cpp
#include <iostream>
#include <string>

int main()
{
    int payload_count = 2;
    double height_m = 1.3;
    bool connected = false;
    std::string mode = "WAIT";

    height_m = height_m + 0.4;
    connected = true;
    mode = "READY";

    std::cout << "payload=" << payload_count << '\n';
    std::cout << "height_m=" << height_m << '\n';
    std::cout << std::boolalpha << "connected=" << connected << '\n';
    std::cout << "mode=" << mode << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/02_values.cpp -o build/ex_02_values && \
  ./build/ex_02_values
```

**本次运行捕获的标准输出**：

```text
payload=2
height_m=1.7
connected=true
mode=READY
```

**逐步追踪**：`height_m` 先是 1.3，赋值后是 1.7；`connected` 从 false 变为 true；`mode` 从 WAIT 变为 READY。`std::boolalpha` 只是让布尔输出显示成英文 true/false，不会改变变量类型。

**回到 v11**：`position_` 是观测状态，`target_` 是任务命令。两者即使都是 `Point3`，也不能因为类型相同就混成一个变量。

**练习**：定义“剩余瓶数”“图像是否有效”“投放口偏移米数”三个变量，分别选择类型。答案：通常是整数、布尔值、浮点数；若有效性有更多原因，应再引入状态或诊断信息。

参考：C++17 `[basic.def]`、`[dcl.init]`，见 [R03]。

<a id="ch06"></a>
# 第 6 章：类型、取值范围、`size_t` 与有符号/无符号

## 6.1 类型是操作约束，不只是占几个字节

| 类型 | 适合表达 | 不宜表达 |
|---|---|---|
| `bool` | 一个明确的真/假条件 | 很多种错误原因 |
| `int` | 小规模计数、一般整数运算 | 假定任何平台都固定 32 位 |
| `double` | 米、秒、弧度、比例 | 永久唯一 ID、精确计数 |
| `std::size_t` | 对象大小、标准容器索引 | 可为负的差值 |
| `std::int64_t` | 需要固定宽度的有符号整数，类型存在时 | 不经检查替换所有类型 |
| `std::uint32_t` | 位掩码、协议中的无符号字段 | 负坐标 |

`sizeof(T)` 返回 `T` 对象所占的字节数，其结果类型是 `std::size_t`。标准不要求普通 `int` 和 `long` 在所有平台具有相同宽度。`<cstdint>` 的精确宽度类型在实现支持该宽度时提供；本书测试平台具有这些类型。

## 6.2 为什么索引经常写成 `std::size_t`

`vector.size()` 的返回类型是无符号大小类型。把循环变量也写成相容的索引类型，可以减少无意义的有符号/无符号比较。不过，“用了 size_t”不代表不会出错。

```cpp
// 危险思路：空容器的 size()-1 不是 -1。
// for (std::size_t i = values.size() - 1; i >= 0; --i) { ... }
```

无符号整数不能表示负数，`i >= 0` 始终成立。倒序循环可以改成：

```cpp
// 此片段在 values 已定义时使用。
for (std::size_t i = values.size(); i > 0; --i) {
    // 使用 values[i - 1]。
}
```

## 6.3 转换之前先检查意义

```cpp
// 已知 count 是可能为负的外部输入。
if (count < 0) {
    // 拒绝输入。
} else {
    std::size_t index = static_cast<std::size_t>(count);
    // 转换以后还要检查是否小于容器 size()。
}
```

`static_cast` 不会替你验证业务含义。把 -1 转成无符号数，并不能修复“无效索引”，只会得到很大的值。向更窄整数转换、浮点转整数，也需要检查范围。

## 6.4 整数溢出和协议字段

无符号运算按模数规则回绕；有符号整数溢出在 C++17 中不能作为正常业务逻辑依赖。例如无限递增的 ID，应考虑范围和回绕后重复风险。业务程序常用更宽类型减少风险，但这不等于从数学上消除上限。

**回到 v11**：舵机参数数组由 ROS 参数接口读取为整数数组，源码使用 `int64_t`；它随后转成 MAVROS 请求中的浮点字段。读代码时要跟踪“读取类型 → 范围检查 → 转换类型”，不能只看终端打印的 1500。

**练习**：`std::size_t index = 0; --index;` 是否得到 -1？答案：不会；它发生无符号回绕。不要把这当作寻找前一个航点的办法。

参考：C++17 `[basic.fundamental]`、`[conv.integral]`、`[expr.sizeof]`，见 [R03]。

<a id="ch07"></a>
# 第 7 章：整数除法、浮点误差、字面量与单位转换

## 7.1 左侧是 `double`，右侧也可能先做错运算

```cpp
int distance = 5;
int speed = 2;
double time = distance / speed;
```

结果是 2.0，而不是 2.5。因为右侧两个操作数都是整数，先进行整数除法，再把得到的整数转换成 double。正确方式是在除法之前让至少一个操作数成为浮点：

```cpp
double time = static_cast<double>(distance) / speed;
```

这不是“把答案转成 double”，而是“改变计算本身使用的类型”。仍然必须确认 `speed != 0`。

## 7.2 数字后缀是什么意思

`2` 通常是 int；`2U` 是无符号整数字面量；`2.0` 默认是 double；`2.0f` 是 float；`1.0e-3` 是 0.001。`0U` 不代表“单位 U”，`1.0e-6` 也不是减法表达式。

原代码中频繁出现 `0.0`、`0U`、`1.0e-6`，是为了表达合适的类型和数量级。ROS 的某些消息字段是 float、某些是 double，实际以接口定义为准。

## 7.3 浮点数为什么不能总用 `==`

许多十进制小数不能被有限长度的二进制浮点精确表示。常见平台上，`0.1 + 0.2` 与直接写 `0.3` 可能出现极小差异。它不是程序“漂了 0.1 米”，而是数值表示误差。

工程比较应区分两件事：数值计算的舍入容差，以及真实传感器的测量误差。不能把测试用的 `1e-9` 直接当作投放精度。比较理论公式可以用小容差；比较飞行误差应该用有物理意义的米、秒或弧度阈值。

<a id="ex03"></a>
**完整例程 03：`examples/03_numbers.cpp`**

```cpp
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>

int main()
{
    int distance_m = 5;
    int speed_m_s = 2;
    double wrong_time_s = distance_m / speed_m_s;
    double time_s = static_cast<double>(distance_m) / speed_m_s;

    std::size_t count = 0;
    std::uint32_t mask = 1U << 2U;
    std::cout << "integer division=" << wrong_time_s << '\n';
    std::cout << "real division=" << time_s << '\n';
    std::cout << "count=" << count << " mask=" << mask << '\n';
    std::cout << std::setprecision(17) << "0.1+0.2=" << 0.1 + 0.2 << '\n';
    std::cout << "double digits=" << std::numeric_limits<double>::digits << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/03_numbers.cpp -o build/ex_03_numbers && \
  ./build/ex_03_numbers
```

**本次运行捕获的标准输出**：

```text
integer division=2
real division=2.5
count=0 mask=4
0.1+0.2=0.30000000000000004
double digits=53
```

其中 `setprecision(17)` 让输出显示更多有效数字，不会提高存储精度。`numeric_limits<double>::digits` 描述浮点尾数精度，示例输出与当前编译平台有关。

## 7.4 转弧度必须显式写出来

```cpp
constexpr double pi = 3.14159265358979323846;
double angle_rad = angle_deg * pi / 180.0;
```

标准三角函数通常以弧度为输入。`sin(90)` 不是“90 度的正弦”。看到状态机的 `8.0 * kPi / 180.0`，应该读成“把 8 度门限换成弧度门限”。

## 7.5 NaN 和无穷大不是正常观测

除一般有限值外，浮点实现还可能提供 NaN 和正负无穷。针对外部数据，应先用 `std::isfinite` 检查。尤其要避免：

```cpp
// 有缺陷的检查：NaN 与 threshold 比较通常不会进入这个分支。
if (error > threshold) {
    // 拒绝。
}
// 此处不能据此推断 error 一定合法。
```

后面第 39 章会完整讲有限值检查、夹紧和数学函数的输入域。

**练习**：把 100 毫秒转换成秒，正确写法是 `100.0 / 1000.0` 或采用 chrono；`100 / 1000` 先得到整数零。

参考：C++17 `[expr.mul]`、`[lex.fcon]`、`[conv.fpint]`、`[numeric.limits]`，见 [R03]。

<a id="ch08"></a>
# 第 8 章：输入、输出、字符串，先从可靠读取一个数开始

## 8.1 输出流与输入流方向相反

`std::cout << value` 把值写出去；`std::cin >> value` 尝试从输入中解析一个值。它不是往变量里随便塞一段字节，而是按变量类型解析。

<a id="ex04"></a>
**完整例程 04：`examples/04_input.cpp`**

```cpp
#include <cmath>
#include <iostream>

int main()
{
    double height_m = 0.0;
    std::cout << "Enter a height in [0.8, 4.0] m:\n";
    if (!(std::cin >> height_m)) {
        std::cerr << "Input is not a number.\n";
        return 1;
    }
    if (!std::isfinite(height_m) || height_m < 0.8 || height_m > 4.0) {
        std::cerr << "Height is outside the exercise range.\n";
        return 2;
    }
    std::cout << "Accepted " << height_m << " m (offline only).\n";
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/04_input.cpp -o build/ex_04_input && \
  printf '1.3\n' | ./build/ex_04_input
```

**本次运行捕获的标准输出**：

```text
Enter a height in [0.8, 4.0] m:
Accepted 1.3 m (offline only).
```

此练习只验证输入，不发送任何飞行命令。示例把 0.8～4.0 作为教学范围，不是为所有飞机规定统一安全高度。

## 8.2 为什么要写 `if (!(std::cin >> height_m))`

括号中的提取操作返回输入流自身；流可以在条件判断中转换成“读取是否仍然成功”的布尔结果。`!` 对结果取反，所以这一句的意思是“读取失败则进入分支”。

如果输入 `hello`，程序不能把它解析成 double，因此返回非零退出码。若只写 `std::cin >> height_m;` 而不检查，后续程序可能继续使用旧值或不合预期的值。

这个最小例子只读取一个数值 token。它**没有严格拒绝数字之后的所有多余字符**，例如 `1.3xyz` 可能先解析出 1.3。第 40 章将用整行读取和完整解析解决这个边界。

## 8.3 `std::string` 与字符

```cpp
char axis = 'X';
std::string mode = "GUIDED";
```

单引号通常表示一个字符字面量；双引号表示字符串字面量。`"X"` 和 `'X'` 不是相同类型。`std::string` 自己管理文本存储，可以赋值和拼接：

```cpp
std::string label = "payload_" + std::to_string(1);
```

不要写 `"payload_" + 1` 并期待得到 `payload_1`；那可能是对字符串字面量退化出的指针做偏移。字符串与字符数组的区别在第 18 章展开。

## 8.4 `getline`、空格和换行

`std::cin >> name` 默认按空白分隔读取；`std::getline(std::cin, name)` 读取一整行。混用时，前一次提取操作留下的换行可能导致下一次 getline 立即读到空行。常见处理是明确消费剩余行，或统一使用“逐行读取再解析”。不要不断加随机的 `cin.get()` 直到现象消失。

## 8.5 `cerr`、`endl`、刷新

错误信息可以写到 `std::cerr`。`std::endl` 会输出换行并刷新流；`'\n'` 只表达换行，不要求每次强制刷新。高频循环通常没有必要每行都用 endl，但也不能把“用了换行”理解成消息立刻可靠写入了磁盘。

**练习**：分别输入 `1.3`、`-1`、`hello`，观察退出码。Linux 终端可以在程序结束后执行 `echo $?` 查看上一个命令的退出状态。

参考：C++17 `[iostreams]`、`[strings]`，见 [R03]。

<a id="ch09"></a>
# 第 9 章：运算符与表达式，把每个条件读成一句话

## 9.1 基本算术和比较

`+ - * /` 表示算术运算；`%` 是整数余数运算，不用于 double。`==` 比较相等，`!=` 比较不相等，`< <= > >=` 比较大小。`=` 是赋值，不能与 `==` 混淆。

```cpp
bool inside = error_m <= 0.20;
```

先比较，再把结果保存到 inside。它保存的是布尔值，不是误差本身。

## 9.2 逻辑运算和短路

`&&` 表示同时满足；`||` 表示至少一个满足；`!` 表示取反。它们产生条件判断，不是连接字符串。

C++ 内置逻辑运算从左向右判断并短路：`a && b` 中 a 为 false 时不再求值 b；`a || b` 中 a 为 true 时不再求值 b。这能保护后面的访问：

```cpp
// pointer 已声明的上下文片段。
if (pointer != nullptr && *pointer > 0.0) {
    // 左侧先确认存在，右侧才读取对象。
}
```

顺序颠倒就失去保护。v11 中 `!active_bucket_.has_value() || ...` 后面的访问，同样依赖短路与合适的条件顺序。

## 9.3 位运算不是逻辑运算

`& | ^ ~ << >>` 可以作用于整数位。`1U << 2U` 把一个无符号 1 左移两位，结果为 4。位掩码可以同时表示多个独立开关；枚举 State 则通常只表示当前处于一个状态。

`a & b` 不具备 `a && b` 的短路保护。不要为了“写少一个字符”替换它们。操作流时的 `<<` 又是另一种重载语义，应结合操作数类型阅读。

## 9.4 三目运算符

```cpp
int pwm = release ? 1500 : 1200;
```

先判断 release，为 true 选 1500，否则选 1200。这是表达式，可以产生一个值。与之等价的清晰初学写法是先定义 pwm，再用 if/else 赋值。

复杂的嵌套三目并不比清晰的 if 更高级。读不顺时，先展开成分支，再确认类型和取值。

## 9.5 自增、复合赋值与求值顺序

`++i` 先增加再产生表达式结果；`i++` 先保留旧结果再增加。单独作为循环更新时，内置整数通常效果相同。`x += step` 表达“把 step 累加到 x”。

不要把多个修改同一变量的副作用塞进一条复杂语句。即便某些 C++17 求值规则更明确，拆开仍更适合任务代码审查。

**练习**：把 `connected && armed && geometry_ok` 写成自然语言，并说明 connected 为 false 时后两项是否求值。答案：飞控连接、已解锁、几何条件同时满足；第一项失败后内置 `&&` 链会短路。

参考：C++17 `[expr.log.and]`、`[expr.log.or]`、`[expr.cond]`、`[expr.shift]`，见 [R03]。

<a id="ch10"></a>
# 第 10 章：`if`、`else` 与提前返回——把错误入口挡在外面

## 10.1 一条分支决定一段代码是否执行

```cpp
if (error_m <= tolerance_m) {
    // 条件成立时执行。
} else {
    // 条件不成立时执行。
}
```

花括号中的代码可以是多条语句。即使只有一条，也建议保留花括号，避免以后加语句时产生误解。`if (...) ;` 中多出来的分号是一条空语句，会让后面的代码块不再受这个条件控制。

## 10.2 `else if` 表示有顺序的筛选

<a id="ex05"></a>
**完整例程 05：`examples/05_branches.cpp`**

```cpp
#include <iostream>

int main()
{
    bool connected = true;
    bool armed = false;
    double error_m = 0.18;
    double speed_m_s = 0.10;
    bool geometry_ok = error_m <= 0.20;
    bool motion_ok = speed_m_s <= 0.15;

    if (!connected) {
        std::cout << "WAIT_CONNECTION\n";
    } else if (!armed) {
        std::cout << "OFFLINE: no release command\n";
    } else if (geometry_ok && motion_ok) {
        std::cout << "candidate only: still need continuous stability\n";
    }
    std::cout << "geometry=" << std::boolalpha << geometry_ok << '\n';
    std::cout << "motion=" << motion_ok << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/05_branches.cpp -o build/ex_05_branches && \
  ./build/ex_05_branches
```

**本次运行捕获的标准输出**：

```text
OFFLINE: no release command
geometry=true
motion=true
```

示例的输出不会进入 armed 分支，因为第二个判断已经选中了“未解锁”。后面的 `else if` 不是独立执行的多个 if。分支顺序就是决策优先级。

在飞行状态机中，失联或人工接管通常应比“继续飞航点”先处理；这属于业务优先级，不是 C++ 自动替你安排的。

## 10.3 保护性提前返回

等学到函数后，常用下面的形式：

```cpp
// 语法片段：假定此代码位于一个返回 bool 的函数内。
if (!connected) {
    return false;
}
if (!have_position) {
    return false;
}
return geometry_ok;
```

比多层嵌套更容易看到“正常路径的前提”。但要注意：`return` 退出**当前函数**，不是整个程序，也不是自动触发返航。要返航必须有明确的状态迁移或命令处理。

## 10.4 一个瞬时条件不是连续稳定条件

`if (error <= 0.2)` 只描述这次执行时的条件。它没有证明过去 0.5 秒都满足，也没有证明图像来自新帧。后面会用状态变量保存“从什么时候开始持续满足”，这就是复杂状态机需要记忆的原因。

**练习**：把示例中的 `armed` 改成 true，预测输出；再把 speed 改成 0.3，增加一个 else 说明为何不满足。不能为了让测试通过而把所有条件改成 true。

参考：C++17 `[stmt.if]`、`[stmt.return]`，见 [R03]。

<a id="ch11"></a>
# 第 11 章：循环不是“程序卡住”，关键在于每次推进了什么

## 11.1 `for` 的执行顺序

```cpp
for (int i = 0; i < 3; ++i) {
    std::cout << i << '\n';
}
```

执行顺序是：初始化 i 一次；判断 `i < 3`；成立则执行循环体；执行 `++i`；再次判断。输出 0、1、2，而不是 1、2、3。循环变量的作用域只在这个 for 及其循环体内。

## 11.2 `while` 和 `do...while`

`while` 先判断条件，可能一次也不执行；`do...while` 先执行循环体，再判断，至少执行一次。无限循环不是必然错误，但必须有明确的退出或调度方式。

```cpp
// 错误思路：条件在循环内没有任何变化。
// while (!ready) { }
```

在单线程回调系统中，这种循环可能把本来负责更新 ready 的回调堵在外面。这不是“等待更耐心”，而是阻止系统取得进展。

## 11.3 `break`、`continue` 和 `return`

`break` 退出最近一层循环或 switch；`continue` 跳过本轮剩余代码，进入下一轮；`return` 退出整个当前函数。嵌套循环中的 break 不会自动跳出所有层。

删除容器元素时，尤其要保证每次要么更新迭代器、要么删除后取得下一个迭代器，否则会死循环。第 35 章会针对 v11 的 pending 命令列表做实验。

<a id="ex06"></a>
**完整例程 06：`examples/06_loops.cpp`**

```cpp
#include <iostream>

int main()
{
    const int lane_count = 7;
    const double half_width_m = 3.1;
    for (int lane = 0; lane < lane_count; ++lane) {
        double fraction = static_cast<double>(lane) / (lane_count - 1);
        double y_m = -half_width_m + 2.0 * half_width_m * fraction;
        std::cout << "lane=" << lane << " y=" << y_m;
        if (lane % 2 == 0) {
            std::cout << " forward\n";
        } else {
            std::cout << " backward\n";
        }
    }
    int checks = 0;
    while (checks < 3) {
        ++checks;
    }
    std::cout << "checks=" << checks << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/06_loops.cpp -o build/ex_06_loops && \
  ./build/ex_06_loops
```

**本次运行捕获的标准输出**：

```text
lane=0 y=-3.1 forward
lane=1 y=-2.06667 backward
lane=2 y=-1.03333 forward
lane=3 y=0 backward
lane=4 y=1.03333 forward
lane=5 y=2.06667 backward
lane=6 y=3.1 forward
checks=3
```

## 11.4 用变量表检查蛇形逻辑

这里 lane 从 0 到 6。`lane % 2 == 0` 判断偶数，分别给相邻搜索线相反方向。分数 `lane / (lane_count - 1)` 必须在除法之前转为浮点；否则多数分数都是零。

如果 `lane_count` 可能等于 1，就不能直接除以 `lane_count - 1`。v11 用三目运算在单线时选择 0.5。不要只测试“正常七条线”。

**练习**：把 lane_count 改成 1，先判断哪里会出问题，再使用 if 或三目处理。此处是纯输出练习，不代表生成的搜索路线已经适合真机。

参考：C++17 `[stmt.iter]`、`[stmt.break]`、`[stmt.cont]`，见 [R03]。

<a id="ch12"></a>
# 第 12 章：第一次阶段实践——写一个离线任务条件检查器

## 12.1 现在只组合已经学过的内容

目标不是写飞机控制，而是根据输入事实给出一个结果。程序应拥有这些变量：连接状态、是否有位置、目标高度、当前误差、当前速度。它输出“等待连接”“等待位置”“参数无效”“尚未对准”或“瞬时条件满足”。

只需要变量、算术、比较、if/else 和输出。不要提前加入类、智能指针或 Lambda。这样你可以判断问题究竟是条件写错，还是框架使用错误。

## 12.2 先写决策表，再写代码

| connected | have_position | height 合法 | error/speed 合法 | 结果 |
|---|---|---|---|---|
| false | 任意 | 任意 | 任意 | 等待连接 |
| true | false | 任意 | 任意 | 等待位置 |
| true | true | false | 任意 | 拒绝配置 |
| true | true | true | false | 尚未满足 |
| true | true | true | true | 仅瞬时条件满足 |

用第 5、10 章的完整程序作为起点修改，逐行执行这张表。特别要测试 `error == threshold`；如果要求“包含边界”，代码就应使用 `<=` 而不是 `<`。

## 12.3 不要引入一个看不见的“自动投放”

在这个练习里，条件满足时只打印文本。不要把名为 `release()` 的函数偷偷实现成串口输出。教学代码应让其影响范围清楚可见。

## 12.4 自测与答案要点

题一：把连接状态改成 false，其余保持满足，为什么不能输出“满足投放”？答案：组合条件必须同时成立，且决策表优先处理连接。

题二：把目标高度设成 NaN，为什么只检查“大于 4”不够？答案：NaN 比较可能使这个拒绝条件不成立，应先检查有限性。

题三：连续运行程序十次都打印满足，是否证明飞机稳定 0.5 秒？答案：不证明；这些运行甚至不共享同一个状态对象，更没有连续观测时间证据。

完成本章后，你应当能读懂普通业务 if，而不是只能认出关键词。下一篇开始把重复逻辑收进函数。


# 第三篇：函数、引用、指针与数据结构

<a id="ch13"></a>
# 第 13 章：函数从零——谁调用谁，值从哪里来，执行到哪里回去

## 13.1 为什么要把代码收进函数

假设你在搜索、对准和返航阶段都要计算两点之间的平面距离。复制三遍公式，会有三处需要修正；写成函数后，调用者只表达“我要距离”，函数内部负责怎样计算。

先不用结构体，直接传四个数字：

<a id="ex07"></a>
**完整例程 07：`examples/07_functions.cpp`**

```cpp
#include <cmath>
#include <iostream>

double distance_xy(double ax, double ay, double bx, double by)
{
    double dx = ax - bx;
    double dy = ay - by;
    return std::hypot(dx, dy);
}

bool inside_tolerance(double error, double tolerance)
{
    return std::isfinite(error) && std::isfinite(tolerance) &&
           error >= 0.0 && tolerance >= 0.0 && error <= tolerance;
}

int main()
{
    double error = distance_xy(0.0, 0.0, 3.0, 4.0);
    bool ready = inside_tolerance(error, 0.2);
    std::cout << "distance=" << error << '\n';
    std::cout << std::boolalpha << "ready=" << ready << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/07_functions.cpp -o build/ex_07_functions && \
  ./build/ex_07_functions
```

**本次运行捕获的标准输出**：

```text
distance=5
ready=false
```

这个程序引入两个函数：`distance_xy` 返回距离；`inside_tolerance` 判断距离能否接受。两者都是教学函数，不发送任何消息。

## 13.2 把一个函数拆成五块

```cpp
double distance_xy(double ax, double ay, double bx, double by)
{
    double dx = ax - bx;
    double dy = ay - by;
    return std::hypot(dx, dy);
}
```

| 位置 | 名称 | 意义 |
|---|---|---|
| 第一个 `double` | 返回类型 | 调用完成后交回一个浮点数 |
| `distance_xy` | 函数名 | 调用者使用的名字 |
| 括号里的四组声明 | 形参列表 | 本次调用接收的四份输入 |
| `{ ... }` | 函数体 | 实际执行的语句 |
| `return ...` | 返回语句 | 计算返回值并结束本次调用 |

调用 `distance_xy(0.0, 0.0, 3.0, 4.0)` 时，括号内的四个值是**实参**。函数定义里的 ax、ay、bx、by 是**形参**。实参与形参按位置对应，不按你脑子里的名字对应。

## 13.3 逐步模拟一次调用

`main` 执行到调用表达式时，准备四个参数值；函数内部得到 ax=0、ay=0、bx=3、by=4；局部变量 dx=-3、dy=-4；`hypot` 得到 5；函数返回 5；`main` 把结果初始化给 error；然后继续下一句。

```text
main 的 error 尚未完成初始化
  → distance_xy 的参数与 dx/dy 存在
  → 返回数值 5
main 的 error = 5
  → distance_xy 的普通局部变量生命周期结束
```

这就是调用与返回的基本模型。实现通常使用调用栈，但优化器可以内联函数，因此“每次调用必定在机器栈上形成同样结构”不是语言保证。你需要理解的是局部对象的独立性与生命周期。

## 13.4 参数复制与局部修改

`double ax` 是按值传递。函数中的 ax 是本次调用自己的参数对象。给 ax 重新赋值，不会改变调用者原来的变量。小型数值类型按值传递通常最清晰。

两个参数都叫 `value` 的不同函数，不会因此共享同一个变量。名字的可见性和对象是否同一个，是两件事。

## 13.5 `void` 与提前返回

```cpp
// 这是一个完整的函数定义，但不是独立可执行程序。
void print_waiting()
{
    std::cout << "waiting\n";
    return;
}
```

`void` 表示没有返回值。可以写无表达式的 `return;` 提前离开。非 void 函数则要保证所有正常返回路径都有合适的值，不能只在一个 if 分支返回。

**练习**：写一个 `double travel_time(double distance, double speed)`。先写出输入契约：有限值、distance 非负、speed 正值；再决定非法输入如何报告。此时可以先返回一个 bool 并通过单独打印报告，后面再学 optional/异常，不要让除零变成默认错误处理。

参考：C++17 `[expr.call]`、`[dcl.fct]`、`[stmt.return]`，见 [R03]。

<a id="ch14"></a>
# 第 14 章：函数声明、定义、重载、默认参数与接口设计

## 14.1 为什么“函数写在下面”有时编译不过

编译器处理调用时，需要已经知道函数的声明。可以把定义放在调用前，也可以先声明：

```cpp
// 语法片段：前面已有必要头文件。
double square(double value); // 声明：告诉编译器接口。

int main()
{
    std::cout << square(3.0) << '\n';
}

double square(double value) // 定义：给出实现。
{
    return value * value;
}
```

声明只描述“有这样一个函数”；定义还提供函数体。只有声明而没有被链接进程序的定义，会出现链接错误。把函数名拼错成另一个名字，也不会因为“看起来差不多”而连接成功。

## 14.2 返回类型不是重载的唯一依据

可以按参数不同重载：

```cpp
int twice(int value) { return value * 2; }
double twice(double value) { return value * 2.0; }
```

`twice(2)` 与 `twice(2.0)` 选择不同版本。不能只靠返回类型重载 `int read()` 和 `double read()`，因为单看 `read()` 的调用无法按你希望的方式区分。

重载不是越多越好。高度米与厘米都使用 double 时，函数重载并不能区分单位；应该用明确命名，或在更成熟的代码中引入单位类型。

## 14.3 默认参数在哪里生效

```cpp
bool acceptable(double error, double tolerance = 0.20);
```

`acceptable(0.1)` 相当于使用默认 tolerance；`acceptable(0.1, 0.12)` 则显式覆盖。默认参数一般放在头文件中的声明处，不要在同一作用域下的定义处重复给出。

默认参数不是全局可变配置。修改源码中的默认参数，需要重新编译相应调用者。它与 ROS 运行参数的读取方式不同。

## 14.4 输入、输出与副作用

一个函数可以做纯计算，也可以修改对象、写文件、发消息。接口应该让调用者知道它属于哪一种。

`distance_xy(...)` 适合纯计算；`update_track(...)` 暗示修改轨迹；`send_servo(...)` 暗示对外动作。返回 `true` 到底意味着“请求已发送”还是“动作已完成”，必须明确。C++ 的 bool 本身无法替你区分这些业务语义。

对于复杂任务，尽量把“计算期望值”和“执行外部命令”拆成不同函数。这让计算部分能够离线测试。

## 14.5 递归、内联与其他容易误解的术语

递归是函数直接或间接调用自身。必须有终止条件，否则调用链可能持续增长。当前 v11 的状态推进主要依靠定时 tick，不应改成“SEARCH 里递归调用 SEARCH 直到找到桶”。

`inline` 不保证编译器把代码展开；它的重要语言作用还涉及多翻译单元中的定义规则。`constexpr` 函数不保证每次调用都在编译期执行。`noexcept` 不会帮你捕获异常。这些都不是“加上就更快、更安全”的装饰词。

**练习**：为什么 `send_request()` 返回 true 后不能马上 `payload_done = true`？答案：接口可能只说明请求被提交，异步响应和真实动作仍未确认。这个问题会在第 44、56 章回到实际代码。

参考：C++17 `[over.load]`、`[dcl.fct.default]`、`[dcl.inline]`，见 [R03]。

<a id="ch15"></a>
# 第 15 章：引用与参数传递——把 `const T&` 讲透

## 15.1 引用是另一个名字，不是新的独立数值

```cpp
int count = 2;
int& alias = count;
alias = 3;
```

此时 count 也变成 3。`int&` 表示对 int 对象的左值引用；alias 绑定到 count，不是复制一个值等于 2 的新整数。

```text
count ─┐
       ├── 同一个 int 对象，当前值 3
alias ─┘
```

正常使用的引用必须在初始化时绑定到有效对象。引用一旦绑定，不能像指针那样重新指向另一个对象。执行 `alias = other` 是把 other 的值赋给 count，不是改绑。

<a id="ex08"></a>
**完整例程 08：`examples/08_references.cpp`**

```cpp
#include <iostream>

void change_copy(int count)
{
    count = 99;
    std::cout << "inside copy=" << count << '\n';
}

void change_original(int& count)
{
    count = 3;
}

int read_only(const int& count)
{
    return count;
}

int main()
{
    int count = 2;
    change_copy(count);
    std::cout << "after copy=" << count << '\n';
    change_original(count);
    std::cout << "after reference=" << read_only(count) << '\n';
    int& alias = count;
    alias = 4;
    std::cout << "after alias=" << count << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/08_references.cpp -o build/ex_08_references && \
  ./build/ex_08_references
```

**本次运行捕获的标准输出**：

```text
inside copy=99
after copy=2
after reference=3
after alias=4
```

## 15.2 三种函数参数的区别

| 参数形式 | 创建独立的参数值吗 | 能否通过参数修改调用者对象 | 适用情况 |
|---|---|---|---|
| `T value` | 是，按值传递 | 通常不能直接修改原对象；指针成员另论 | 小型数值、需要自己拥有一份值 |
| `T& value` | 不复制被引用对象 | 能 | 明确的原地修改 |
| `const T& value` | 不复制被引用对象 | 不能通过这个引用直接修改对象 | 借用读取较大的对象 |

对 `std::string`、普通容器等按只读引用传入，通常能避免整份数据复制；对 int、double 等小标量，按值传递更自然。不是所有参数都应“为了效率”统一改成引用。

## 15.3 `const T&` 中的每个部分

`T` 指被引用对象的类型；`&` 表示引用；`const` 限制经该引用进行修改。比如：

```cpp
void print_name(const std::string& name)
{
    std::cout << name << '\n';
    // name = "other"; // 编译错误：不允许通过该 const 引用修改。
}
```

const 不会给对象加物理锁，也不会阻止其他合法的非 const 别名修改同一个对象。它是类型系统中的访问约束，不是数据新鲜度或线程同步保证。

## 15.4 临时对象和引用返回的陷阱

const 引用可以在适当情境下绑定到临时对象，例如向函数传入 `std::string("demo")`。临时对象的生存期规则有严格边界，不能据此把引用存下来以后一直用。

```cpp
// 错误示例：不要运行。
const std::string& bad_name()
{
    std::string local = "temporary";
    return local; // local 在函数结束时销毁，返回的是悬空引用。
}
```

正确方式通常是按值返回 `std::string`。现代 C++ 可以执行复制消除或移动，不要为了想象中的性能优化返回局部引用。

返回某个容器元素的引用也有前提：容器和元素必须继续存在，且后续操作不能使引用失效。第 35 章会进一步展开。

## 15.5 回到 v11：一行函数声明的阅读顺序

```cpp
double distance_xy(const Point3& a, const Point3& b)
```

先读“函数名 distance_xy”；再读“有两个 Point3 输入”；再读“借用读取，不复制，不允许经 a/b 修改”；最后读“返回 double”。Point3 的具体字段到第 19 章再定义，但此时引用机制应该已经清楚。

**练习**：把例程中的 `change_original(int& count)` 改成 `change_original(int count)`，预测 main 中 count 的变化。答案：原对象不再被该参数的赋值修改。

参考：[R04]、[R05]；C++17 `[dcl.ref]`、`[class.temporary]`，见 [R03]。

<a id="ch16"></a>
# 第 16 章：指针、地址、解引用与空值——不跳过任何一层

## 16.1 指针变量保存“指向哪里”的信息

```cpp
double height = 1.3;
double* pointer = &height;
```

声明中的 `*` 表示 pointer 的类型是“指向 double 的指针”。表达式中的 `&height` 表示取得 height 的地址。`*pointer` 则表示访问它所指向的 double 对象。

```text
pointer 对象：保存 height 的地址 ──→ height 对象：保存 1.3
```

指针自身也是对象，它有自己的类型、值、作用域和生命周期。不要把“指针的值”和“它指向的值”混为一谈。

<a id="ex09"></a>
**完整例程 09：`examples/09_pointers.cpp`**

```cpp
#include <iostream>

bool set_if_present(double* value, double replacement)
{
    if (value == nullptr) {
        return false;
    }
    *value = replacement;
    return true;
}

int main()
{
    double height = 1.3;
    double* pointer = &height;
    double other = 2.2;
    *pointer = 1.7;
    pointer = &other;
    *pointer = 2.5;
    std::cout << "height=" << height << " other=" << other << '\n';
    std::cout << std::boolalpha;
    std::cout << "null accepted=" << set_if_present(nullptr, 4.0) << '\n';
    std::cout << "value accepted=" << set_if_present(&height, 4.0) << '\n';
    std::cout << "height=" << height << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/09_pointers.cpp -o build/ex_09_pointers && \
  ./build/ex_09_pointers
```

**本次运行捕获的标准输出**：

```text
height=1.7 other=2.5
null accepted=false
value accepted=true
height=4
```

第一次 `*pointer = 1.7` 改变 height；`pointer = &other` 改变指向；后面的 `*pointer = 2.5` 改变 other。读这一段时必须跟踪两件事：pointer 当前指向谁，以及谁的值被修改。

## 16.2 `nullptr` 表示不指向对象

`double* p = nullptr;` 是明确的空指针状态。可以比较是否为空，但不能解引用。检查 `p != nullptr` 只证明它不是空指针，不能证明它没有悬空；对象已销毁时，指针可能仍是非空。

正常工程中，裸指针常用于短期借用一个对象；拥有动态对象时优先使用后面讲的智能指针。不要看到星号就默认“必须手工 delete”。

## 16.3 指针和 `const` 的四种组合

| 声明 | 可改变指针指向吗 | 可经它修改对象吗 |
|---|---|---|
| `T* p` | 可以 | 可以，前提是对象本身可修改 |
| `const T* p` | 可以 | 不可以 |
| `T* const p` | 不可以 | 可以 |
| `const T* const p` | 不可以 | 不可以 |

从变量名向外看会更清楚。`const T*` 与 `T const*` 等价。顶层 const 约束指针变量，底层 const 约束被指向的类型。

这个区别会直接影响 ROS 消息参数：`const Msg::SharedPtr msg` 与 `Msg::ConstSharedPtr msg` 的 const 位置不同，第 26 章会专门对照。

## 16.4 `.` 与 `->`

拥有对象或引用时用点访问成员：`point.x`。拥有指针时常用箭头：`pointer->x`，对普通指针可理解为 `(*pointer).x`。括号不可随意省略，因为运算符优先级不同。

以后 `home_->x` 中 home_ 可能是 optional，并非裸指针；它通过重载提供类似的访问写法。**相同符号不保证相同所有权。**

## 16.5 指针算术和数组边界

对同一数组范围内的指针可以进行受约束的加减，步长按元素大小计算；它不是任意整数地址相加。一般任务代码不需要手写跨缓冲区的指针算术。

不要用 `reinterpret_cast` 来“修复类型报错”；不要把一个对象的地址强行解释成完全不相干的消息类型。类型错误通常意味着接口需要重新理解，而不是转换还不够强制。

**练习**：非空指针一定安全吗？答案：不一定，可能指向已销毁对象、失效的容器元素，或错误类型的内存。

参考：C++17 `[conv.ptr]`、`[expr.unary.op]`、`[expr.add]`、`[basic.life]`，见 [R03]、[R06]。

<a id="ch17"></a>
# 第 17 章：作用域、生命周期、`const`、`constexpr`、`static` 与命名空间

## 17.1 名字看得见，不等于对象一定活着

作用域描述某个名字在什么位置可见；生命周期描述对象什么时候存在。局部变量的名字通常在代码块内可见，对象也通常在离开块时销毁，但引用、指针、静态对象会让两者不再一一对应。

```cpp
int count = 1;
{
    int inner_count = 2;
    std::cout << inner_count;
}
// inner_count 的名字在这里不可见。
```

不建议内层再定义同名 count 来遮蔽外层 count。编译器可能接受，但人容易改错变量；配套构建打开 `-Wshadow` 帮助发现这种情况。

## 17.2 `const` 与 `constexpr`

`const` 表示通过相应声明不允许修改。它的初值可以运行时得到，例如 `const double measured = read_sensor();`。

`constexpr` 变量需要能由常量表达式初始化。对于函数，constexpr 表明它在满足条件时可以用于常量表达式，不表示所有调用都强制发生在编译期。

```cpp
constexpr double square(double value) { return value * value; }
constexpr double fixed = square(3.0); // 常量表达式用途。
// double measured = ...;
// double runtime_result = square(measured); // 仍可以运行时求值。
```

本书采用 C++17，因此不使用 C++20 的 `consteval`、`constinit`、`std::numbers::pi` 来替代这些例子。

## 17.3 `static` 不是单一意思

函数中的局部 static 保存跨调用状态：

<a id="ex10"></a>
**完整例程 10：`examples/10_scope.cpp`**

```cpp
#include <iostream>

namespace units {
constexpr double pi = 3.14159265358979323846;
constexpr double to_radians(double degrees)
{
    return degrees * pi / 180.0;
}
}

int next_demo_id()
{
    static int counter = 0;
    return ++counter;
}

int main()
{
    constexpr double half_turn = units::to_radians(180.0);
    std::cout << "half_turn=" << half_turn << '\n';
    std::cout << "id=" << next_demo_id() << '\n';
    std::cout << "id=" << next_demo_id() << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/10_scope.cpp -o build/ex_10_scope && \
  ./build/ex_10_scope
```

**本次运行捕获的标准输出**：

```text
half_turn=3.14159
id=1
id=2
```

counter 只初始化一次，之后每次调用继续使用它；它不是每次进入函数都重置为零的普通局部变量。初始化机制与后续并发修改是两件事：局部 static 的初始化有相应线程安全保证，但 `++counter` 不会因此自动变成线程安全操作。

文件作用域的 static 可以限制名字的链接范围；类内 static 表示属于类而不是每个实例一份。后一种用法到第 21 章再看。不要把这三种解释混成“所有 static 都不会销毁”。

## 17.4 命名空间解决同名组织问题

例程中的 `units::pi` 与 `units::to_radians` 放在名为 units 的命名空间。`::` 指定从哪个作用域找名字。

原 v11 文件开头使用 `namespace { ... }`，这是匿名命名空间，通常用于把辅助名字限制在本翻译单元内。它与类的 private 不同：前者是文件组织和链接层面的范围，后者是成员访问控制。

`using namespace std;` 会把大量名字引入当前查找范围，初学教材和公共头文件中不推荐如此写。选择性使用 `using Clock = ...;` 是给一个类型取别名，副作用更小。

## 17.5 成员状态为什么不同于局部变量

稳定计时起点必须在下一次 tick 仍然存在，所以它适合成为对象成员。一次距离计算中的 dx、dy 只在本次函数里有意义，适合局部变量。用 static 强行保存所有任务状态，会导致多个实例共享状态，通常不是所需设计。

**练习**：为什么不能在每次 tick 里重新定义 `start = now()` 后检查等待是否超过 0.5 秒？答案：计时起点不断重置，几乎永远达不到目标。

参考：C++17 `[basic.scope]`、`[basic.stc]`、`[dcl.constexpr]`、`[namespace.def]`，见 [R03]。

<a id="ch18"></a>
# 第 18 章：数组、字符串、`char** argv` 与字符串视图

## 18.1 原生数组：固定数量、同一种类型

```cpp
double heights[3] = {2.2, 1.7, 1.3};
```

有效下标是 0、1、2。`heights[3]` 已经越界。`[]` 不会自动检查范围。

原生数组不能像普通变量那样整体赋值，也常在表达式和函数参数中退化成指向首元素的指针。函数参数写 `double values[]` 时，通常不能在函数里靠 `sizeof(values)` 求原数组长度，因为此处参数已经按指针处理。

因此新代码经常使用 `std::array<T,N>` 或 `std::vector<T>`；它们在第 32 章集中讲，不要求现在马上掌握模板实现。

## 18.2 C 字符串和 `std::string`

字符串字面量 `"ABC"` 包括 A、B、C 和末尾的零字符，共四个 char 元素。C 字符串通过结尾 `\0` 标识结束；它与字符 `'0'` 不同。

`std::string` 管理长度和存储，可以包含内部零字符；因此不能把所有 string 操作都按“遇零结束”理解。`text.c_str()` 提供面向 C 接口的字符指针，指针有效性依赖 text 继续存活且未执行使其失效的操作。

不要缓存 `make_string().c_str()` 供以后使用；临时 string 被销毁后，指针可能悬空。

## 18.3 `argc` 与 `argv` 逐层拆解

```cpp
int main(int argc, char** argv)
```

argc 是参数数量；argv 是指向字符指针的指针，可以按数组方式访问每个参数字符串。`argv[1]` 本身是一个 char 指针；`argv[1][0]` 才是该参数的第一个字符。先检查 argc，再访问相应位置。

执行 `./program demo` 时，demo 是运行参数，不是 C++ 源码里的变量名。ROS 的 `rclcpp::init(argc, argv)` 会利用其中相关参数，但基础形式仍是标准的 main 参数。

## 18.4 `string_view` 不拥有字符

```cpp
std::string name = "CUADC";
std::string_view view = name;
```

view 只是借用 name 的字符区域和长度信息，不负责让 name 活得更久。name 销毁或其存储失效后，view 也不能继续读取。把值类型换成 view 可以减少复制，但会增加生命周期要求。

<a id="ex11"></a>
**完整例程 11：`examples/11_arrays_strings.cpp`**

```cpp
#include <cstddef>
#include <iostream>
#include <string>
#include <string_view>

int main(int argc, char** argv)
{
    double heights[3] = {2.2, 1.7, 1.3};
    for (std::size_t index = 0; index < 3; ++index) {
        std::cout << "height[" << index << "]=" << heights[index] << '\n';
    }
    std::string name = "cuadc";
    std::string_view view = name; // name 在 view 的整个使用期内保持有效且未改变。
    std::cout << "name=" << view << " length=" << view.size() << '\n';
    std::cout << "arg_count=" << argc - 1 << '\n';
    if (argc > 1) {
        std::cout << "first_arg=" << argv[1] << '\n';
    }
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/11_arrays_strings.cpp -o build/ex_11_arrays_strings && \
  ./build/ex_11_arrays_strings demo
```

**本次运行捕获的标准输出**：

```text
height[0]=2.2
height[1]=1.7
height[2]=1.3
name=cuadc length=5
arg_count=1
first_arg=demo
```

示例中 name 在 view 的所有使用期间都存在，也没有改变存储，所以借用成立。不能把 `std::string_view view = std::string("CUADC");` 当作等价安全写法。

## 18.5 中文字符串长度

在 UTF-8 环境下，一个中文字符常由多个字节编码，`std::string::size()` 返回 char 元素数量，不能直接解释成用户看见的字符数。任务日志可以包含中文，但字符串切片、长度限制、协议字段长度要分清字节与字符。

**练习**：解释 `const char* state_name(...)` 为什么可以返回字符串字面量，而不能返回局部 string 的 c_str。答案：字符串字面量的存储期覆盖程序运行；局部 string 在返回后销毁。

参考：C++17 `[dcl.array]`、`[conv.array]`、`[lex.string]`、`[string.view]`，见 [R03]。

<a id="ch19"></a>
# 第 19 章：`struct` 与 `enum class`——第一次建立任务的数据语言

## 19.1 用结构体把相关字段放在一起

之前的距离函数接收四个 double。三个坐标总是一起出现时，可以给这一组数据一个名字：

```cpp
struct Point3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};
```

这是类型定义，还没有创建某个具体点。`Point3 position{};` 才定义一个对象；`position.x` 访问其成员。结尾的分号是类型定义语法的一部分。

这个定义中的成员默认值能使 `Point3 p;` 具有确定的三个零值。但零向量是否代表“有效观测”，仍由业务决定。

## 19.2 聚合初始化与按值复制

`Point3 target{1.0,2.0,3.0};` 按成员顺序初始化。`Point3 copy = target;` 对当前只含三个 double 的结构体创建独立副本。修改 copy.x 不会改变 target.x。

如果结构体里装的是指针，复制的是指针值，不会自动复制被指向的整块数据。复制语义由成员类型共同决定，不能把“struct 复制”统一称为深复制。

<a id="ex12"></a>
**完整例程 12：`examples/12_struct_enum.cpp`**

```cpp
#include <cmath>
#include <iostream>

struct Point3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

enum class State { WAIT, SEARCH, DONE };

const char* state_name(State state)
{
    switch (state) {
    case State::WAIT: return "WAIT";
    case State::SEARCH: return "SEARCH";
    case State::DONE: return "DONE";
    }
    return "UNKNOWN";
}

double distance_xy(const Point3& a, const Point3& b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

int main()
{
    Point3 origin{};
    Point3 target{3.0, 4.0, 1.3};
    State state = State::SEARCH;
    std::cout << state_name(state) << " distance=" << distance_xy(origin, target) << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/12_struct_enum.cpp -o build/ex_12_struct_enum && \
  ./build/ex_12_struct_enum
```

**本次运行捕获的标准输出**：

```text
SEARCH distance=5
```

现在 `distance_xy(const Point3&, const Point3&)` 的每一部分都有先修知识：类型、对象、成员、函数、const 引用、返回值。

## 19.3 枚举表达有限选择

```cpp
enum class State { WAIT, SEARCH, DONE };
```

State 是类型，`State::SEARCH` 是这个类型的一个枚举值。它不是字符串 `"SEARCH"`，也不是能够随便与其他整数混用的变量。

写 `State state = State::WAIT;` 后，state 保存当前选中的状态。枚举列出可能状态，但**不会自动产生状态迁移规则**；何时从 WAIT 进入 SEARCH 仍由代码定义。

## 19.4 `switch` 的执行语义

switch 根据一个离散值选择对应 case。若 case 执行后既不 break 也不 return，可能继续执行后面的 case，称为贯穿。确实需要贯穿时，C++17 可使用 `[[fallthrough]];` 明确意图；状态机通常应避免意外贯穿。

示例的 `state_name` 每个 case 直接 return，所以不需要再 break。打印状态时用这个显式转换函数，比在日志里只输出一个 7 更好理解。

## 19.5 枚举类不是验证器

显式 `static_cast<State>(999)` 并不会自动验证这个数是否对应你列出的任务状态。外部协议转枚举之前，应检查合法取值。强类型有帮助，但不替代输入检查。

**练习**：增加 `ALIGN` 状态，并修改 state_name。再思考：只增加枚举值，程序就会自动执行对准吗？答案：不会，必须增加处理逻辑与进入条件。

参考：C++17 `[class]`、`[dcl.enum]`、`[stmt.switch]`，见 [R03]。

<a id="ch20"></a>
# 第 20 章：第二次阶段实践——离线航点检查器

<a id="ex13"></a>
**完整例程 13：`examples/13_waypoints.cpp`**

```cpp
#include <cmath>
#include <iostream>

struct Point3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

bool valid_waypoint(const Point3& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::isfinite(point.z) && point.z >= 0.8 && point.z <= 4.0;
}

int main()
{
    Point3 route[3] = {{30.0, -3.0, 2.2}, {34.0, -3.0, 2.2}, {34.0, 0.0, -1.0}};
    int accepted = 0;
    for (const Point3& point : route) {
        if (!valid_waypoint(point)) {
            std::cout << "rejected z=" << point.z << '\n';
            continue;
        }
        ++accepted;
    }
    std::cout << "accepted=" << accepted << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/13_waypoints.cpp -o build/ex_13_waypoints && \
  ./build/ex_13_waypoints
```

**本次运行捕获的标准输出**：

```text
rejected z=-1
accepted=2
```

## 20.1 为什么这是一次真正的组合练习

`Point3` 把一组输入组织起来；`valid_waypoint` 负责判断，不输出也不改变参数；main 负责遍历和报告。它已经具备“数据模型、业务函数、程序入口”的分工，但还没有用类。

`for (const Point3& point : route)` 是范围 for：依次访问数组元素，point 是当前元素的只读引用。冒号在这里分隔“每个元素的声明”和“被遍历范围”，不是继承，也不是构造函数初始化列表。

## 20.2 先用已知坏数据验证拒绝路径

第三个点高度为 -1，应该被拒绝。验证器不仅要证明正常输入能通过，还要证明不合法输入不会被悄悄变成正常任务。

将第三点改成合法高度后，接受计数应从 2 变成 3。再把某个坐标改为无穷或 NaN，应拒绝。这个练习可以通过替换常量完成，不需要真实飞控。

## 20.3 拆清三种“成功”

编译成功说明语法和类型检查通过；程序输出 accepted=2 说明当前输入与判断逻辑得到这个结果；真机到达航点则是另外的物理事件。不要从前两者跳到第三者。

## 20.4 本篇验收

能够解释实参/形参、声明/定义、值/引用/指针、作用域/生命周期、const/constexpr/static，以及 struct 对象与 enum 状态。能够读懂 `const Point3& point`，并说明这个引用不拥有一个新的独立点。

下一篇引入类时，只是在这些已有概念上把状态和操作组织在一起，不是换一种全新的语言。


# 第四篇：对象、类、资源与所有权

<a id="ch21"></a>
# 第 21 章：`class`、对象、成员函数与 `this`

## 21.1 为什么普通函数之外还需要类

想象一个“已完成投放数量”变量。若任何函数都能任意把它改成 100，就很难保证最多两瓶。类可以把数据和维护该数据规则的操作放在一起，通过接口限制修改入口。

<a id="ex14"></a>
**完整例程 14：`examples/14_class.cpp`**

```cpp
#include <iostream>

class PayloadCounter {
public:
    bool mark_one_done()
    {
        if (completed_ >= capacity_) {
            return false;
        }
        ++completed_;
        return true;
    }

    int completed() const
    {
        return completed_;
    }

private:
    int capacity_ = 2;
    int completed_ = 0;
};

int main()
{
    PayloadCounter first;
    PayloadCounter second;
    first.mark_one_done();
    std::cout << "first=" << first.completed() << '\n';
    std::cout << "second=" << second.completed() << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/14_class.cpp -o build/ex_14_class && \
  ./build/ex_14_class
```

**本次运行捕获的标准输出**：

```text
first=1
second=0
```

这里 first 和 second 是同一类型的两个对象，但各自保存自己的 completed_。调用 first.mark_one_done() 不会改变 second。这是学习 ROS Node 成员变量之前必须建立的基本认识。

## 21.2 类型、实例、字段与方法

`PayloadCounter` 是类类型；first 是对象；capacity_ 和 completed_ 是数据成员；mark_one_done 和 completed 是成员函数。它们不是四个可以随意互换的名字。

`first.completed()` 带括号，表示调用函数；`completed_` 不带括号，表示保存状态的数据成员。尾部下划线只是命名约定。

类定义中的 `public:` 和 `private:` 控制访问。外部可以调用 public 方法，但不能直接访问 private 字段。访问控制在编译层面起作用，不是加密，也不防止错误的指针操作。

## 21.3 成员函数如何知道正在操作哪个对象

普通非静态成员函数具有隐含的当前对象指针 `this`。在 mark_one_done 内写 `++completed_`，可以理解为通过当前对象访问这个成员。

```cpp
// 类内部的语法片段。
int completed() const
{
    return this->completed_;
}
```

调用 `first.completed()` 时，this 指向 first；调用 `second.completed()` 时，this 指向 second。`this` 不是一个全局“当前飞机”，更不是自动拥有对象生命周期的智能指针。

## 21.4 函数末尾的 `const`

`int completed() const` 承诺不通过当前对象的普通非 mutable 数据成员执行修改。它允许只读对象调用该方法，并帮助编译器拦截误改。

```cpp
const PayloadCounter fixed;
std::cout << fixed.completed();
// fixed.mark_one_done(); // 不允许：这是可能修改状态的非 const 方法。
```

const 成员函数仍可能改变外部全局状态、写日志，或通过某些间接成员修改别的对象。因此它不等同于“数学纯函数”。`mutable` 可以让指定成员在 const 方法中改变，常用于缓存或锁，但不要拿它绕过所有约束。

## 21.5 `struct` 与 `class` 的本质关系

C++ 中 struct 也可以有构造、成员函数、继承和访问控制。主要默认差异是：struct 默认 public，class 默认 private；默认继承访问也不同。不是“struct 只能放数据、class 才能放函数”的语言限制。

作为风格，单纯承载几何字段的 Point3 用 struct；维护任务不变量的 Mission 用 class，通常更容易读。

## 21.6 静态成员函数和静态成员变量

静态成员函数不依赖一个具体实例，没有隐含 this；因此只靠输入参数的数学工具可以声明为 static。静态数据成员通常由同一类所有实例共享，不能误当作每个飞机各自一份状态。

C++17 支持 `inline static` 数据成员在类中定义，但初学任务代码不需要用它保存运行状态。跨实例共享的任务索引往往是一种设计错误。

**练习**：为什么 `State state_` 适合普通成员，而不宜随手写成 static？答案：每个任务对象应独立保存当前状态；static 会让多个对象共享同一份数据。

参考：C++17 `[class.access]`、`[class.this]`、`[class.static]`、`[class.mfct.non-static]`，见 [R03]。

<a id="ch22"></a>
# 第 22 章：构造函数与初始化列表——冒号后面到底发生什么

## 22.1 构造函数不是普通的“初始化方法”

构造函数在对象建立时参与初始化，名称与类名相同，没有返回类型。它不能写成 `void Mission()`。成员已经存在之后再给它们赋值，与在初始化列表中构造它们，不是同一个过程。

```cpp
class Counter {
public:
    explicit Counter(int capacity) : capacity_(capacity) {}
private:
    int capacity_;
};
```

`Counter counter(2);` 创建对象并把 2 交给构造函数。这里 capacity 是形参，capacity_ 是成员。冒号引出成员初始化列表，`capacity_(capacity)` 用形参初始化成员。

## 22.2 为什么不全部在构造函数体里赋值

类类型成员若先默认构造再赋值，可能多做工作。const 成员和引用成员更必须在初始化阶段获得初值或绑定，不能等进函数体再用普通赋值补救。

```cpp
class BorrowedReading {
public:
    explicit BorrowedReading(const double& source) : source_(source) {}
private:
    const double& source_;
};
```

这个类只是借用 source，并不让外部 double 自动活得更久。构造得正确与长期使用安全，是两项不同要求。

## 22.3 真正的初始化顺序

基类先初始化；随后普通数据成员按**类中声明顺序**初始化；最后执行构造函数体。不是按你在冒号后排列的顺序。

```cpp
// 阅读练习，不推荐的写法。
class Example {
    int first_;
    int second_;
public:
    Example() : second_(2), first_(1) {}
};
```

实际仍先 first_ 后 second_。为避免误导，初始化列表应与声明顺序一致。如果 first_ 的初始化依赖尚未初始化的 second_，就可能读取不合法状态。

## 22.4 默认构造、委托构造与 `explicit`

没有必填参数的构造函数可以用于默认构造。`= default` 请求编译器生成相应特殊成员；它不等于“无条件把所有字段清成零”，成员初值仍很重要。

委托构造允许一个构造函数调用同一类的另一个构造函数：`Counter() : Counter(2) {}`。这样公共验证逻辑只需写一处。

`explicit` 防止某些隐式转换。例如只有一个参数的构造函数，若没有 explicit，可能允许把整数悄悄当作 Counter 使用。对拥有业务含义的类型，显式创建通常更清楚。

<a id="ex15"></a>
**完整例程 15：`examples/15_constructors.cpp`**

```cpp
#include <iostream>
#include <stdexcept>
#include <string>

class NamedCounter {
public:
    explicit NamedCounter(const std::string& name, int capacity = 2)
        : name_(name), capacity_(capacity)
    {
        if (capacity_ <= 0) {
            throw std::invalid_argument("capacity must be positive");
        }
        std::cout << "construct " << name_ << '\n';
    }

    ~NamedCounter()
    {
        std::cout << "destroy " << name_ << '\n';
    }

    int capacity() const { return capacity_; }

private:
    std::string name_;
    int capacity_;
};

int main()
{
    NamedCounter outer("outer");
    {
        NamedCounter inner("inner", 3);
        std::cout << "capacity=" << inner.capacity() << '\n';
    }
    std::cout << "leaving main\n";
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/15_constructors.cpp -o build/ex_15_constructors && \
  ./build/ex_15_constructors
```

**本次运行捕获的标准输出**：

```text
construct outer
construct inner
capacity=3
destroy inner
leaving main
destroy outer
```

这个程序按作用域显示构造与销毁顺序。例程中的异常检查用于拒绝非正容量，完整异常机制在下一章讲；第一次阅读只运行给出的合法输入。

## 22.5 回到 ROS 构造写法

```cpp
VisualDropMissionNode()
    : Node("visual_drop_mission_node")
{
    // 创建订阅、发布器、客户端和定时器。
}
```

这里不是给名叫 Node 的变量赋字符串，而是在初始化基类子对象。基类 `rclcpp::Node` 的完整介绍在第 52 章。先把冒号、括号、函数体的层次区分开。

## 22.6 对象构造完成前不要让它承担完整任务

不要在构造中启动真实投放，不要假定 shared_from_this 已经可用，不要让外部线程在成员尚未全部建立时访问对象。构造阶段建立依赖，显式启动阶段执行业务，通常更容易验证。

**练习**：一个类有 `const int capacity_;`，为什么在构造函数体里写 `capacity_ = 2;` 不行？答案：这是对已经存在的 const 成员进行赋值，应使用初始化列表。

参考：C++17 `[class.ctor]`、`[class.base.init]`、`[dcl.fct.spec]`，见 [R03]。

<a id="ch23"></a>
# 第 23 章：析构、RAII、异常、`noexcept`——资源怎样可靠收尾

## 23.1 资源不只有内存

文件句柄、锁、线程、相机对象、网络连接，都有获得与释放的生命周期。遗漏释放会导致泄漏或占用；释放过早会让后续访问失效。

RAII 的思想是：把资源生命周期绑定到对象生命周期。构造建立资源，析构负责收尾。离开作用域时，即使通过提前 return 或异常退出，已经完成构造的局部对象仍会按规则析构。

<a id="ex16"></a>
**完整例程 16：`examples/16_raii.cpp`**

```cpp
#include <iostream>
#include <stdexcept>

class ScopeMarker {
public:
    explicit ScopeMarker(int& active) : active_(active) { ++active_; }
    ~ScopeMarker() { --active_; }
    ScopeMarker(const ScopeMarker&) = delete;
    ScopeMarker& operator=(const ScopeMarker&) = delete;
private:
    int& active_;
};

int main()
{
    int active = 0;
    try {
        ScopeMarker marker(active);
        std::cout << "inside=" << active << '\n';
        throw std::runtime_error("demo failure");
    } catch (const std::exception& error) {
        std::cout << "caught=" << error.what() << '\n';
    }
    std::cout << "after=" << active << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/16_raii.cpp -o build/ex_16_raii && \
  ./build/ex_16_raii
```

**本次运行捕获的标准输出**：

```text
inside=1
caught=demo failure
after=0
```

示例不操作任何硬件。ScopeMarker 构造时把 active 加一，析构时减一。抛出异常之后，控制流跳到 catch，但 marker 的清理没有丢失，所以 after=0。

## 23.2 析构函数是什么

`~ScopeMarker()` 是析构函数，没有返回类型。普通自动对象离开作用域时调用；由智能指针管理的对象，在相应所有权结束时调用。

如果一个构造函数尚未完成就抛异常，那个完整对象的析构函数不会被调用；但已经成功构造的基类和成员会被清理。所以资源应尽量交给成员对象管理，而不是在构造中随便 new 一串裸资源。

## 23.3 `try`、`throw`、`catch`

`throw` 报告异常并离开当前正常路径；系统寻找合适的 catch。中间已构造的自动对象执行析构，称为栈展开。`catch (const std::exception& error)` 按只读引用接住标准异常基类，可调用 `what()` 获得诊断文本。

不是所有错误都要抛异常。一次桶漏检是正常业务事件，更适合 optional 或结果枚举；配置无法建立、文件读取失败等，可根据接口策略使用异常或显式错误结果。不要在高频循环中把大量预期失败当作异常控制流。

## 23.4 `noexcept` 不等于“内部不会出错”

`noexcept` 表示函数不允许异常正常向外传播。如果异常真的逃出一个 noexcept 函数，程序会终止。它不会自动捕获，也不会自动回滚外部设备。

析构函数应避免把异常传播出去，尤其在已有异常正在展开时。若资源关闭需要报告可恢复错误，可提供显式 `close()`/`stop()` 返回结果，析构负责最后的尽力清理。

## 23.5 `new`、`delete` 和智能指针的铺垫

```cpp
// 仅说明语法；后续新代码优先使用智能指针。
int* pointer = new int(2);
delete pointer;
pointer = nullptr;
```

new 在这里包括申请存储并构造对象；delete 负责相应对象销毁与存储释放。`new[]` 与 `delete[]` 必须配对，不能拿 `delete`、`free` 随意混用。早期 return 或异常可能使手工 delete 被跳过，这正是 RAII 的价值。

## 23.6 软件析构不能替代物理安全状态

关闭一个“舵机客户端对象”，不等于舵机已经回到挂载 PWM；销毁一个 future，不等于飞控撤销了命令；关闭进程，不等于飞机着陆。这些外部状态必须通过明确的任务逻辑和反馈处理。

**练习**：为什么例程把 ScopeMarker 的复制构造和复制赋值声明为 delete？答案：它表示一个独占的清理责任，随便复制可能让多个对象重复执行同一份收尾语义。

参考：C++17 `[class.dtor]`、`[except.ctor]`、`[except.spec]`；资源设计参考 [R07]。

<a id="ch24"></a>
# 第 24 章：复制、移动、左值右值与 Rule of Zero

## 24.1 先分清构造和赋值

```cpp
std::string original = "frame";
std::string copied = original; // 建立新对象：复制构造。
std::string assigned;
assigned = original;          // 对已有对象赋值：复制赋值。
```

复制与赋值不是同一个特殊成员函数。类如果没有自己管理原始资源，通常让 string、vector、智能指针等成员决定资源行为，自己不手写复杂复制代码，称为 Rule of Zero。

## 24.2 左值右值只先掌握最实用的部分

一个有名字、可在后续继续引用的对象表达式通常是左值；临时计算结果通常是右值。完整标准还有 glvalue、prvalue、xvalue 分类，不必先背树状图，但应知道**值类别属于表达式，不只属于类型**。

`T&&` 可表示右值引用，常用于移动操作。一个有名字的右值引用参数，在函数体里用它的名字时仍是左值表达式。这是后面 `std::move`、`std::forward` 存在的原因。

## 24.3 `std::move` 自己不搬数据

```cpp
std::string destination = std::move(original);
```

move 让表达式可用于选择移动相关操作；真正怎样转移资源，由目标类型的移动构造或赋值决定。它不是把内存中的字节强行挪到另一个地址，也不保证任何类型都获得性能提升。

<a id="ex17"></a>
**完整例程 17：`examples/17_move.cpp`**

```cpp
#include <iostream>
#include <string>
#include <utility>

class Packet {
public:
    explicit Packet(std::string label) : label_(std::move(label)) {}
    Packet(const Packet& other) : label_(other.label_)
    {
        std::cout << "copy\n";
    }
    Packet(Packet&& other) noexcept : label_(std::move(other.label_))
    {
        std::cout << "move\n";
    }
    Packet& operator=(const Packet&) = default;
    Packet& operator=(Packet&&) noexcept = default;
    const std::string& label() const { return label_; }
private:
    std::string label_;
};

int main()
{
    Packet source("frame-1");
    Packet copied = source;
    Packet moved = std::move(source);
    std::cout << "copied=" << copied.label() << '\n';
    std::cout << "moved=" << moved.label() << '\n';
    // 不假设 source 内部的 string 移动后一定为空；重新赋值后再使用。
    source = Packet("frame-2");
    std::cout << "source=" << source.label() << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/17_move.cpp -o build/ex_17_move && \
  ./build/ex_17_move
```

**本次运行捕获的标准输出**：

```text
copy
move
copied=frame-1
moved=frame-1
source=frame-2
```

例程用打印区分 copy 与 move。移动后的 source 仍然是一个存在的对象，可以析构或重新赋值。对于本例内部的标准 string，不应在未重新赋值前依赖其具体内容“必定为空”。而移动后的 unique_ptr 有明确的空指针语义，下一章会看到。

## 24.4 为什么不随手 `return std::move(local)`

按值返回局部对象时，编译器可以执行适用的复制消除；C++17 对一些 prvalue 场景有更强保证，而具名返回值优化仍有条件。手工加入 std::move 可能阻碍某些优化。

优先写清楚的 `return result;`，让类型拥有合理移动/复制能力。不要为了“看起来高性能”牺牲可读性和正确性。

## 24.5 Rule of Five 到底解决什么

如果类直接管理一个需要独占释放的资源，就必须审视析构、复制构造、复制赋值、移动构造、移动赋值这五种操作，避免双重释放、泄漏或错误共享。不是要求每个类都写五个长函数。

可以禁止复制，只允许移动；也可以完全不可复制不可移动。自己声明析构或复制操作，还可能影响编译器是否自动生成移动操作，所以不能想当然地认为“编译器一定会替我生成所有版本”。

`= default` 表示使用默认实现；`= delete` 表示该操作不允许被调用。二者与函数体为空不同。

## 24.6 移动与引用失效

v11 从 pending 队列取视觉帧时，先把队首对象移动到局部变量，再 pop_front。局部 frame 拥有自己接手的检测数据，后续可以使用；指向原队首对象的引用在删除后不能继续访问。

**练习**：`std::move` 后原对象是否一律消失？答案：不是。对象生命周期与其资源是否被转移是两件事，具体可用状态取决于类型契约。

参考：C++17 `[basic.lval]`、`[class.copy]`、`[forward]`，见 [R03]；设计习惯参考 [R07]。

<a id="ch25"></a>
# 第 25 章：`unique_ptr`——一个资源只有一个拥有者

## 25.1 所有权是一种责任

指针说明“对象在哪里”；所有权说明“谁负责让对象最终被销毁”。裸指针本身并不明确这个责任。`std::unique_ptr<T>` 表达独占拥有一个 T 对象，默认销毁策略会在所有权结束时 delete 对象。

<a id="ex18"></a>
**完整例程 18：`examples/18_unique.cpp`**

```cpp
#include <iostream>
#include <memory>
#include <utility>

struct Reading { double height_m = 0.0; };

void observe(const Reading& reading)
{
    std::cout << "height=" << reading.height_m << '\n';
}

int main()
{
    auto owner = std::make_unique<Reading>();
    owner->height_m = 1.3;
    observe(*owner);
    auto next_owner = std::move(owner);
    std::cout << std::boolalpha << "old owner empty=" << (owner == nullptr) << '\n';
    observe(*next_owner);
    next_owner.reset();
    std::cout << "next owner empty=" << (next_owner == nullptr) << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/18_unique.cpp -o build/ex_18_unique && \
  ./build/ex_18_unique
```

**本次运行捕获的标准输出**：

```text
height=1.3
old owner empty=true
height=1.3
next owner empty=true
```

`std::make_unique<Reading>()` 建立 Reading 并返回 unique_ptr。`owner->height_m` 访问对象；`observe(*owner)` 把对象以引用借给函数，没有转移所有权。

## 25.2 为什么复制被禁止

如果 `auto second = owner;` 也得到独占拥有者，就会有两个对象都认为自己应该销毁同一资源。unique_ptr 因此禁止普通复制。

`auto next_owner = std::move(owner);` 显式转移所有权。转移后 owner 为空；next_owner 负责资源。不要继续 `owner->...`。

## 25.3 常用操作

| 操作 | 含义 | 常见误用 |
|---|---|---|
| `p.get()` | 取得不转移所有权的裸指针 | 对 get 返回值手工 delete |
| `p.reset()` | 释放当前所拥有对象并变为空 | 之后继续解引用 |
| `p.reset(new_pointer)` | 接管新指针并处理旧对象 | 让两个拥有者接管同一裸指针 |
| `p.release()` | 放弃所有权并返回裸指针，不负责删除 | 以为 release 自动释放内存 |
| `if (p)` | 判断是否非空 | 以为非空就说明业务数据有效 |

release 是高级所有权交接接口，不是日常的“清空”。一般优先 reset 或作用域结束。

## 25.4 什么时候选 unique_ptr

普通 Point3 不必动态分配，直接作为成员通常更简单。需要运行时选择不同实现、对象不能方便按值保存、或者有清楚的独占生命周期时，可以用 unique_ptr。

不要为了“现代 C++”把每个 double 都放进智能指针。更少的分配和更直接的数据结构，往往更容易推理。

## 25.5 借用接口不必要求智能指针

只需要读一个对象的函数，接收 `const T&` 通常就够；只有确实要接管所有权，才接收 `unique_ptr<T>` 并移动。接口要求与它真实承担的责任应一致。

**练习**：`observe(*owner)` 为什么不延长 Reading 的生命周期？答案：传的是短期引用，生命周期仍由 owner 管理；调用过程中 owner 必须保持拥有状态。

参考：C++17 `[unique.ptr]`，见 [R03]；所有权设计参考 [R07]。

<a id="ch26"></a>
# 第 26 章：`shared_ptr`、`weak_ptr` 与 ROS 的 `SharedPtr`

## 26.1 多个拥有者共享同一个对象

`std::shared_ptr<T>` 允许多个 shared_ptr 共同拥有一个对象。复制指针会共享对象，不会自动复制 T 本身。当最后一个共享拥有者结束时，所管理的对象按删除器规则销毁。

<a id="ex19"></a>
**完整例程 19：`examples/19_shared_weak.cpp`**

```cpp
#include <iostream>
#include <memory>

struct Reading { double height_m = 1.3; };

int main()
{
    auto first = std::make_shared<Reading>();
    std::weak_ptr<Reading> weak = first;
    {
        auto second = first;
        std::shared_ptr<const Reading> read_only = second;
        std::cout << "owners=" << first.use_count() << '\n';
        std::cout << "readonly height=" << read_only->height_m << '\n';
    }
    if (auto locked = weak.lock()) {
        std::cout << "lock height=" << locked->height_m << '\n';
    }
    first.reset();
    std::cout << std::boolalpha << "expired=" << weak.expired() << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/19_shared_weak.cpp -o build/ex_19_shared_weak && \
  ./build/ex_19_shared_weak
```

**本次运行捕获的标准输出**：

```text
owners=3
readonly height=1.3
lock height=1.3
expired=true
```

例程中 first、second、read_only 指向同一个 Reading，所以在那个代码块内观察到三份共享所有权。退出块后两份消失，但 first 仍拥有它。

`use_count()` 适合教学观察，不应被拿来作为多线程下复杂生命周期决策的可靠条件。

## 26.2 `weak_ptr` 观察但不拥有

weak 不增加共享拥有者数量。通过 `weak.lock()` 尝试临时取得一个 shared_ptr；成功则可以安全访问该对象，失败则表示资源已不能由这个 weak 取得。

```cpp
if (auto locked = weak.lock()) {
    // locked 在这个范围内持有共享所有权。
}
```

这比先 `expired()` 再单独访问更合适：状态可能变化，而 lock 将“检查并获得所有权”组合成相应操作。

## 26.3 循环引用

若 A 用 shared_ptr 拥有 B，而 B 也用 shared_ptr 拥有 A，即使外部引用消失，两者也可能无法归零。一个方向应当只是 weak_ptr 观察，或重新设计所有权图。

同样，Node 拥有 Timer，Timer 的回调又强持有 Node，也可能形成环。`[this]` 不强持有 Node，却要求 Node 在回调运行时仍存在。选择哪种方式必须结合执行器停止、回调资源销毁和对象生命周期，不是统一替换一个关键字就能解决。

## 26.4 最容易混淆的四种 const

假设 `Msg::SharedPtr` 等价于 `std::shared_ptr<Msg>`：

| 类型写法 | 指针句柄可改指向 | 可经它修改 Msg |
|---|---|---|
| `Msg::SharedPtr` | 可以 | 可以 |
| `const Msg::SharedPtr` | 当前句柄不能重新赋值 | 通常仍可修改 Msg |
| `std::shared_ptr<const Msg>` / 常见 `Msg::ConstSharedPtr` | 可以 | 不可以 |
| `const std::shared_ptr<const Msg>` | 不可以 | 不可以 |

所以 `const nav_msgs::msg::Odometry::SharedPtr msg` 中，const 主要约束局部 shared_ptr 句柄，不等价于消息内容只读。要表明只读消息，优先理解并使用 `ConstSharedPtr` 对应的类型。

## 26.5 `make_shared` 与原始指针

`make_shared<T>(arguments...)` 构造一个由 shared_ptr 管理的对象。不要对同一个裸指针分别建立两个独立 shared_ptr；这样会产生两个控制块和重复销毁风险。也不要对已经由 shared_ptr 管理的对象随便写 `shared_ptr<T>(this)`。

需要从对象内获得现有共享所有权时，某些类使用 `enable_shared_from_this`；但构造函数执行时通常尚不能使用 shared_from_this 获取有效共享拥有者。ROS 相关写法在后面再看，不要求初学时立即自己设计这种类。

## 26.6 智能指针不会自动让消息线程安全

共享引用计数的实现有相应并发保证，不等于被管理的 T 的所有字段可以无锁并发写。多个回调同时改同一 Point3，仍然需要互斥、原子或明确的单线程约束。

**回到 v11**：成员 `timer_`、订阅器和客户端保存智能指针，是为了维持 ROS 资源生命周期；这不意味着它们持有的业务状态天然不可出错。

参考：C++17 `[util.smartptr.shared]`、`[util.smartptr.weak]`，见 [R03]；const 说明参考 [R05]。

<a id="ch27"></a>
# 第 27 章：继承、虚函数、`override`、`final` 与组合

## 27.1 继承表示一种类型关系

```cpp
class FakeHeightSource final : public HeightSource
```

冒号引出基类，public 表达公开继承，final 表示不允许继续从 FakeHeightSource 派生。它不同于构造函数中的冒号，必须结合上下文阅读。

<a id="ex20"></a>
**完整例程 20：`examples/20_inheritance.cpp`**

```cpp
#include <iostream>
#include <memory>

class HeightSource {
public:
    virtual ~HeightSource() = default;
    virtual double height_m() const = 0;
};

class FakeHeightSource final : public HeightSource {
public:
    explicit FakeHeightSource(double height) : height_(height) {}
    double height_m() const override { return height_; }
private:
    double height_;
};

void print_height(const HeightSource& source)
{
    std::cout << "height=" << source.height_m() << '\n';
}

int main()
{
    std::unique_ptr<HeightSource> source = std::make_unique<FakeHeightSource>(1.3);
    print_height(*source);
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/20_inheritance.cpp -o build/ex_20_inheritance && \
  ./build/ex_20_inheritance
```

**本次运行捕获的标准输出**：

```text
height=1.3
```

例程中的 HeightSource 定义“能够提供高度”的接口。FakeHeightSource 实现这个接口，返回教学常数。调用者通过基类引用使用它，不需要知道它内部到底读相机还是返回假值。

## 27.2 虚函数与动态分派

`virtual double height_m() const = 0;` 是纯虚函数，表示接口要求派生类提供实现。含有未实现纯虚函数的类是抽象类，不能直接创建实例。

派生类写 `double height_m() const override`，让编译器检查它确实覆盖了一个合适的基类虚函数。拼错名字、参数不同或遗漏 const，override 能帮助发现错误。

没有 virtual 的普通同名函数，不会因为“名字一样”自动获得相同的运行时分派行为。

## 27.3 为什么基类析构函数通常要 virtual

通过基类指针拥有派生对象并销毁时，需要正确调用派生析构逻辑。对这种多态拥有接口，基类应提供合适的虚析构。

例程用 `unique_ptr<HeightSource>` 管理 FakeHeightSource，正因为基类有 `virtual ~HeightSource() = default;` 才符合这种使用方式。

## 27.4 对象切片

把派生对象按值复制到一个基类对象，只能保留基类部分，称为切片。为了保留动态类型，通常用基类引用或指针传递，而不是按值把多态对象切成一块。

同样，public 继承不是通用的“代码复用按钮”。“Mission 里面有一个 Tracker”更自然地表达为成员组合，不应该写成“Mission 是一种 Tracker”。

## 27.5 对 v11 的实际意义

v11 继承 `rclcpp::Node` 是为了成为一个 ROS 节点，并获得创建通信资源、日志和时钟等接口。大部分算法模块却不需要继承 Node：几何、滤波、轨迹等可以保持普通函数或普通类，更容易独立测试。

本书不要求初学者先掌握复杂多重继承或虚继承。它们不属于读懂当前任务节点的必要前提；遇到相关库源码再单独学习。

**练习**：你会让 `Mission` 继承 `BucketTrack`，还是让它拥有 active_bucket 成员？答案：通常选后者；任务不是一个桶轨迹，二者是包含关系。

参考：C++17 `[class.derived]`、`[class.virtual]`、`[class.dtor]`，见 [R03]。

<a id="ch28"></a>
# 第 28 章：函数指针、成员函数指针、`std::function` 和 `std::bind`

## 28.1 回调的最小含义

回调不是某种神秘线程。它只是把一段“以后要调用的操作”交给另一段代码保存，在事件发生时由对方调用。保存回调与执行回调是两个不同动作。

先看普通函数指针：

```cpp
double twice(double value);
double (*function_pointer)(double) = &twice;
```

括号必须保留。function_pointer 是变量名；`(*function_pointer)` 表明它是指针；后面的 `(double)` 是所指函数的参数列表；最左边 double 是返回类型。

## 28.2 成员函数还需要一个对象

`&Scaler::apply` 不是“立刻调用 apply”，而是形成一个成员函数指针。它没有绑定到哪个 Scaler 对象，所以调用时还要给出对象：

```cpp
(object.*member_pointer)(2.0);
```

通过对象指针调用时会看到 `->*`。实际 ROS 应用常不直接写这些调用，而是使用 Lambda 或 bind 把“成员函数 + 当前对象”打包。

<a id="ex21"></a>
**完整例程 21：`examples/21_callables.cpp`**

```cpp
#include <functional>
#include <iostream>

double twice(double value) { return value * 2.0; }

struct Scaler {
    double factor = 3.0;
    double apply(double value) const { return factor * value; }
};

int main()
{
    double (*function_pointer)(double) = &twice;
    std::cout << "free=" << function_pointer(2.0) << '\n';
    Scaler object;
    double (Scaler::*member_pointer)(double) const = &Scaler::apply;
    std::cout << "member=" << (object.*member_pointer)(2.0) << '\n';
    auto bound = std::bind(&Scaler::apply, &object, std::placeholders::_1);
    std::function<double(double)> callback = bound;
    std::cout << "bound=" << callback(2.0) << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/21_callables.cpp -o build/ex_21_callables && \
  ./build/ex_21_callables
```

**本次运行捕获的标准输出**：

```text
free=4
member=6
bound=6
```

## 28.3 把 `std::bind` 展开成自然语言

```cpp
std::bind(&Scaler::apply, &object, std::placeholders::_1)
```

第一项：以后调用 Scaler::apply；第二项：使用 object 这个实例；第三项：apply 的第一个参数由将来调用 bound 时提供。`_1` 是占位符，不是“订阅第一条消息”的计数器。

因此 `bound(2.0)` 大致表达 `object.apply(2.0)`。bind 保存的是对象指针时，object 必须在调用时仍存活；bind 不会自动延长一个裸对象的生命周期。

## 28.4 `std::function` 保存统一调用接口

`std::function<double(double)>` 表示一个可调用对象，接收一个 double，返回一个 double。它可以包装相容的函数指针、bind 结果、Lambda 或带 `operator()` 的函数对象。

它使用类型擦除：调用者只关心签名，不关心具体可调用类型。便利背后可能有额外存储、复制或动态分配成本；是否发生要看实现和可调用对象大小，不能声称“永远零开销”。

空的 std::function 不能直接调用，应先检查。C++17 的 std::function 对保存的可调用对象有复制相关要求；捕获独占 unique_ptr 的移动专用 Lambda 不一定能直接放进去。

## 28.5 回到 v11 的绑定

```cpp
std::bind(&VisualDropMissionNode::odom_callback,
          this,
          std::placeholders::_1)
```

现在可以读成：创建一个回调，收到参数后在当前任务对象上调用 odom_callback。里面没有“自动等待消息”的语言机制；真正接收消息并决定何时调用的是 ROS 执行器。

第 30 章会写出等价 Lambda。先理解这两种写法背后的对象和参数，才不会只是背一个长表达式。

**本篇验收**：能画出资源拥有关系，能判断对象什么时候销毁，能区分 const 指针句柄与只读对象，能解释 `[this]` 或 bind 中的 this 为什么不等于共享所有权。

参考：C++17 `[func.wrap.func]`、`[func.bind]`、`[expr.mptr.oper]`，见 [R03]。


# 第五篇：读懂现代 C++ 的类型、容器和算法

<a id="ch29"></a>
# 第 29 章：`auto`、类型别名、`decltype` 与长类型拆解

## 29.1 `auto` 不表示动态类型

```cpp
auto count = 2;       // 推导为 int。
auto height = 1.3;    // 推导为 double。
```

变量类型在编译时已经确定，后面不能因为给 height 赋字符串就变成字符串类型。auto 不是 Python 式的“任意对象容器”。

auto 需要从初始化表达式获得信息，普通 `auto value;` 不足以推导。数字写成 2 还是 2.0，也会影响推导类型。

## 29.2 `auto` 与引用、const 的相互作用

<a id="ex22"></a>
**完整例程 22：`examples/22_auto_types.cpp`**

```cpp
#include <iostream>
#include <type_traits>

int main()
{
    const int fixed = 2;
    auto copy = fixed;
    const auto& view = fixed;
    decltype(fixed) same_declared_type = 3;
    // static_assert 是编译时断言，不会在运行时打印。
    static_assert(std::is_same_v<decltype(copy), int>, "copy is int");
    static_assert(std::is_same_v<decltype(view), const int&>, "view is const int&");
    static_assert(std::is_same_v<decltype(same_declared_type), const int>, "const preserved");
    copy = 4;
    std::cout << "fixed=" << fixed << " copy=" << copy << " view=" << view << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/22_auto_types.cpp -o build/ex_22_auto_types && \
  ./build/ex_22_auto_types
```

**本次运行捕获的标准输出**：

```text
fixed=2 copy=4 view=2
```

`auto copy = fixed` 得到独立的 int 值，不保留被复制对象顶层 const；`const auto& view = fixed` 得到只读引用。`auto&` 保留引用关系；`auto&&` 涉及引用折叠，可以在泛型代码里绑定不同值类别，初学时不要把它当作“更快的 auto”。

例程中的 `static_assert` 是编译期检查，条件不满足就编译失败，不会到运行时打印。`std::is_same_v` 来自 `<type_traits>`，用于比较类型，具体模板机制下一章组展开。

## 29.3 类型别名不是新类型

```cpp
using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
```

using 在这里给已有类型取较短名字。TimePoint 与相应原类型完全相同。`using Meters = double; using Seconds = double;` 也不会让编译器禁止米和秒混算；要获得强单位约束，需要不同的结构体或成熟单位库。

ROS 的 `SharedPtr` 常就是嵌套类型别名，所以看到 `rclcpp::TimerBase::SharedPtr` 时，应依次读作用域，不要把整个名字当成一个不可拆解的关键词。

## 29.4 `decltype` 与表达式

`decltype(x)` 对直接命名实体有特殊规则，通常取得其声明类型；`decltype((x))` 因为是一般表达式，可能取得引用类型。两者不同，不能为了“多加一层括号更保险”随意替换。

`decltype(auto)` 可以保留表达式相关的引用语义，因此也更容易无意中返回悬空引用。当前任务业务层若不需要复杂泛型返回，直接写清楚返回类型往往更合适。

## 29.5 长类型的阅读顺序

例如：

```cpp
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
```

先找最外层：Subscription 的 SharedPtr；再看模板参数：Odometry 消息类型；再看命名空间：Subscription 属于 rclcpp，Odometry 属于 nav_msgs::msg。它不是五种不同指针，而是一个具体订阅器类型的智能指针别名。

**练习**：`const auto result = *home;` 是复制还是引用？答案：这里是按值推导并复制 contained value，再让 result 只读；要借用通常写 `const auto& result = *home;`，并承担 home 生命周期要求。

参考：C++17 `[dcl.spec.auto]`、`[dcl.type.simple]`、`[dcl.typedef]`，见 [R03]。

<a id="ch30"></a>
# 第 30 章：Lambda 逐个符号拆解——捕获、参数、返回、生命周期

## 30.1 从一个普通函数走到 Lambda

普通函数可以写成 `bool enough(int count) { return count >= 3; }`。如果判据只在一个排序或订阅语句旁使用，可以在使用处创建一个无名可调用对象：

```cpp
auto enough = [](int count) { return count >= 3; };
```

`[]` 是捕获列表；`(int count)` 是参数；`{...}` 是函数体；最后分号结束变量初始化语句。Lambda 表达式创建闭包对象，创建时不会自动执行函数体。执行 `enough(4)` 才调用它。

## 30.2 捕获不等于参数

参数由调用者在每次调用时提供；捕获把创建 Lambda 时的外部环境带入闭包。`[threshold]` 把 threshold 复制进闭包；`[&threshold]` 借用外部变量。

<a id="ex23"></a>
**完整例程 23：`examples/23_lambdas.cpp`**

```cpp
#include <functional>
#include <iostream>
#include <memory>

int main()
{
    int threshold = 3;
    auto by_value = [threshold](int count) { return count >= threshold; };
    auto by_reference = [&threshold](int count) { return count >= threshold; };
    threshold = 5;
    std::cout << std::boolalpha;
    std::cout << "value=" << by_value(4) << " reference=" << by_reference(4) << '\n';

    auto counter = [count = 0]() mutable { return ++count; };
    std::cout << "counter=" << counter() << ',' << counter() << '\n';

    std::function<int()> later;
    {
        auto state = std::make_shared<int>(7);
        later = [state]() { return *state; }; // 按值捕获所有权，state 活过这个代码块。
    }
    std::cout << "later=" << later() << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/23_lambdas.cpp -o build/ex_23_lambdas && \
  ./build/ex_23_lambdas
```

**本次运行捕获的标准输出**：

```text
value=true reference=false
counter=1,2
later=7
```

把外部 threshold 从 3 改为 5 后，按值捕获者仍使用创建时的 3，按引用捕获者使用当前的 5。两者适用于不同需求：冻结任务配置更适合值语义；需要查看当前对象状态时可能使用引用或 this，但必须保证生命周期。

## 30.3 常见捕获形式

| 捕获形式 | 含义 | 要检查的风险 |
|---|---|---|
| `[]` | 不捕获局部环境 | 只能使用参数、可见的静态/全局等名字 |
| `[x]` | 复制 x | x 是指针时只复制指针，不复制对象 |
| `[&x]` | 引用 x | 调用时 x 是否仍存在 |
| `[this]` | 复制当前对象指针 | 当前对象是否仍存在、是否并发修改 |
| `[=]` | 默认按值捕获需要的局部变量 | 容易隐藏实际捕获，this 的语义尤其要注意 |
| `[&]` | 默认按引用捕获需要的局部变量 | 存为长期回调时容易悬空 |
| `[name = expression]` | 初始化捕获 | 可命名快照、移动资源、明确含义 |

C++17 支持 `[*this]` 复制当前对象，但 Node 等不可复制或语义复杂的对象不适合盲目使用。不要把它当作修复 this 生命周期的通用办法。

## 30.4 `mutable` 改的是闭包内部的可修改性

`[count = 0]() mutable { return ++count; }` 在闭包自己的 count 上计数，不会修改某个同名外部变量。默认的按值捕获在普通 Lambda 调用中经常表现为只读；mutable 允许改变闭包保存的副本。

这与类数据成员上的 mutable 是相关但不同的语法位置。学习时应先问：“现在被允许修改的是哪个对象？”

## 30.5 引用捕获的悬空例子

```cpp
// 错误示例，不要用于运行。
std::function<int()> make_bad_callback()
{
    int local = 7;
    return [&local]() { return local; };
}
```

函数返回后 local 已销毁，回调里的引用悬空。若需要冻结值，改成 `[local]`；若要共享长期状态，应明确分配和持有所有权，而不是把 `&` 改来改去碰运气。

## 30.6 泛型 Lambda 和显式返回类型

`[](const auto& item) { ... }` 表示参数类型可由调用推导，属于泛型 Lambda。`[](...) -> bool { ... }` 显式写返回类型。箭头在这里不是成员访问。

用泛型 Lambda 能减少重复，但新人读实际状态机时，先把 `auto` 还原成具体 BucketTrack 或 NavigationSample，通常更容易判断代码是否合法。

## 30.7 与 `std::bind` 的等价对照

```cpp
[this](const nav_msgs::msg::Odometry::SharedPtr msg) {
    this->odom_callback(msg);
}
```

它与前文绑定当前对象成员函数的意图相近。Lambda 把参数转发和捕获写在一个位置，通常更直观；阅读原代码仍要能够理解 bind。

参考：C++17 `[expr.prim.lambda]`、`[expr.prim.lambda.capture]`，见 [R08]。

<a id="ch31"></a>
# 第 31 章：模板从最简单函数开始——类型参数不是运行参数

## 31.1 为什么同一个算法要支持不同类型

比较两个 int 与两个 double，核心逻辑都是“返回较小值”。模板让你把类型也作为编译时参数，而不是写两份几乎一样的函数。

<a id="ex24"></a>
**完整例程 24：`examples/24_templates.cpp`**

```cpp
#include <iostream>
#include <string>
#include <utility>

template<typename T>
T smaller(T left, T right)
{
    return left < right ? left : right;
}

template<typename T>
class Box {
public:
    using Value = T;
    explicit Box(T value) : value_(std::move(value)) {}
    const T& value() const { return value_; }
private:
    T value_;
};

template<typename T>
typename Box<T>::Value copy_value(const Box<T>& box)
{
    return box.value();
}

int main()
{
    std::cout << "integer=" << smaller(2, 3) << '\n';
    std::cout << "real=" << smaller<double>(2.0, 1.3) << '\n';
    Box<std::string> name("CUADC");
    std::cout << copy_value(name) << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/24_templates.cpp -o build/ex_24_templates && \
  ./build/ex_24_templates
```

**本次运行捕获的标准输出**：

```text
integer=2
real=1.3
CUADC
```

`template<typename T>` 声明一个类型参数 T；`T smaller(T left, T right)` 用该类型描述输入和返回；调用 smaller(2,3) 时可以从实参推导 T=int。

## 31.2 尖括号和圆括号分别放什么

```cpp
smaller<double>(2.0, 1.3)
```

尖括号指定模板实参类型 double；圆括号提供本次运行调用的数据。`create_subscription<Odometry>(topic, qos, callback)` 的结构是同一种思路：消息类型在模板参数位置，话题名称和回调在运行参数位置。

`smaller(2, 1.3)` 对这个简单模板不能直接推导一个相同的 T，因为两个实参分别给出 int 与 double。可以统一输入类型或显式选择 T，但转换风险仍由你负责。

## 31.3 类模板

`Box<int>` 与 `Box<std::string>` 是由同一个类模板得到的不同具体类型。类模板不是“一个对象里面随时改变字段类型”；具体实例类型在编译时就确定。

`std::vector<Point3>`、`std::optional<TimePoint>`、`rclcpp::Client<Service>` 都应按这个模式阅读，不需要先会写它们的内部实现。

## 31.4 `typename` 的两种常见位置

在 `template<typename T>` 中，typename 表示 T 是类型参数；在 `typename Box<T>::Value` 中，它告诉编译器：依赖模板参数 T 的这个嵌套名字应按类型解释。

没有依赖模板参数时，例如普通 `Clock::time_point`，通常不需要这样帮助消歧。不要把 typename 机械地加在每个 `::` 前后。

## 31.5 为什么模板定义经常放在头文件

编译器在生成某个具体模板实例时，通常需要看到模板定义。因此把模板函数实现只放在一个普通 `.cpp` 中，而其他文件只看到声明，可能得到实例化或链接问题。常用方式是把定义放进头文件；显式实例化属于另一种受控组织方式。

这与“所有普通函数都放头文件”不是同一建议。普通非 inline 函数的多文件定义涉及 ODR，第 41 章再讲。

## 31.6 `if constexpr`、类型特征与转发：会读即可，不必滥用

C++17 的 if constexpr 按编译期条件选择分支，适合泛型代码。`std::is_same_v`、`std::is_arithmetic_v` 之类类型特征描述类型性质。`static_assert` 在编译期验证条件。

模板参数推导中的 `T&&` 可以成为转发引用；`std::forward<T>` 保留调用者表达式的适当值类别，`std::move` 则明确把表达式转换成可移动的形式。若不清楚区别，业务函数优先用具体类型和值/const 引用，不要自行创造“万能转发包装层”。

ROS 内部常见 `enable_if`、`typename ...::type`、参数包 `...`，它们用于约束重载或转发可变参数。读应用层时理解其作用即可，先不要跳进所有模板元编程细节。

## 31.7 精读 v11 的 `value_at<T>`

它在容器内有该下标时返回该项；容器非空但下标越界时返回最后一项；只有容器为空时才返回 fallback。旧教程把它概括为“越界时 fallback”过于粗略。

这个差别必须写测试：`{0.7,0.8}`、index=99、fallback=1.0 的结果是 0.8，不是 1.0。**模板机制与业务 fallback 策略要分别阅读。**

参考：C++17 `[temp]`、`[temp.deduct]`、`[stmt.if]`；源码依据 [B1]。

<a id="ch32"></a>
# 第 32 章：`std::array` 与 `std::vector`——长度、容量、元素和生命周期

## 32.1 先区分固定长度和动态长度

`std::array<T,N>` 的元素数量 N 是类型的一部分，适合固定两个舵机通道等数据；`std::vector<T>` 的元素数量可以运行时变化，适合检测列表、航点列表和轨迹集合。

vector 对普通元素提供连续存储，可以按下标访问；对象本身与它管理的元素存储不是同一回事。不要因为 vector 是局部变量，就推断其全部元素都必须位于栈上。

<a id="ex25"></a>
**完整例程 25：`examples/25_vector.cpp`**

```cpp
#include <array>
#include <iostream>
#include <stdexcept>
#include <vector>

int main()
{
    std::vector<double> values;
    values.reserve(4);
    std::cout << "after reserve: size=" << values.size() << '\n';
    values.push_back(2.2);
    values.push_back(1.7);
    values.emplace_back(1.3);
    for (double& value : values) {
        value += 0.1;
    }
    for (const double& value : values) {
        std::cout << value << ' ';
    }
    std::cout << '\n';
    try {
        std::cout << values.at(99) << '\n';
    } catch (const std::out_of_range&) {
        std::cout << "index rejected\n";
    }
    std::array<int, 2> channels{9, 10};
    std::cout << "channel count=" << channels.size() << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/25_vector.cpp -o build/ex_25_vector && \
  ./build/ex_25_vector
```

**本次运行捕获的标准输出**：

```text
after reserve: size=0
2.3 1.8 1.4 
index rejected
channel count=2
```

## 32.2 `size()` 与 `capacity()`

size 是当前真正存在的元素数量；capacity 是无需重新分配即可容纳的容量。`reserve(4)` 只保证相关容量，不创建四个可用元素。所以紧接着访问 values[0] 仍然不合法。

`resize(4)` 才把元素数量改为四；增加的新元素按相应规则初始化。`clear()` 删除所有元素并令 size 为零，但通常不强制释放已分配容量。`shrink_to_fit()` 是非强制性的缩容请求，不能当成必须发生的内存释放保证。

## 32.3 圆括号与花括号可能产生不同含义

```cpp
std::vector<double> a(3, 1.3); // 三个元素，都是 1.3。
std::vector<double> b{3, 1.3}; // 两个元素：3.0 和 1.3。
```

第二个通常选择 initializer_list 相关构造。不要把“花括号更现代”当作无脑替换理由。

## 32.4 访问和添加

| 方法 | 意义 | 前提或注意事项 |
|---|---|---|
| `v[i]` | 下标访问 | 必须 `i < size()`，不自动抛越界异常 |
| `v.at(i)` | 带范围检查的访问 | 越界抛 `std::out_of_range` |
| `v.front()` / `back()` | 首/末元素 | 容器必须非空 |
| `push_back(value)` | 追加一个值 | 可能复制/移动，可能重新分配 |
| `emplace_back(args...)` | 用参数在容器位置构造 | 不代表无条件比 push_back 快或无风险 |
| `pop_back()` | 删除末元素 | 必须非空，相关引用失效 |

## 32.5 遍历中的复制与借用

`for (auto item : values)` 每次按值取得元素；`auto&` 借用并允许修改；`const auto&` 借用只读。对较大的 BucketTrack，意外复制会增加成本；对很小的 double，按值遍历完全合理。

在基于引用的遍历中向同一个 vector 追加元素，可能导致重分配和引用失效。先收集修改，再统一追加，或使用明确的索引/分阶段逻辑，通常更好审查。

## 32.6 `vector<bool>` 是特殊化

vector<bool> 可能使用位压缩并返回代理对象，不具备与 vector<int> 完全相同的元素引用行为。v11 用它保存关联使用标记是可读的，但学习一般引用规则时，不要拿 vector<bool> 当普通 T& 的典型代表。教学 Tracker 使用 unsigned char 标记减少这一额外难点。

**练习**：为什么 reserve 以后 size 仍为零？答案：预留存储能力不等于建立元素对象。

参考：C++17 `[array]`、`[vector]`、`[vector.capacity]`、`[vector.modifiers]`，见 [R03]、[R09]。

<a id="ch33"></a>
# 第 33 章：`deque`、队列、历史窗口与移动后删除

## 33.1 为什么历史不一定用 vector

导航历史的常见操作是尾部追加、头部删除。deque 提供双端操作，避免每次删除 vector 首元素时移动后面的大量元素。它也支持随机访问，但不保证像 vector 那样整块连续。

<a id="ex26"></a>
**完整例程 26：`examples/26_deque.cpp`**

```cpp
#include <cstddef>
#include <deque>
#include <iostream>

int main()
{
    std::deque<int> history;
    const std::size_t capacity = 3;
    for (int sample = 1; sample <= 5; ++sample) {
        history.push_back(sample);
        while (history.size() > capacity) {
            history.pop_front();
        }
    }
    std::cout << "window:";
    for (int sample : history) {
        std::cout << ' ' << sample;
    }
    std::cout << '\n';
    while (!history.empty()) {
        int value = history.front(); // 先取值，后删除；不保留将被删除元素的引用。
        history.pop_front();
        std::cout << "consume=" << value << '\n';
    }
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/26_deque.cpp -o build/ex_26_deque && \
  ./build/ex_26_deque
```

**本次运行捕获的标准输出**：

```text
window: 3 4 5
consume=3
consume=4
consume=5
```

样本 1～5 进入一个容量为三的窗口，最后留下 3、4、5。理解窗口时，要同时看“现在有多少元素”和“最旧样本何时被删”。

## 33.2 按数量裁剪与按时间裁剪

`while (history.size() > max_count)` 限制内存数量；`while (newest_time - oldest_time > duration)` 限制时间跨度。它们不是同一条件。

真实视觉队列通常还需要限制单次处理工作量和总积压。只保留五秒历史，不能自动保证每秒只有固定数量样本，也不能证明主线程绝不会被队列处理拖慢。

## 33.3 先移动到局部，再删除队首

v11 的模式如下：

```cpp
PendingVisionFrame frame = std::move(pending_vision_frames_.front());
pending_vision_frames_.pop_front();
```

第一句让 frame 接收队首对象的可移动资源；第二句删除队首对象。之后使用 frame。错误模式是先绑定队首引用，再 pop_front，再继续用那个引用。

```cpp
// 错误示例。
// const auto& frame = queue.front();
// queue.pop_front();
// use(frame); // 引用所指向元素已被删除。
```

## 33.4 引用与迭代器失效规则不是一张表通吃

deque 在两端插入时，对迭代器与已有元素引用的影响可能不同；删除哪个元素、是否从中间修改，也会改变结果。最稳妥的业务方式是：不要长期保存会被修改的历史容器迭代器，查询后立即使用需要的值或快照。

std::queue 是容器适配器，暴露 push、front、pop 等队列操作，但不提供通用遍历。需要按时间二分查找时，deque 更适合当前教学目标。

**回到 v11**：navigation_history_ 是历史，pending_vision_frames_ 是等待处理的工作，两者即使都用 deque，淘汰和等待策略也完全不同。

参考：C++17 `[deque]`、`[queue]`，见 [R03]；源码依据 [B1]。

<a id="ch34"></a>
# 第 34 章：`optional`、`pair`、`tuple`、`variant`——不要用一个零代表所有未知

## 34.1 optional 表示“有或没有一个值”

```cpp
std::optional<double> height;
```

height 现在没有 double 值。这与“有一个值且值为 0.0”不同。给它赋 1.3 后才处于有值状态。

<a id="ex27"></a>
**完整例程 27：`examples/27_optional_variant.cpp`**

```cpp
#include <cmath>
#include <iostream>
#include <optional>
#include <string>
#include <utility>
#include <variant>

std::optional<double> accept_height(double height)
{
    if (!std::isfinite(height) || height < 0.8 || height > 4.0) {
        return std::nullopt;
    }
    return height;
}

int main()
{
    auto first = accept_height(1.3);
    auto second = accept_height(-1.0);
    if (first) {
        std::cout << "first=" << *first << '\n';
    }
    std::cout << "fallback=" << second.value_or(0.0) << '\n';
    first.reset();
    std::cout << std::boolalpha << "has value=" << first.has_value() << '\n';
    std::pair<int, std::string> status{7, "SEARCH"};
    const auto& [id, name] = status;
    std::cout << id << ':' << name << '\n';
    std::variant<int, std::string> event = std::string("timeout");
    if (const auto* text = std::get_if<std::string>(&event)) {
        std::cout << "event=" << *text << '\n';
    }
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/27_optional_variant.cpp -o build/ex_27_optional_variant && \
  ./build/ex_27_optional_variant
```

**本次运行捕获的标准输出**：

```text
first=1.3
fallback=0
has value=false
7:SEARCH
event=timeout
```

## 34.2 每一种访问方式的责任

| 写法 | 含义 | 错误风险 |
|---|---|---|
| `opt.has_value()` / `if (opt)` | 判断是否有值 | 只说明存在，不说明新鲜或可信 |
| `*opt` | 访问所含对象 | 必须有值，不提供自动错误恢复 |
| `opt->member` | 访问所含对象的成员 | 同样必须有值 |
| `opt.value()` | 取值，空时抛异常 | 调用者要理解异常策略 |
| `opt.value_or(fallback)` | 按值获得结果或备用值 | 备用表达式作为实参会求值，不是惰性回调 |
| `opt.reset()` | 销毁所含对象并变为空 | 之前引用所含对象的引用不能再用 |
| `std::nullopt` | 明确表达无值 | 不等于 nullptr，也不是零 |

C++17 中 optional 不是用于保存引用 T& 的通用容器；需要借用时可以使用指针或明确的 reference_wrapper 等设计。本书不把后来标准加入的能力混入 C++17。

## 34.3 有值不等于“刚刚看到”

冻结的第二目标在任务中仍有值，但可能很久没有相机观测。optional<BucketTrack> 只解决存在性；时间戳、arrival、新鲜度和身份条件要另行保存。

这是 v11 把冻结任务目标的 arrival 保持为旧时刻的重要原因。不要创建一个合成目标后把当前时间当成“新视觉证明”。

## 34.4 pair 与结构化绑定

`std::pair<int,std::string>` 保存两个不同类型的值。`const auto& [id,name] = status;` 用结构化绑定给它们起本地名字。按引用绑定与按值绑定仍然不同。

tuple 可保存更多元素，但 `(id,x,y,z,diameter,confidence,stamp)` 很难长期维护。具有明确业务意义的一组数据，通常用字段具名的 struct 更清楚。

## 34.5 variant 表示“几种类型中的一种”

`std::variant<int,std::string>` 某一时刻保存其中一种类型。`std::get_if<T>(&event)` 在类型匹配时返回指针，否则返回空指针；`std::get<T>` 在错误类型时有异常语义。

它可用于类型明确的事件系统，例如导航事件、视觉事件和 ACK 事件，但初学状态机不必为所有数据引入 variant。先把普通 struct 和 enum 设计清楚。

**练习**：`optional<double>{0.0}` 与 `nullopt` 相等吗？答案：前者有值，后者无值，语义不同。

参考：C++17 `[optional]`、`[pairs]`、`[tuple]`、`[variant]`，见 [R03]、[R10]。

<a id="ch35"></a>
# 第 35 章：迭代器与半开区间——理解 `begin()`、`end()` 和失效

## 35.1 迭代器不是一定等于裸指针

迭代器提供遍历容器的统一接口。可以把它先理解成“指向某个位置的游标”，但具体类型由容器决定，不能认为所有迭代器都能任意加减。

`begin()` 指向第一个元素；`end()` 表示最后元素之后的位置。`[begin,end)` 是半开区间，包含 begin，不包含 end。空容器的 begin 等于 end。

```text
元素：      1    2    3
位置：    begin           end
                        ↑ 尾后位置，不是第四个可读元素
```

找到 end 说明没有相应元素，不能解引用。

<a id="ex28"></a>
**完整例程 28：`examples/28_iterators.cpp`**

```cpp
#include <iostream>
#include <iterator>
#include <vector>

int main()
{
    std::vector<int> values{1, 2, 3, 4};
    auto it = values.begin();
    std::cout << "first=" << *it << '\n';
    ++it;
    std::cout << "second=" << *it << '\n';
    std::cout << "distance=" << std::distance(values.begin(), values.end()) << '\n';
    for (auto cursor = values.begin(); cursor != values.end();) {
        if (*cursor % 2 == 0) {
            cursor = values.erase(cursor);
        } else {
            ++cursor;
        }
    }
    std::cout << "remaining:";
    for (int value : values) {
        std::cout << ' ' << value;
    }
    std::cout << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/28_iterators.cpp -o build/ex_28_iterators && \
  ./build/ex_28_iterators
```

**本次运行捕获的标准输出**：

```text
first=1
second=2
distance=4
remaining: 1 3
```

## 35.2 基本操作

`*it` 访问当前元素；`it->field` 访问元素字段；`++it` 前进一步；`it != end` 判断还没有越过范围。随机访问迭代器支持 `it + n` 和差值；其他类别未必支持。

`std::distance` 返回两个迭代器之间的步数，对随机访问范围可以是常数时间，对其他范围可能需要逐步走。不要把它放在大循环里并假定没有成本。

## 35.3 删除时为什么写 `it = erase(it)`

删除当前元素会使当前迭代器不能继续按原状态使用。erase 返回适合继续遍历的位置，因此典型写法是：

```cpp
for (auto it = values.begin(); it != values.end();) {
    if (should_remove(*it)) {
        it = values.erase(it);
    } else {
        ++it;
    }
}
```

循环更新部分故意留空，因为两个分支负责不同更新。删除后再无条件 ++it，会跳过某些元素，甚至超出合法范围。

## 35.4 vector 重分配导致的失效

如果 vector 追加元素时容量不足，可能重新分配元素存储。此前指向元素的指针、引用和迭代器可能全部失效。reserve 能在容量范围内降低这种情况，但不是永久有效的承诺。

如果想长期识别一个桶，保存 track ID 然后重新查找，通常比缓存 `BucketTrack*` 跨多次容器修改更稳妥。即使 ID 稳定，目标是否仍可信还要另查。

## 35.5 读 v11 的两个典型位置

pending_servo_commands_ 的处理循环使用 `it = erase(it)`；目标选择函数在排序临时候选指针时，需要确保被指向的 known_buckets_ 在该次操作期间不发生破坏性修改。当前单线程调度模型与局部执行顺序是理解这种代码的前提。

参考：C++17 `[iterator.requirements]`、`[vector.modifiers]`、`[deque.modifiers]`，见 [R03]、[R09]。

<a id="ch36"></a>
# 第 36 章：查找和判定算法——从手写循环到 `find_if`、`all_of`、`lower_bound`

## 36.1 先手写一次，再看标准算法

找 ID=2 的桶，可以从 begin 循环到 end，遇到匹配返回。`std::find_if` 把这个通用遍历过程写好了，你只需提供“什么算匹配”的谓词。

```cpp
auto found = std::find_if(tracks.begin(), tracks.end(),
    [id](const Track& track) { return track.id == id; });
if (found != tracks.end()) {
    // 现在才可以使用 found->...。
}
```

谓词就是返回条件结果的可调用对象，不要求必须是 Lambda。找到的是迭代器，不是一个新拷贝的 Track。

## 36.2 `find` 与 `find_if`

find 按相等比较寻找某个值；find_if 按自定义谓词寻找。对于整数 ID 列表可以用 find；对于包含很多字段的 Track 列表更适合 find_if。

“不在列表中”通常通过返回 end 表达，不是返回 -1。不要把不同接口的未找到语义混用。

## 36.3 any、all、none 与空集合

`any_of` 判断是否至少一个满足；`all_of` 判断是否全部满足；`none_of` 判断是否没有元素满足。空范围上 all_of 和 none_of 为 true，any_of 为 false。

这不是错误，而是逻辑定义。但如果任务要求“三个目标都稳定”，你必须先检查数量，再 all_of。不能让“没有任何目标”的空容器通过“全部稳定”的业务门禁。

## 36.4 `lower_bound` 找的是第一个不小于目标的位置

对有序值 `{1,3,5}` 查询 3，得到指向 3 的迭代器；查询 4，得到指向 5；查询 6，得到 end。它不保证找到一个恰好相等的值。

对于历史样本，可以定义比较器：

```cpp
[](const NavigationSample& sample, double stamp) {
    return sample.stamp_s < stamp;
}
```

这样返回第一个时间戳不小于查询时刻的样本。此前数据必须满足相应分区/有序要求；无序或含 NaN 的时间戳不能随便交给它。

## 36.5 用它构造插值的前后样本

先得到 upper；若 upper==begin，缺少前样本；若 upper==end，缺少后样本；否则 after=*upper、before=*(upper-1)。随后还要检查间隔、时间范围和数值合法性。

在 v11 的具体实现里，查询精确等于第一个样本时会被拒绝；等于中间或最后一个样本时，只要前一间隔合格可以接受。旧文把条件简单写成两侧严格小于，不能完整描述这个代码边界。

**练习**：在 `{0.00,0.03,0.06}` 查询 0.06，upper 是 end 吗？答案：不是，lower_bound 指向 0.06 这个实际元素。这个差别会影响边界测试。

参考：C++17 `[alg.find]`、`[alg.all.of]`、`[lower.bound]`，见 [R03]；v11 边界依据 [B1]。

<a id="ch37"></a>
# 第 37 章：排序、比较器、删除与归约——不要把容器算法当黑盒

<a id="ex29"></a>
**完整例程 29：`examples/29_algorithms.cpp`**

```cpp
#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>

struct Bucket { int id; double diameter; };

int main()
{
    std::vector<Bucket> buckets{{3, 0.25}, {1, 0.15}, {2, 0.20}};
    auto found = std::find_if(buckets.begin(), buckets.end(),
        [](const Bucket& bucket) { return bucket.id == 2; });
    if (found != buckets.end()) {
        std::cout << "found diameter=" << found->diameter << '\n';
    }
    std::sort(buckets.begin(), buckets.end(),
        [](const Bucket& left, const Bucket& right) {
            if (left.diameter != right.diameter) {
                return left.diameter < right.diameter;
            }
            return left.id < right.id;
        });
    std::cout << "selected=" << buckets[0].id << ',' << buckets[1].id << '\n';
    std::vector<int> values{1, 2, 3, 4, 5};
    values.erase(std::remove_if(values.begin(), values.end(),
        [](int value) { return value % 2 == 0; }), values.end());
    auto upper = std::lower_bound(values.begin(), values.end(), 3);
    std::cout << "lower_bound=" << *upper << '\n';
    std::cout << "sum=" << std::accumulate(values.begin(), values.end(), 0) << '\n';
    std::cout << std::boolalpha << "all positive="
              << std::all_of(values.begin(), values.end(), [](int v) { return v > 0; })
              << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/29_algorithms.cpp -o build/ex_29_algorithms && \
  ./build/ex_29_algorithms
```

**本次运行捕获的标准输出**：

```text
found diameter=0.2
selected=1,2
lower_bound=3
sum=9
all positive=true
```

## 37.1 比较器回答的是“a 是否应排在 b 前”

```cpp
[](const Bucket& a, const Bucket& b) {
    return a.diameter < b.diameter;
}
```

返回 true 表示 a 应严格排在 b 前。不能使用 `<=`，因为同一个对象与自己比较不应该被认定为“严格排在自己前面”。排序要求比较关系满足严格弱序，违反要求不是可接受的小误差。

## 37.2 相等时加确定性次关键字

直径相同时按 id 比较，可以让排序结果更可重复。`std::sort` 不保证等价元素保留原顺序；需要这种语义时选择 `stable_sort`，同时确认成本。

不要用随意的 epsilon 比较器“消除抖动”。例如三个值两两接近关系可能不具传递性，导致排序关系不一致。应先验证有限值，必要时明确量化/分桶策略，再建立合法比较顺序。

原 v11 的比较器包含自己的近等处理逻辑；教学版示例采用更直接的有限数值排序和 ID 次序，不声称两者逐行相同。

## 37.3 erase-remove 的两个阶段

```cpp
values.erase(
    std::remove_if(values.begin(), values.end(), predicate),
    values.end());
```

remove_if 把需要保留的元素移动到前部，返回新的**逻辑末尾**；它并不改变 vector 的 size。随后 erase 真正删除尾部那一段元素。

可以先拆开来理解：

```cpp
auto new_end = std::remove_if(values.begin(), values.end(), predicate);
values.erase(new_end, values.end());
```

new_end 到旧 end 之间的元素不再具有你需要的正常业务意义，不要继续拿它们做候选目标。

## 37.4 `accumulate` 的初值决定结果类型

```cpp
double total = std::accumulate(values.begin(), values.end(), 0.0);
```

如果初值写整数 0，累加器类型可能不符合小数求和需求。跟整数除法一样，左侧最后用 double 接结果，不一定能挽救中间已经按整数计算的问题。

## 37.5 其他常用算法

count_if 统计满足条件的数量；min_element/max_element 返回极值位置；transform 对每个元素计算结果写入输出范围；copy 复制范围。使用输出迭代器时要保证目标范围有效，或使用适当的 back_inserter，不要往尚未 resize 的 vector 索引中写数据。

**回到任务**：按直径排序、去掉过期轨迹、查找已投 ID、检查空间独立性，都是容器算法应用；它们的数学和内存前提必须先成立。

参考：C++17 `[alg.sorting]`、`[alg.remove]`、`[accumulate]`，见 [R03]。

<a id="ch38"></a>
# 第 38 章：map、set、哈希容器与复杂度——什么时候不用 vector

## 38.1 容器选择首先看访问模式

vector 适合顺序遍历和下标；map 适合按键有序组织；unordered_map 适合按哈希键查找；set/unordered_set 适合成员是否存在的集合关系。

对比赛中少量桶，vector 加 find_if 通常已足够清晰。复杂容器并不会自动让系统更可靠或更快，额外分配和不确定迭代顺序也要考虑。

<a id="ex30"></a>
**完整例程 30：`examples/30_maps.cpp`**

```cpp
#include <cstdint>
#include <iostream>
#include <map>
#include <string>
#include <unordered_set>

int main()
{
    std::map<int, std::string> tracks{{7, "small"}, {9, "medium"}};
    auto found = tracks.find(8);
    std::cout << std::boolalpha << "found=" << (found != tracks.end()) << '\n';
    std::cout << "before subscript=" << tracks.size() << '\n';
    std::string newly_inserted = tracks[8]; // 不存在则插入；并非只读查询。
    std::cout << "after subscript=" << tracks.size() << '\n';
    std::cout << "new value empty=" << newly_inserted.empty() << '\n';
    std::unordered_set<int> used{7};
    std::cout << "used 7=" << (used.count(7) != 0) << '\n';
    std::uint32_t axes = (1U << 0U) | (1U << 1U);
    std::cout << "roll=" << ((axes & (1U << 0U)) != 0U) << '\n';
    std::cout << "yaw=" << ((axes & (1U << 2U)) != 0U) << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/30_maps.cpp -o build/ex_30_maps && \
  ./build/ex_30_maps
```

**本次运行捕获的标准输出**：

```text
found=false
before subscript=2
after subscript=3
new value empty=true
used 7=true
roll=true
yaw=false
```

## 38.2 `map[key]` 可能修改容器

找不到 key 时，operator[] 会插入一个相应默认值。因此“用下标只读查看一个可能不存在的键”可能让容器变大。只读查询使用 find；要求存在并希望异常报告时可考虑 at。

C++17 中判断 unordered_set 是否含有某个值可以用 `count(id) != 0` 或 find；不要把 C++20 的 contains 混进默认 C++17 示例。

## 38.3 有序不等于业务合理

map 按键排序，不会自动按目标价值排序。unordered 容器的遍历顺序也不应作为“先投哪个桶”的依据。决策顺序应通过明确排序或选择函数建立。

## 38.4 复杂度只是增长趋势

典型 map 查找是对数复杂度；unordered 容器平均查找常数级，但最坏情况和哈希质量、负载等有关；vector 线性查找是线性复杂度。只有三五个对象时，清晰的数据局部性和简单逻辑可能比复杂度符号更有意义。

真正的高频成本还包括复制图像、日志格式化、动态分配、锁竞争和等待，不能只盯着一个 find_if。

## 38.5 位掩码与状态枚举再对照

`axes = 1 | 2` 表示同时选中两个独立轴；`state = State::SEARCH` 表示单一当前任务阶段。位掩码适合多个独立开关，枚举适合互斥阶段。把二者混用，会让“同时处于起飞和落地”等非法状态更难控制。

**本篇验收**：能读懂一条完整 find_if Lambda，能解释 optional 的存在性，知道 reserve 不等于 resize，知道 end 不能解引用，知道比较器不能用 `<=`。

参考：C++17 `[associative.reqmts]`、`[unord.req]`、`[map.access]`，见 [R03]。


# 第六篇：把语言知识变成可验证的工程代码

<a id="ch39"></a>
# 第 39 章：数学工具、有限值、范围检查与“安全默认值”的边界

## 39.1 `std::hypot`、`sqrt`、`abs` 各自解决什么

`std::hypot(dx,dy)` 计算平面向量长度；`std::sqrt(value)` 计算平方根；`std::abs` 对合适类型求绝对值。包含 `<cmath>` 并使用带 std 限定的合适重载，避免把 double 误交给不合适的整数接口。

hypot 的实现通常比简单 `sqrt(dx*dx+dy*dy)` 更妥善处理部分缩放问题，但不能把它当作对任意外部数据自动验证的算法。传入 NaN、无穷或已经溢出的差值，仍然没有正常测量意义。

## 39.2 三角函数与角度环绕

sin、cos 输入弧度；atan2(y,x) 结合两个分量决定角度象限；asin 的实数输入域是 [-1,1]。四元数转 pitch 前，常对计算值做范围夹紧，以处理归一化和舍入造成的微小越界。

```cpp
double angle = std::atan2(std::sin(value), std::cos(value));
```

这把角度化到与原角等价的主值范围。两次航向分别为 179° 和 -179° 时，差值应沿短方向解释为约 2°，而不是按普通减法误认为 -358°。

<a id="ex31"></a>
**完整例程 31：`examples/31_math.cpp`**

```cpp
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

constexpr double pi = 3.14159265358979323846;

double normalize_angle(double radians)
{
    return std::atan2(std::sin(radians), std::cos(radians));
}

int main()
{
    double delta = normalize_angle((-179.0 - 179.0) * pi / 180.0);
    std::cout << "short delta deg=" << delta * 180.0 / pi << '\n';
    double missing = std::numeric_limits<double>::quiet_NaN();
    std::cout << std::boolalpha << "finite=" << std::isfinite(missing) << '\n';
    double ratio = 1.2;
    if (std::isfinite(ratio)) {
        ratio = std::clamp(ratio, 0.0, 1.0);
    }
    std::cout << "clamped=" << ratio << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/31_math.cpp -o build/ex_31_math && \
  ./build/ex_31_math
```

**本次运行捕获的标准输出**：

```text
short delta deg=2
finite=false
clamped=1
```

## 39.3 `fmod` 不是始终返回正数的数学模

`std::fmod(-10.0,360.0)` 返回负的余数。要得到 [0,360) 的度数，常先取余再在负值时加 360。v11 的 normalize_degrees 就采用这种处理。

数学公式中的“取模”有不同约定，不能只凭中文名推断 C++ 函数的符号行为。

## 39.4 `min`、`max`、`clamp` 不是数据验证器

`std::clamp(x,low,high)` 表达把一个有序范围内的值限制在边界。调用前必须确保 low 不大于 high，并处理 NaN 等不符合正常排序含义的数据。

```cpp
// 更清楚的参数检查流程片段。
if (!std::isfinite(speed) || speed <= 0.0) {
    // 拒绝配置，给出原因。
} else {
    speed = std::clamp(speed, min_speed, max_speed);
}
```

“非法值自动改成某个默认值”与“拒绝非法值”是不同策略。飞行任务中，有些关键配置应明确拒绝，而不是悄悄夹紧后起飞。教程中的数学函数更多采用显式错误报告，v11 有些参数则采用夹紧和 profile 覆盖，两者区别要保留。

## 39.5 `numeric_limits`、epsilon、infinity

`numeric_limits<double>::epsilon()` 是与浮点类型有关的机器精度量，不是你飞机的测量噪声标准差。`infinity()` 可以用于搜索最小代价时的初值；`quiet_NaN()` 可以构造测试用非法数据。它们不应作为可用的实际目标坐标进入控制链。

代码中常写 `best_distance = infinity()`，随后用合法候选覆盖。若没有候选，应另有指针/optional/布尔状态表明“未找到”，不要只把 infinity 一路传到 setpoint。

## 39.6 四元数处理还需要哪些保护

先检查四个分量有限，再检查范数足够大，再归一化，再转欧拉角。只判断 norm>1e-6，不能证明所有输入都满足完整业务要求。新的教学实现可以增加检查，但必须明确这不是声称原 v11 已经包含所有检查。

## 39.7 设计可检查的数值契约

每个函数都应能回答：接受什么单位？是否允许负值？允许多大范围？失败是返回无值还是抛异常？输出是否仍要检查？对轨迹函数，还要说明 max_speed 约束的是数学 setpoint，不是实际机体速度。

**练习**：为什么 `max(0.3, user_speed)` 不能代替完整配置验证？答案：它只表达一个比较/选择策略，没有验证来源、上限、单位、NaN 和整套配置的一致性。

参考：C++17 `[c.math]`、`[alg.min.max]`、`[numeric.limits]`，见 [R03]；v11 数学与参数函数见 [B1]。

<a id="ch40"></a>
# 第 40 章：文件、流、解析与日志——读到字符串不等于读到合法数值

## 40.1 文件流是 RAII 的实际应用

`std::ofstream` 写文件，`std::ifstream` 读文件，`std::istringstream` 从内存字符串中解析。它们是不同数据源上的流接口。

<a id="ex32"></a>
**完整例程 32：`examples/32_file.cpp`**

```cpp
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>

std::optional<double> parse_number(const std::string& text)
{
    std::istringstream input(text);
    double result = 0.0;
    if (!(input >> result) || !std::isfinite(result)) {
        return std::nullopt;
    }
    input >> std::ws;
    if (!input.eof()) {
        return std::nullopt;
    }
    return result;
}

int main(int argc, char** argv)
{
    const std::filesystem::path path = argc > 1 ? argv[1] : "cpp_tutorial_report.txt";
    {
        std::ofstream output(path);
        if (!output) {
            std::cerr << "Cannot open output file.\n";
            return 1;
        }
        output << "1.3\n";
        output.close();
        if (!output) {
            std::cerr << "Write or close failed.\n";
            return 2;
        }
    }
    std::ifstream input(path);
    std::string line;
    if (!std::getline(input, line)) {
        std::cerr << "Cannot read line.\n";
        return 3;
    }
    auto parsed = parse_number(line);
    if (!parsed) {
        return 4;
    }
    std::cout << "read=" << *parsed << '\n';
    std::cout << std::boolalpha << "reject suffix=" << !parse_number("1.3m").has_value() << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/32_file.cpp -o build/ex_32_file && \
  ./build/ex_32_file
```

**本次运行捕获的标准输出**：

```text
read=1.3
reject suffix=true
```

示例默认只在当前工作目录创建 `cpp_tutorial_report.txt`，内容为教学数据。自动测试会在临时目录运行，避免覆盖已有文件。用户手工运行时，也应使用专门练习目录。

## 40.2 写文件不能只检查“构造成功”

磁盘空间、权限、文件路径和关闭时错误，都可能影响写入。例程显式 close 后再检查流状态，使错误更容易报告；析构仍作为资源收尾保障。

“调用了写入函数”与“数据已经可靠持久化到物理介质”不是同一承诺。真正高可靠记录还涉及刷新、文件系统、设备和断电策略，本书不把普通 ostream 当作完整持久化协议。

## 40.3 严格解析需要消费整段输入

`std::stod("1.3m")` 在相应调用方式下可能先解析出数字部分，因此不能只看返回值。应检查解析位置，或如例程一样用 istringstream 提取后消费空白，再确认到达输入末尾。

例程拒绝多余后缀，拒绝无法解析的输入，并检查 finite。若解析 CSV，要明确是否支持引号、逗号嵌入和转义。手工按逗号拆分的最小练习不应宣称支持所有 CSV 格式。

## 40.4 路径与当前目录

`std::filesystem::path` 是 C++17 路径工具，能组织路径而不用随意拼斜杠。但它不会自动让路径存在或获得权限。相对路径相对于进程当前工作目录，不是源文件所在目录。

文件不存在时，先打印当前目录和实际路径。不要在源码里硬编码某个队员的 `/home/name/...` 并要求所有电脑都一样。

## 40.5 标准流与 ROS 日志宏

标准输出适合小练习；ROS 的 RCLCPP_INFO/WARN/ERROR 提供带上下文的日志。原工程使用 printf 风格格式字符串，格式必须与参数类型匹配：double 常用 `%f`，size_t 常用 `%zu`，字符串 `.c_str()` 对应 `%s`。

`%s` 不能直接接收一个 std::string 对象；`%d` 也不能随意接 size_t。变参宏不会因为是 ROS 就自动修复类型不匹配。模板类型较复杂时，先保存中间变量再打印更易审查。

## 40.6 日志应该回答状态迁移原因

打印 `RELEASE failed` 太少；把每帧所有成员打印出来又太多。合适日志包括当前阶段、输入年龄、未满足门禁、请求 ID 和终止原因。高频循环要节流，但关键一次性事件应独立记录。

**练习**：修改例程，让它读取不存在的目录下的文件，确认失败返回码和错误提示，而不是静默继续使用 0.0。

参考：C++17 `[fstreams]`、`[sstream]`、`[fs]`，见 [R03]；ROS 日志接口见 [R11]。

<a id="ch41"></a>
# 第 41 章：从一个 `.cpp` 到多文件工程——头文件、ODR 与 CMake

## 41.1 `.hpp` 放接口，`.cpp` 放实现

本书配套纯 C++ 核心分成：

```text
include/cuadc_lab/core.hpp   类型、函数声明、必要模板定义
src/core.cpp                非模板函数与成员函数定义
examples/                   每个例程有自己的 main
```

`#include "cuadc_lab/core.hpp"` 让例程知道接口；链接 core.cpp 产生的库，才找到非内联函数定义。

不要直接 `#include "core.cpp"` 来逃避链接问题。这会模糊翻译单元边界，并可能在多个调用者中产生重复定义。

## 41.2 头文件保护

配套头文件使用：

```cpp
#ifndef CUADC_LAB_CORE_HPP
#define CUADC_LAB_CORE_HPP
// 声明与必要定义。
#endif
```

它防止同一翻译单元因多条 include 路径重复处理同一份定义。`#pragma once` 是常见工具链支持的替代方式，但上面的宏保护更直观地展示机制。

保护宏不能解决跨多个 `.cpp` 中出现不允许重复的函数或变量定义；那是链接与单一定义规则的问题。

## 41.3 ODR 的实用版本

普通非 inline 函数在需要时应有一个合适定义；不要在每个头文件包含者中制造不同定义。类和模板可以在满足规则时在多个翻译单元出现一致定义。

如果函数声明为 `double f(double);`，实现误写成 `double f(int)`，它们可能成为不同函数，而不是同一个函数的声明/定义。编译各文件成功后，链接仍可能失败。

## 41.4 CMake 是构建说明，不是另一种 C++

```cmake
cmake_minimum_required(VERSION 3.16)
project(cuadc_cpp_basics LANGUAGES CXX)

add_library(cuadc_core STATIC src/core.cpp)
target_compile_features(cuadc_core PUBLIC cxx_std_17)
target_include_directories(cuadc_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(module_demo examples/36_modules.cpp)
target_link_libraries(module_demo PRIVATE cuadc_core)
```

add_library 建立库目标；add_executable 建立可执行目标；target_link_libraries 描述链接依赖；PUBLIC 的包含目录和编译特性可向使用者传播，PRIVATE 只对当前目标需要。

这里 `${...}` 是 CMake 变量展开，不是 C++ 模板；库目标名也不是某个 C++ 命名空间。

<a id="ex36"></a>
**完整例程 36：`examples/36_modules.cpp`**

```cpp
#include "cuadc_lab/core.hpp"
#include <iostream>
int main()
{
    const cuadc_lab::Point3 origin{0.0, 0.0, 0.0};
    const cuadc_lab::Point3 point{3.0, 4.0, 2.0};
    std::cout << "distance_xy=" << cuadc_lab::distance_xy(origin, point) << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread -Iinclude examples/36_modules.cpp src/core.cpp src/mission.cpp -o build/ex_36_modules && \
  ./build/ex_36_modules
```

**本次运行捕获的标准输出**：

```text
distance_xy=5
```

该例不能只编译一个 `.cpp` 而忽略 core.cpp；自动插入的编译命令会包含核心源文件。

## 41.5 构建目录独立

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

-S 指源码目录，-B 指构建目录。不要把 generated 文件与自己写的头文件混在一起。ROS 的 colcon 会调用各包声明的构建系统，不能把 colcon、CMake、g++ 当成三个可随便替换的同层命令。

## 41.6 不需要一次把 v11 拆成几十个文件

先抽出纯数学和时间判定，写测试，再拆出跟踪和轨迹。接口不稳定时过早分成很多类，会增加阅读负担。拆分目标是责任清楚、可独立验证，而不是目录看起来像大型项目。

参考：[R12]、C++17 `[basic.def.odr]`、`[cpp.include]`，见 [R03]。

<a id="ch42"></a>
# 第 42 章：调试、测试、警告与 Sanitizer——怎样证明修改没有破坏旧行为

## 42.1 编译器警告不是噪声

配套工程在 GNU/Clang 下打开常用警告，还包括转换和遮蔽检查。警告指出的是值得审查的位置，不是每条都等于漏洞；但关闭所有警告通常会失去低成本的诊断机会。

`-Wconversion` 帮助发现某些可能改变值的隐式转换；`-Wshadow` 帮助发现同名遮蔽；`-Werror` 可把警告作为构建失败，适合已经整理干净的教学例程或受控 CI，不应靠它代替运行测试。

## 42.2 最小复现

当 ROS 节点里的一个排序比较器报错，把容器和比较器抽成 20 行纯 C++ 程序。这样可以判断是 C++ 类型问题，还是依赖安装、消息类型、执行器环境问题。

最小复现不只是求助材料，也是自己定位错误的方法。保留一个确定输入和期望输出，问题解决后把它变成回归测试。

## 42.3 调试器的最小用法

```bash
g++ -std=c++17 -g -O0 example.cpp -o example
gdb ./example
```

在 gdb 中可以使用 `break main`、`run`、`next`、`step`、`print variable`、`backtrace`。next 执行下一条而通常不进入函数；step 尝试进入；backtrace 查看调用链。优化编译可能使变量被消除或执行顺序难以与源码直观对应，所以初学先使用调试构建。

不要在真实飞行进程上随意打断点使它停住。先在离线程序或隔离仿真中调试。

## 42.4 `assert` 的范围

assert 适合开发期验证内部假设，但定义 NDEBUG 后常被关闭。因此不能仅靠 assert 验证真实传感器输入或飞行关键门禁。

本书核心测试使用持续有效的检查函数：失败时抛出异常并返回非零测试结果。测试用异常是为了把失败报告清楚，不是在比赛循环里靠异常实现日常控制。

## 42.5 至少覆盖五类输入

普通值、精确边界、空输入、非法数值、时间异常。比如严格插值，要测试中间时刻、精确首尾样本、无历史、重复时间、间隔过大和 NaN。只测试一条正常轨迹，很难发现危险边界。

## 42.6 Sanitizer 能做什么，不能做什么

AddressSanitizer 可帮助发现部分越界和生命周期错误；UndefinedBehaviorSanitizer 可帮助发现部分未定义行为；ThreadSanitizer 面向部分数据竞争。它们改变运行方式，有开销，也不是形式化证明。

配套工程提供 UBSan 构建选项；是否已经实际执行及结果见验证报告。ROS、飞控和相机不在本机可运行验证范围内，不能因纯算法测试通过就宣布系统可实飞。

## 42.7 测试应当可重复

不要让“等 0.5 秒”的测试真睡几秒再碰调度运气。把当前时间作为输入，手工推进到 0、100、200 毫秒，结果就可以确定。下一章开始把这种设计应用到状态机。

参考：[R02]、[R13]、[R14]。

<a id="ch43"></a>
# 第 43 章：`chrono` 从时间单位讲到时间点——为什么不能拿所有时间互相减

## 43.1 时间间隔和时间点不同

“过了 500 毫秒”是 duration；“从某个时钟得到的当前时刻”是 time_point。两个同类时钟的时间点相减得到间隔；时间点加间隔得到另一个时间点。

```cpp
using Clock = std::chrono::steady_clock;
auto start = Clock::now();
auto elapsed = Clock::now() - start;
```

不能把 `.count()` 得到的裸数字都默认当成秒。count 的单位由 duration 类型决定。

<a id="ex33"></a>
**完整例程 33：`examples/33_chrono.cpp`**

```cpp
#include <chrono>
#include <iostream>

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

int main()
{
    Clock::time_point start{}; // 教学使用合成时刻，不代表现实纪元。
    Clock::time_point current = start + 1250ms;
    auto elapsed = current - start;
    double seconds = std::chrono::duration<double>(elapsed).count();
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
    std::cout << "seconds=" << seconds << '\n';
    std::cout << "milliseconds=" << milliseconds << '\n';
    std::cout << std::boolalpha << "timeout=" << (elapsed >= 1s) << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/33_chrono.cpp -o build/ex_33_chrono && \
  ./build/ex_33_chrono
```

**本次运行捕获的标准输出**：

```text
seconds=1.25
milliseconds=1250
timeout=true
```

## 43.2 `duration<double>` 为什么能得到秒

`std::chrono::duration<double>` 默认周期单位是秒，内部数量表示采用 double。把 elapsed 转成这个类型后 count() 得到浮点秒数。

`duration_cast<milliseconds>` 则转换到毫秒间隔，若目标数量类型是整数，不能表示的小数部分会按转换规则丢失。不要用整数秒检查 0.45 秒稳定阈值。

## 43.3 `50ms` 是用户自定义字面量

`using namespace std::chrono_literals;` 让你可以写 50ms、2s 等时间字面量。它们是有类型的对象，不是普通变量名，也不是注释单位。

`using namespace` 在这里仅引入专门的字面量命名空间，和在全局引入整个 std 的影响范围不同。

## 43.4 三种时钟角色

steady_clock 用于单调经过时间，例如 timeout；system_clock 面向现实日历时间，可能因系统校时变化；ROS 的时钟还可能受模拟时间和 /clock 影响。

对消息对齐，要保证时钟域、时间戳来源和延迟含义一致。对任务“已经等了多久”，采用单调时间通常更合适。两种目的不应混为一谈。

## 43.5 统一主机时钟不等于硬件同步

v11 用 odom 回调收到时的 ROS now() 存历史，并要求视觉帧使用相应主机时间约定。这有助于避免不同设备时钟零点不一致，但 odom 到达时间仍可能晚于飞控采样，wait_for_frames 后记时也不必等于曝光中心。

因此，本版称它为“接收时间约定下的严格历史插值”，不把它夸大为相机与 IMU/GNSS 的硬件同步。只有阅读上游节点和实际日志，才能评估残余延迟。

## 43.6 为什么注入时间更好测试

把函数从 `bool update(bool ok)` 改成 `bool update(bool ok, Time now)`，算法不需要知道是真实时钟还是测试时钟。生产环境传 Clock::now()，测试传人工构造的时间点。

`Time{}` 是默认时间点，常用作教学相对原点；不应把它当作“已经有一条合法传感器观测”。存在性另用 optional。

**练习**：100 毫秒等待与时间戳 100 毫秒有什么区别？答案：前者是间隔；后者还需要说明相对于哪个时钟和原点。

参考：C++17 `[time]`、`[time.clock.steady]`，见 [R03]；ROS 时钟设计 [R15]；v11 [B1]。

<a id="ch44"></a>
# 第 44 章：异步从零——`promise`、`future`、线程、锁与 ROS 回调

## 44.1 同步、异步、并发、并行不是同义词

同步接口常让调用者等结果才继续；异步接口允许先提交工作，以后再取得结果。并发描述多个任务的推进关系；并行描述它们可能同时在不同执行资源上运行。

异步不要求每个请求创建一个新线程；单线程事件循环也能处理很多尚未完成的任务。ROS Service 的 future 更不是“飞控专门给你开了一个 C++ 线程”。

## 44.2 先用单线程完成一个 future 实验

<a id="ex34"></a>
**完整例程 34：`examples/34_future.cpp`**

```cpp
#include <chrono>
#include <future>
#include <iostream>

using namespace std::chrono_literals;

int main()
{
    std::promise<int> producer;
    std::future<int> consumer = producer.get_future();
    std::cout << std::boolalpha << "valid=" << consumer.valid() << '\n';
    for (int tick = 0; tick < 5; ++tick) {
        if (tick == 3) {
            producer.set_value(42);
        }
        if (consumer.valid() && consumer.wait_for(0ms) == std::future_status::ready) {
            std::cout << "result=" << consumer.get() << " tick=" << tick << '\n';
        } else {
            std::cout << "tick=" << tick << " no result\n";
        }
    }
    std::cout << "valid after get=" << consumer.valid() << '\n';
    std::promise<int> another;
    auto shared = another.get_future().share();
    another.set_value(7);
    std::cout << "shared reads=" << shared.get() << ',' << shared.get() << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/34_future.cpp -o build/ex_34_future && \
  ./build/ex_34_future
```

**本次运行捕获的标准输出**：

```text
valid=true
tick=0 no result
tick=1 no result
tick=2 no result
result=42 tick=3
tick=4 no result
valid after get=false
shared reads=7,7
```

promise 是结果的提供端，future 是结果的取得端。这个例程不需要后台线程：循环第 3 次时调用 set_value，future 就变为 ready。由此可以看出 future 的核心是**共享结果状态**，不是线程本身。

## 44.3 `valid()` 与 `ready` 的区别

valid 表示 future 是否关联一个共享状态；ready 表示这个共享状态的结果已准备好。一个有效 future 可以尚未完成；一个已准备好的结果也可能是异常而不是成功值。

| 操作 | 作用 | 可能阻塞吗 |
|---|---|---|
| `valid()` | 检查是否关联共享状态 | 不等待结果 |
| `wait()` | 等结果准备好 | 会等待 |
| `wait_for(duration)` | 最多等待指定时长并返回状态 | 非零时长可能等待 |
| `wait_for(0ms)` | 立即检查当前状态 | 不为结果额外等待 |
| `get()` | 取得值或重新抛出存储的异常 | 未准备好时可能等待 |

不要对无效 future 随意调用 wait/get。普通 std::future 的 get 通常只能消费一次，之后 valid 为 false；shared_future 可以多次读取同一结果，且允许复制句柄。

## 44.4 `.share()` 并不是复制两套结果

`future.share()` 把独占取得接口变成共享取得接口，并使原 future 不再持有原有取得状态。复制 shared_future 共享的是结果状态，不是启动另一次请求。

v11 中 `.async_send_request(request).future.share()` 可以拆成三步：提交请求得到包装对象；从中取普通 future；把它转成 SharedFuture 保存。包装对象还包含 request_id，不能在需要清理请求时随意丢弃。

## 44.5 future 可以保存异常

提供端可以设置异常；普通任务也可能因提供端消失而产生 broken_promise。结果 ready 只说明“有结果可读取”，读取时仍可能抛异常。

因此应把“收到响应”“响应接受”“动作完成”分成独立判断。一个 ready future 返回 response->success=false，也完全合法。

## 44.6 `std::async` 的两个容易忽略的行为

std::async 可以选择异步或延迟执行策略。默认策略不保证立即开后台线程；显式 deferred 会在相应等待/读取触发时执行。wait_for 可能返回 deferred，不是只有 ready 和 timeout 两种状态。

某些由 `std::async(std::launch::async,...)` 创建的共享状态，在最后一个关联 future 被销毁时可能等待后台任务结束。把一个临时 async future 立即丢掉，并不保证“发出去就完全不阻塞”。

ROS 客户端内部使用的 promise/future 生命周期与 std::async 的线程策略不是同一机制。不要把一个 API 的析构规则生搬到另一个上面。

## 44.7 真正多线程时，需要保护共享数据

<a id="ex35"></a>
**完整例程 35：`examples/35_thread.cpp`**

```cpp
#include <atomic>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>

int main()
{
    int counter = 0;
    std::mutex mutex;
    std::atomic<int> finished{0};
    auto work = [&]() {
        for (int i = 0; i < 1000; ++i) {
            std::lock_guard<std::mutex> lock(mutex);
            ++counter;
        }
        ++finished;
    };
    std::thread first(work);
    // 若创建第二个线程失败，必须先回收已经启动的 first。
    try {
        std::thread second(work);
        second.join();
    } catch (...) {
        first.join();
        throw;
    }
    first.join();
    std::cout << "counter=" << counter << " finished=" << finished.load() << '\n';
    auto deferred = std::async(std::launch::deferred, []() { return 6 * 7; });
    std::cout << "deferred result=" << deferred.get() << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread examples/35_thread.cpp -o build/ex_35_thread && \
  ./build/ex_35_thread
```

**本次运行捕获的标准输出**：

```text
counter=2000 finished=2
deferred result=42
```

例程启动两个线程，使用 mutex 与 lock_guard 保护 counter 的增加，最后 join 等待线程结束。finished 使用 atomic，但 counter 本身仍通过锁保护。

lock_guard 构造时持锁，离开作用域时释放，正是 RAII。join 是等待线程结束，在这个离线 main 中合理；在 20 Hz 任务 tick 里无期限 join 则会阻塞任务。

## 44.8 `atomic<bool>` 不会自动保护一个完整位姿

如果一个线程写 position.x/y/z，另一个线程读取，旁边加一个 atomic<bool> have_position 并不自动让全部位姿操作成为一致快照。仍需合适的同步协议，例如在同一 mutex 内更新/复制整个结构。

`volatile` 也不是线程安全关键字。未同步数据竞争可能产生未定义行为。shared_ptr 的引用计数安全，不等于 shared_ptr 指向的对象数据可无锁乱改。

## 44.9 ROS 中最常见的自我阻塞

假设单线程执行器正在运行 timer 回调。你在回调里发送 service，然后 while 等响应。执行器却需要先结束这个回调，才能处理那个响应，于是程序自己堵住自己。

正确结构是：本 tick 提交请求并保存 future、request_id、发送时刻；以后 tick 用零等待检查；准备好则读取；超过期限则记录失败并清理本地 pending 请求。**超时不等于远端命令未执行，也不等于撤销成功。**

## 44.10 本书采用的边界

纯 C++ 例程验证 promise/future 和共享状态概念。ROS 请求清理通过已核对的 Humble 客户端接口讲解。线程池、无锁队列、内存序、协程和硬实时设计不是初学这份任务源码的先决条件；本书会告诉你它们的边界，但不让新人在还不会引用时先实现一个调度器。

**练习**：future.valid()==true 是否足以调用 get 而不阻塞？答案：不够，还应确认准备好；而 ready 也不代表业务成功。

参考：C++17 `[futures]`、`[thread]`、`[thread.mutex]`、`[atomics]`，见 [R16]、[R03]；ROS 客户端源码 [R17]。


# 第七篇：把已经学会的 C++，组合成可测试的任务零件

前六篇回答“这一句 C++ 是什么意思”。这一篇开始回答“这些句子怎样共同解决一个问题”。仍然不接 ROS、不接飞控。这样发生错误时，不会把语言错误、网络错误和物理错误混在一起。

<a id="ch45"></a>
# 第 45 章：状态机不是 `switch` 的别名，而是跨调用保存进度

## 45.1 先从一个生活化过程理解“状态”

假设程序要完成“等待数据 → 检查数据 → 生成报告”。普通函数可以一次执行完。但飞机起飞、等待响应和连续稳定都需要经过时间，函数不能一直霸占 CPU 等待。

状态机的办法是：每次只执行当前步骤中立即能做的事情，记录进度，然后返回。下次再次调用时，从保存的进度继续。

```cpp
// 语法片段；WAIT 和 CHECK 是已经定义的 enum class 枚举值。
if (state_ == State::WAIT && have_data_) {
    state_ = State::CHECK;
}
```

state_ 必须是成员变量或其他寿命足够长的对象。如果把它写成 tick 内部每次重新初始化的局部变量，任务就永远从第一步开始。

## 45.2 三类数据必须分开命名

| 类别 | 例子 | 谁改变它 | 回答的问题 |
|---|---|---|---|
| 观测事实 | `observation.position` | 传感器回调/离线输入 | 现在在哪里？ |
| 控制目标 | `target_` | 任务决策 | 希望去哪里？ |
| 任务进度 | `state_`、`payload_` | 状态转换 | 正在做哪一步？ |

给 target_.z 赋值 1.3，并不使 observation.position.z 立刻变成 1.3。把两者混用，是模拟看似秒完成、实机却没有到达的常见根源。

## 45.3 `tick` 的输入和输出可以先写在纸上

本书离线接口采用：

```cpp
// 接口片段；完整定义在附录中的 mission.hpp。
void tick(Time now, const Observation& observation,
          std::optional<Ack> ack = std::nullopt);
```

Time 是时间点；Observation 是当前观测；optional<Ack> 允许这一轮没有响应。三个输入足以让纯 C++ 程序决定下一步，而不需要知道数据是 ROS 发来的还是测试代码生成的。

输出则分为“当前目标”和“本轮新增的一次性请求”。持续目标可以反复读取；一次性请求不能每一轮重复发送。

## 45.4 `enter` 的最小职责

统一状态转换函数，至少更新 state_、entered_，并清除本阶段的稳定计时器。这样“进入状态”是明确事件，而不是散落在几十个函数中的赋值。

不要在 enter 里无条件清除全部变量。比如切换状态时清掉已经选定的第二目标，反而破坏任务。应按生命周期逐项决定：哪些属于整场任务，哪些属于一瓶，哪些只属于一个阶段。

## 45.5 一次 tick 只迁移一个状态，便于推演

本书离线任务使用 switch，状态改变后结束该 case，不在同一轮继续穿透下一个状态。这个策略牺牲的通常只是一个教学 tick，换来明确的可测试顺序。

正式工程也可以允许受限的同轮多次迁移，但必须明确上限与退出条件，不能写成 while 一直转换到“看起来不能再转换”为止。

## 45.6 决策的优先级比 case 排列更重要

先检查输入是否有限、是否新鲜，再处理任务。一次数据严重异常不应被较低优先级的“已到航点”掩盖。

终止状态也应具有吸收性：一旦 STOP/DONE，不应因迟到 ACK 又开始第二瓶。这不是 enum 自动提供的保证，必须在代码入口写出来并测试。

**练习**：如果在 WAIT_RELEASE 中每轮都执行 `send()`，会发生什么？答案：同一个任务意图产生多个独立请求，可能形成重复执行和错配响应；需要单独的“进入时发送”和“等待时检查”。

<a id="ch46"></a>
# 第 46 章：连续稳定门——把布尔值、时间和 `optional` 串起来

## 46.1 问题定义要比代码先出现

要求：某个条件连续为真 200 ms，才返回 true。期间出现 false，重新计时。教学实现另外要求两次检查的间隔不超过 100 ms，否则不能声称中间一直稳定。

这个“最大检查间隔”是本书的教学补充，不是对 v11 原函数的逐字复刻。v11 多处采用 optional 的 stable_since 形式，但不同调用点的健康检查仍要分别审查。

## 46.2 为什么需要两个可选时间点

since_ 表示本次连续满足从何时开始；last_ 表示上次检查何时发生。它们开始都没有值，因此用 optional，而不是用“时间点等于零”暗示未初始化。

执行顺序是：检查回跳/大间隔 → 更新 last_ → 若条件 false 清 since_ → 若首次 true 记录 since_ → 比较持续时间。

## 46.3 手算一次状态变化

| 调用时间 | 条件 | since_ | 返回值 |
|---:|---|---|---|
| 0 ms | true | 0 ms | false |
| 50 ms | true | 0 ms | false |
| 100 ms | true | 0 ms | false |
| 200 ms | true | 0 ms | true |
| 250 ms | false | 无值 | false |
| 300 ms | true | 300 ms | false |
| 1000 ms | true | 1000 ms | false，间隔过大 |

每一行是在解释一个函数调用后的成员变量，而不是在描述函数内部睡眠。

## 46.4 完整实践

<a id="ex37"></a>
**完整例程 37：`examples/37_stable_gate.cpp`**

```cpp
#include "cuadc_lab/core.hpp"
#include <iostream>
int main()
{
    using namespace cuadc_lab;
    StableGate gate{Milliseconds(200), Milliseconds(100)};
    std::cout << std::boolalpha;
    for (long long ms : {0LL, 50LL, 100LL, 150LL, 200LL}) {
        std::cout << ms << "ms -> " << gate.update(true, at_ms(ms)) << '\n';
    }
    std::cout << "bad sample -> " << gate.update(false, at_ms(250)) << '\n';
    std::cout << "restart -> " << gate.update(true, at_ms(300)) << '\n';
    std::cout << "large gap -> " << gate.update(true, at_ms(1000)) << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread -Iinclude examples/37_stable_gate.cpp src/core.cpp src/mission.cpp -o build/ex_37_stable_gate && \
  ./build/ex_37_stable_gate
```

**本次运行捕获的标准输出**：

```text
0ms -> false
50ms -> false
100ms -> false
150ms -> false
200ms -> true
bad sample -> false
restart -> false
large gap -> false
```

类的完整实现收录在附录的 core.hpp/core.cpp。先根据上表自己写，再比较实现。构造参数是 duration 对象，不是两个没有单位的 double。

## 46.5 本模块不能替你检查传感器新鲜度

如果任务线程每 50 ms 调用 update(true)，但是那个 true 是十秒前的旧 odom 算出来的，StableGate 仍可能通过。因为它只看传入布尔值和调用时间。

正确的候选条件应包括新鲜度，例如 `fresh && geometry_ok && motion_ok`。更严格的实现还需要检查观测时间是否推进。软件轮询足够快不等于新数据足够快。

## 46.6 为什么测试中注入时间，比真的睡两秒更好

使用 `at_ms(0)`、`at_ms(50)` 可以立即测试临界点、重复时间、回跳和大空洞，结果可重复。真实 wall clock 容易受系统调度影响，不适合用来验证精确边界。

注入时间改变的是模块的可测试性，不是伪造实测。附录报告会明确写这些是合成时间测试。

<a id="ch47"></a>
# 第 47 章：从 `Point3` 到投放口几何，先理解“数据代表什么”

## 47.1 位置与向量长得一样，但不是同一概念

Point3 可以装位置，也可以装位移。`{0.026,-0.065,-0.32}` 在本工程里是从机体参考点指向投放口的机体系位移，不是世界坐标中的一个点。

一个世界点的位置 = 世界中的机体原点位置 + 已转换到世界方向的相对位移。

```text
机体系位移 r_B --旋转R_WB--> 世界系位移 R_WB*r_B
                                           |
机体世界位置 p_WB -------------------------加--> 投放口世界位置
```

这份课本使用同一个 Point3 保持入门代码简洁。进一步工程化可以引入 BodyVector、WorldPoint 等不同类型，防止把坐标系混着加。

## 47.2 `Rz * Ry * Rx` 应怎样准确理解

这里采用列向量以及主动旋转的数值约定：

\[
v_W=R_z(\psi)R_y(\theta)R_x(\phi)v_B.
\]

计算从右往左：先执行 Rx，再 Ry，再 Rz。不要把这个展开过程与“绕新轴/旧轴”的口头说法混用。欧拉角的内禀/外禀解释有等价关系，但必须先说明约定；代码只需要保持矩阵与输入姿态一致。

以绕 X 轴为例：x 不变，y/z 在平面内旋转。

\[
x_1=x,\quad y_1=\cos\phi\,y-\sin\phi\,z,\quad
z_1=\sin\phi\,y+\cos\phi\,z.
\]

再绕 Y：

\[
x_2=\cos\theta\,x_1+\sin\theta\,z_1,\quad
 y_2=y_1,\quad z_2=-\sin\theta\,x_1+\cos\theta\,z_1.
\]

最后绕 Z：

\[
x_W=\cos\psi\,x_2-\sin\psi\,y_2,\quad
 y_W=\sin\psi\,x_2+\cos\psi\,y_2,\quad z_W=z_2.
\]

core.cpp 中 xr/yr/zr、xp/yp/zp 对应这些中间变量。把中间量单独命名，比写成一个无法检查的长表达式更适合新人。

## 47.3 先通过三个简单单元测试，再代入真实机构

零姿态应保持向量不变；yaw=90° 时 `(1,0,0)` 应变为 `(0,1,0)`；roll=90° 时 `(0,1,0)` 应变为 `(0,0,1)`。这些数值测试比看复杂轨迹“好像方向对”更有诊断力。

旋转还应保持向量长度。我们用普通实数精度容差比较，而不是要求浮点结果逐位完全相等。

## 47.4 反算飞机参考点，而不是让参考点直接到桶心

已知桶的世界坐标 b，投放口机体系偏置 r，当前姿态 R，则：

\[
p_{release}=p_{vehicle}+Rr.
\]

只要求水平对准，可反算：

\[
p_{vehicle,x}=b_x-(Rr)_x,\quad
p_{vehicle,y}=b_y-(Rr)_y.
\]

本工程高度另行指定，所以 z 不使用 `b_z-(Rr)_z`，而是任务起点高度加设定飞行高度。公式的 XY 对准与真实离地高度必须分开。

## 47.5 完整实践

<a id="ex38"></a>
**完整例程 38：`examples/38_geometry.cpp`**

```cpp
#include "cuadc_lab/core.hpp"
#include <iomanip>
#include <iostream>
int main()
{
    using namespace cuadc_lab;
    const Point3 bucket{32.0, 1.0, 0.3};
    const Point3 outlet_body{0.026, -0.065, -0.32};
    const Attitude attitude{radians(8.0), radians(-3.0), radians(30.0)};
    const Point3 vehicle = desired_vehicle(bucket, attitude, outlet_body, 1.3);
    const Point3 outlet = release_point(vehicle, attitude, outlet_body);
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "vehicle=" << vehicle.x << ',' << vehicle.y << ',' << vehicle.z << '\n';
    std::cout << "outlet=" << outlet.x << ',' << outlet.y << ',' << outlet.z << '\n';
    std::cout << "xy_error=" << distance_xy(outlet, bucket) << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread -Iinclude examples/38_geometry.cpp src/core.cpp src/mission.cpp -o build/ex_38_geometry && \
  ./build/ex_38_geometry
```

**本次运行捕获的标准输出**：

```text
vehicle=31.952826,0.995664,1.300000
outlet=32.000000,1.000000,0.975875
xy_error=0.000000
```

输入是固定合成姿态，不是飞控实测。输出 xy_error 接近零说明反解公式与正向公式自洽，不能证明相机外参、EKF 参考点或水瓶真实落点准确。

## 47.6 这里复习了哪些 C++

Point3 和 Attitude 是聚合类型；`const auto rotated` 是局部只读计算结果；函数参数 `const Point3&` 只借用；返回 Point3 是值返回；中间三角函数来自 cmath；有限值检查通过 std::isfinite 完成。

这就是“基础知识→工程代码”的连接：不是突然出现一个高深函数，而是把已经掌握的语言工具应用到明确公式。

<a id="ch48"></a>
# 第 48 章：轨迹函数，按公式一步一步实现

## 48.1 先区分路径、时间参数和真实飞行

路径回答走哪条线；时间参数回答目标点沿线如何移动；飞控跟踪回答真实飞机能否跟上。三者不能用同一个“速度”概念替代。

给两个端点 a、b，以及进度 s∈[0,1]，直线位置是：

\[
p=a+s(b-a).
\]

剩下的问题只是怎样由时间 t 算出 s。

## 48.2 归一化时间

令航段总时间为 T，u=clamp(t/T,0,1)。u 没有单位；t、T 必须采用同一单位。T≤0 或非有限时，教学函数明确拒绝。

规定前20%时间加速，中间60%匀速，后20%减速。归一化速度图的面积必须为1，才能从 s=0 到 s=1。

设归一化峰值速度 k，那么面积为：

\[
\tfrac12(0.2)k+0.6k+\tfrac12(0.2)k=0.8k=1,
\]

所以 k=1.25。

## 48.3 对速度积分，得到三个位置公式

前段速度由0线性增至1.25，所以速度为6.25u；积分后：

\[
s(u)=3.125u^2,\quad 0\le u<0.2.
\]

中段从 s(0.2)=0.125 起步，以1.25持续增加：

\[
s(u)=0.125+1.25(u-0.2)=1.25u-0.125.
\]

后段可以用“距离终点还剩多少”的对称公式：

\[
s(u)=1-3.125(1-u)^2,\quad 0.8<u\le1.
\]

边界两侧位置和速度连续，但加速度在分段点会跳变。它不是最小 jerk 或完整动力学最优轨迹。

## 48.4 为什么 T=距离/(0.8×峰值速度)

若路程 L，实际目标峰值速度是1.25L/T。让它不超过 v_max，则 T≥1.25L/v_max=L/(0.8v_max)。再考虑最短航段时间，取两者最大值。

例如 L=10 m、v_max=2 m/s，得到 T=6.25 s。这是目标轨迹时间，不是实机保证的到达时间。

## 48.5 完整实践与输出

<a id="ex39"></a>
**完整例程 39：`examples/39_trajectory.cpp`**

```cpp
#include "cuadc_lab/core.hpp"
#include <iomanip>
#include <iostream>
int main()
{
    using namespace cuadc_lab;
    const Segment segment = make_segment({0, 0, 0}, {10, 0, 0}, 2.0);
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "duration=" << segment.duration_s << '\n';
    for (double u : {0.0, 0.1, 0.2, 0.5, 0.8, 0.9, 1.0}) {
        const Point3 point = segment.sample(u * segment.duration_s);
        std::cout << "u=" << u << " x=" << point.x << '\n';
    }
    const Point3 next = slew({0, 0, 0}, {1, 0, 0}, 0.25, 0.05);
    std::cout << "slew_step=" << next.x << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread -Iinclude examples/39_trajectory.cpp src/core.cpp src/mission.cpp -o build/ex_39_trajectory && \
  ./build/ex_39_trajectory
```

**本次运行捕获的标准输出**：

```text
duration=6.250
u=0.000 x=0.000
u=0.100 x=0.312
u=0.200 x=1.250
u=0.500 x=5.000
u=0.800 x=8.750
u=0.900 x=9.688
u=1.000 x=10.000
slew_step=0.013
```

额外测试检查 u=0.2/0.8 的连接、负时间、超过T、零距离、参数非法以及1000个采样点的单调性和峰值速度。纯几何函数可以在接飞控之前充分测试。

## 48.6 动态目标为什么用 slew，而不是反复重新建整段

对准过程中视觉目标可能修正，飞机姿态也在变。每一轮重新把时间归零，可能造成轨迹永远处于加速起点。

slew 则比较上次目标与新期望目标的距离，允许的最大步长是 v_max×dt；不足一步就直接到期望，否则沿连线前进最大步长。

本书教学函数显式接收 dt。**v11 也不是一直死用0.05 s**：它默认0.05，随后测量相邻调用时间，只有测得的 dt 有限且位于代码允许范围时采用，否则回退到默认值。具体行为要看 `slew_target_toward`，不能从“定时器50ms”直接推出永远准确50ms。

## 48.7 对12 m/s提案的边界说明

本教程不把增大轨迹峰值当作达到理论最快。若真实最大可用制动加速度为 a，忽略 jerk 等因素时，12 m/s 到0仍需约 v²/(2a) 的制动距离。轨迹和场地必须匹配，不能拿一个小容差圆去抵消动力学约束。

这里是数学推理与风险边界，不是新飞控参数建议，也不是已经完成的 O4 改版。

<a id="ch49"></a>
# 第 49 章：导航历史与时间插值，把容器和算法真正用起来

## 49.1 一个样本需要一起保存什么

只保存 position，不保存 timestamp，就无法判断它对应哪一帧图像。只保存 yaw，不保存坐标，也无法完成位姿转换。于是定义 NavigationSample 聚合结构，把同一个时刻的量放在一起。

本书纯 C++ 练习用“实验起点以来的秒数”double。v11 用 rclcpp::Time。这个简化是为了隔离学习算法，并不允许真实系统把两个时钟域的double直接混用。

## 49.2 在排序历史中找右邻居

```cpp
// 算法片段，history 已按 stamp_s 严格递增。
auto upper = std::lower_bound(
    history.begin(), history.end(), stamp_s,
    [](const NavigationSample& sample, double value) {
        return sample.stamp_s < value;
    });
```

谓词告诉算法：某个 sample 是否仍在查询值左侧。lower_bound 返回第一个“不在左侧”的样本，也就是时间≥查询时间的第一项。

随后必须先排除 begin/end，才可读取 `*upper` 和 `*(upper-1)`。这一段同时用到了 iterator、Lambda、范围前提和 early return。

## 49.3 原实现的等号边界，不能用一句“严格夹住”含糊带过

v11 拒绝 upper==begin 或 upper==end。因而在正常递增历史中，接受区间实际满足 `before.time < t <= after.time`。

如果t恰好是第一项时间，会被拒绝，因为找不到更早样本。如果t恰好是中间项或末项，可以用前一项与这一项进行ratio=1插值。这里不是两端都严格小于。

还必须检查相邻样本间距不超过上限。即使查询时刻恰好在两个样本之间，也不代表跨越一秒空洞的插值可信。

## 49.4 位置和角度不能完全用同一个差值

位置用 `a + ratio*(b-a)`。角度如果从179°到-179°，直接 b-a 得到-358°，会绕远路。应先把差归一化为2°，插值后再归一化结果。

这是对当前小运动欧拉角插值策略的解释，不是声称欧拉线性插值在任意大角运动中优于四元数插值。

## 49.5 完整实践

<a id="ex40"></a>
**完整例程 40：`examples/40_history.cpp`**

```cpp
#include "cuadc_lab/core.hpp"
#include <deque>
#include <iomanip>
#include <iostream>
int main()
{
    using namespace cuadc_lab;
    const std::deque<NavigationSample> history{
        {0.000, {0.000, 0, 0}, {0, 0, radians(179)}},
        {0.040, {0.040, 0, 0}, {0, 0, radians(-179)}},
        {0.080, {0.080, 0, 0}, {0, 0, radians(-177)}}
    };
    std::cout << std::boolalpha << std::fixed << std::setprecision(3);
    const auto middle = interpolate_navigation(history, 0.020);
    if (middle) {
        std::cout << "x=" << middle->position.x << " yaw_deg="
                  << middle->attitude.yaw * 180.0 / pi << '\n';
    }
    std::cout << "first=" << interpolate_navigation(history, 0.0).has_value() << '\n';
    std::cout << "last=" << interpolate_navigation(history, 0.08).has_value() << '\n';
    std::cout << "future=" << interpolate_navigation(history, 0.09).has_value() << '\n';
    std::cout << "gap_reject=" << !interpolate_navigation(history, 0.02, 0.03) << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread -Iinclude examples/40_history.cpp src/core.cpp src/mission.cpp -o build/ex_40_history && \
  ./build/ex_40_history
```

**本次运行捕获的标准输出**：

```text
x=0.020 yaw_deg=180.000
first=false
last=true
future=false
gap_reject=true
```

请分别测试空队列、一项、首样本、末样本、未来时刻、重复时间和乱序。教学版本每次查询先检查整个历史是否有序，便于发现错误，因此额外花费O(N)；正式系统通常在入队时维护不变量，查询无需重复全表验证。

## 49.6 pending队列为何必须先 move，再 pop

v11 的模式类似：

```cpp
// 源码模式讲解；PendingVisionFrame 已在原工程定义。
PendingVisionFrame frame = std::move(pending_vision_frames_.front());
pending_vision_frames_.pop_front();
```

先把首项资源移入局部 frame，再销毁队列首项。此后 frame 是独立对象，可继续处理。若只保存 `const auto& frame=front()` 再pop，引用就指向已销毁对象。

而“暂时没有后一帧导航”时直接 return，保留队首等待后续odom到来；等待过期则弹出并丢弃。return与continue在这里分别表示“本轮先别往后处理”和“这一帧结束，继续下一帧”，语义不同。

## 49.7 时间戳的来源限制

原v11在odom回调中用主机ROS接收时刻为历史打戳，没有直接使用消息的飞控采样时间戳。它意在避免时钟偏差，但接收时间也包含传输和调度延迟。

所以应称为**同一主机时钟域中的时间夹逼与插值**。除非另外完成了硬件采样时刻的校准和验证，不能把它描述成已消除全部视觉/导航时间误差的硬件同步。[B1]

<a id="ch50"></a>
# 第 50 章：跟踪与目标冻结——从“一次测量”变成“任务对象”

## 50.1 三种对象，不要合成一个大struct

Detection：这一帧测到了什么。Track：多帧被认为属于同一物体的历史估计。Plan：本次任务决定处理哪些目标及顺序。

检测可能消失，Track可能更新，Plan则可以冻结。如果把Plan存成vector内部元素裸指针，容器扩容或删除时可能失效；返回值快照是更容易理解的入门实现。

## 50.2 关联不是只判断最近距离

教学版本先检查位置差≤0.45 m、直径差≤0.08 m，才允许形成候选配对。代价是两种差分别除以门限后相加。

枚举所有候选后，按cost升序排列，逐个接受尚未占用的track和detection。这是贪心一对一关联，不是全局最优分配算法。明确方法的边界比起一个高级名字更重要。

## 50.3 为什么先建立索引，再更新容器

候选 Association 保存track索引与detection索引。处理已有轨迹时不新增track，最后再为未匹配检测push_back。这样避免处理中容器扩容、used标记长度和索引范围混乱。

不要在range-for遍历vector的同时向同一个vector追加元素。即便原来的元素引用看起来还可访问，也不能假设范围循环保存的迭代器一直有效。

## 50.4 EMA不是“取平均”四个字就足够

设旧值3.0，新测量3.2，alpha=.25，则新估计3.0+.25×.2=3.05。这个更新可写为 `old += alpha*(measurement-old)`，是复合赋值与普通公式等价的例子。

每一帧不断混合之前的状态，所以并非最近四帧简单均值。采样频率变化也会改变相同alpha对应的时间响应。要做固定时间常数滤波，可以根据dt另行计算alpha，但那属于扩展，不是v11原实现。

## 50.5 中值/MAD和位置偏差的来源要分开

中值把极端值的影响压低；MAD是相对中值的绝对偏差中值。v11对直径用了窗口中值、EMA与MAD，对位置采用EMA和残差平滑。不能统称“全部用了卡尔曼滤波”。

教学Tracker为了让关联流程短而可读，仅用EMA，median/mad在独立函数中演示；未完整复制v11置信度过滤、已投目标排除和任务ID重绑定。这个取舍在代码旁明确列出，不伪装成正式替换节点。

## 50.6 三桶选二是两次不同排序

第一件事是选哪两个：在可靠、空间独立的候选中，直径升序，取最小两个。

第二件事是先处理哪个：仅对这两个按距飞机的位置升序，近的先处理。不能先取最近两个再按大小排，因为这样可能把更小但稍远的桶丢掉。

## 50.7 完整实践

<a id="ex41"></a>
**完整例程 41：`examples/41_tracking.cpp`**

```cpp
#include "cuadc_lab/core.hpp"
#include <iomanip>
#include <iostream>
#include <vector>
int main()
{
    using namespace cuadc_lab;
    Tracker tracker;
    std::vector<Detection> frame{
        {{1, 0, 0}, 0.15}, {{3, 0, 0}, 0.20}, {{5, 0, 0}, 0.25}
    };
    for (int index = 0; index < 3; ++index) {
        tracker.update(frame, static_cast<double>(index) * 0.1);
    }
    const auto plan = tracker.select_two({4, 0, 1.3}, 0.2, false);
    std::cout << "tracks=" << tracker.tracks().size() << " plan=" << plan.size() << '\n';
    for (const auto& target : plan) {
        std::cout << "id=" << target.id << " diameter=" << target.diameter << '\n';
    }
    frame[1].position.x = 3.2;
    tracker.update(frame, 0.3);
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "live_x=" << tracker.tracks()[1].position.x
              << " snapshot_x=" << plan[0].position.x << '\n';
    std::cout << "median=" << median({0.2, 0.21, 0.19, 0.7, 0.2})
              << " mad=" << mad({0.2, 0.21, 0.19, 0.7, 0.2}) << '\n';
    return 0;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread -Iinclude examples/41_tracking.cpp src/core.cpp src/mission.cpp -o build/ex_41_tracking && \
  ./build/ex_41_tracking
```

**本次运行捕获的标准输出**：

```text
tracks=3 plan=2
id=2 diameter=0.2
id=1 diameter=0.15
live_x=3.05 snapshot_x=3.00
median=0.20 mad=0.01
```

运行结果应显示选出的第一个任务目标并不一定是最小桶，而可能是两个已选桶中较近的那个。最后live_x变化而snapshot_x不变，直观展示值复制、引用与任务冻结的区别。

## 50.8 “新鲜的任务记忆”不是“新的视觉帧”

冻结计划的保存时刻，不能替代目标的实际观测时刻。原v11第二目标记忆把arrival保留为很旧的纪元值，就是避免合成记忆被误认成新视觉。

这也说明变量名和时间字段的语义必须写清：created_at、received_at、captured_at、last_seen不能随便互换。[B1]

<a id="ch51"></a>
# 第 51 章：阶段大实践——完整但不接飞机的任务状态机

## 51.1 本练习完整在哪里，不完整在哪里

完整的是程序结构：构造→输入观测→生成目标→持续稳定→发一次请求→等ACK→保持动作时间→收回ACK→下一目标→终止。正常和丢ACK两种分支都可以实际运行。

不包含真实飞控、解锁、MAVROS、相机、载荷机构、避障和实机动力学。RECON只是一个教学航点，不代表读取任何危险品标识；DONE只是本实验协议结束，不是裁判意义的任务成功。

## 51.2 工程文件之间的关系

```text
include/cuadc_lab/core.hpp      几何、轨迹、StableGate、跟踪的声明
src/core.cpp                   上述实现
include/cuadc_lab/mission.hpp   LabMission、Observation、Request、Ack
src/mission.cpp                状态机实现
examples/42_mission.cpp         构造假数据、调用状态机、打印结果
```

这次不再把所有实现放进main。main负责搭建实验环境，LabMission负责业务决策，core负责可重用纯算法。完整五个文件在配套工程和附录中都能找到。

## 51.3 请求编号解决什么问题

一条Request包含id、command、payload。Ack必须带回对应id。等待STOW时收到上一个RELEASE的ACK，不能当成STOW完成。

本模拟只允许一个活动请求，用optional<Request>保存。发送后不再重发；超时转STOP，并写“physical action unknown”。这是响应不确定性的演示，不是教真机用STOP代替受控降落。

## 51.4 主程序完整代码

<a id="ex42"></a>
**完整例程 42：`examples/42_mission.cpp`**

```cpp
#include "cuadc_lab/mission.hpp"
#include <cmath>
#include <iostream>
#include <optional>
#include <string>

int main(int argc, char** argv)
{
    using namespace cuadc_lab;
    const bool drop_ack = argc == 2 && std::string(argv[1]) == "--drop-ack";
    LabMission mission{at_ms(0)};
    Point3 position{};
    std::optional<Request> queued;
    long long reply_at = 0;
    LabState previous = mission.state();
    std::cout << name(previous) << '\n';
    // 每次迭代推进100ms模拟时间，没有真实sleep、串口、网络和飞机。
    for (long long ms = 0; ms <= 60000; ms += 100) {
        const Point3 before = position;
        position = slew(position, mission.target(), 1.0, 0.1);
        Observation obs{at_ms(ms), position,
            std::hypot(position.x - before.x, position.y - before.y) / 0.1,
            (position.z - before.z) / 0.1, true};
        std::optional<Ack> ack;
        if (queued && ms >= reply_at) {
            if (!drop_ack) { ack = Ack{queued->id, true}; }
            queued.reset();
        }
        mission.tick(at_ms(ms), obs, ack);
        if (auto request = mission.take_request()) {
            queued = request;
            reply_at = ms + 200;
        }
        if (mission.state() != previous) {
            std::cout << name(previous) << " -> " << name(mission.state()) << '\n';
            previous = mission.state();
        }
        if (mission.state() == LabState::DONE || mission.state() == LabState::STOP) {
            std::cout << "completed=" << mission.completed_payloads() << '\n';
            if (!mission.stop_reason().empty()) {
                std::cout << "reason=" << mission.stop_reason() << '\n';
            }
            const bool expected = drop_ack ? mission.state() == LabState::STOP :
                mission.state() == LabState::DONE && mission.completed_payloads() == 2;
            return expected ? 0 : 1;
        }
    }
    std::cerr << "simulation did not finish\n";
    return 2;
}
```

**编译与运行**（在配套工程根目录执行；只复制单文件时须相应调整路径）：

```bash
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -pthread -Iinclude examples/42_mission.cpp src/core.cpp src/mission.cpp -o build/ex_42_mission && \
  ./build/ex_42_mission
```

**本次运行捕获的标准输出**：

```text
WAIT
WAIT -> TAKEOFF
TAKEOFF -> COARSE
COARSE -> FINE
FINE -> WAIT_RELEASE
WAIT_RELEASE -> RELEASE_HOLD
RELEASE_HOLD -> WAIT_STOW
WAIT_STOW -> COARSE
COARSE -> FINE
FINE -> WAIT_RELEASE
WAIT_RELEASE -> RELEASE_HOLD
RELEASE_HOLD -> WAIT_STOW
WAIT_STOW -> RECON
RECON -> RETURN
RETURN -> LAND
LAND -> DONE
completed=2
```

逐段阅读：先建立mission；每轮模拟时间增加100ms；用slew生成新的假位置；从位置差估计速度；达到模拟响应时间时构造Ack；调用tick；取走这一轮新请求；状态变化才打印；终态退出。

注意：这个假运动模型直接追踪目标，不考虑惯性、推力和风。由它打印的时间不能用来估算CUADC实飞耗时。

## 51.5 故障注入只改一个输入条件

```bash
./build/ex_42_mission --drop-ack
```

正常运行应处理两个目标并到DONE；丢ACK应在第一条请求超时后到STOP，completed=0，且迟到响应不能重新启动任务。

关闭ACK并没有让模拟进程卡死。它仍逐轮检查时间并更新状态，正是非阻塞设计的意义。

## 51.6 为什么实验稳定时间是200ms，不是原任务500ms

为便于小型合成测试，LabMission采用200ms教学稳定门、小尺度航点和简化误差条件。真实v11的高度、门限和状态逻辑在后面源码章节另行解读。两者不能直接当作同一套参数。

## 51.7 你现在应该能够独立修改的内容

把两个目标改成三个，先更新容器和终止条件，再补对应测试；把ACK延迟改成1200ms，解释为何超时；让观测时间停止推进，检查新鲜度；给观测位置一个NaN，确认没有把它变成正常飞行目标。

练习答案不是“看到了报错就行”，而是说明哪条契约失败、哪个状态迁移发生、哪些变量没有被错误累加。


# 第八篇：现在才进入 ROS 2——把框架调用读回普通 C++

本篇不是再次写一整本ROS教材，而是把C++知识接到ROS2 Humble的具体写法。网络发现、复杂QoS配置和硬件部署可继续阅读项目的ROS教程；这里重点解决“我知道ROS概念，但看不懂这一长行C++”的问题。

<a id="ch52"></a>
# 第 52 章：一个 ROS 节点，究竟何时构造、何时运行

## 52.1 三个不同的名称

`HelloNode` 是C++类名；`cpp_hello` 是运行时ROS节点名；`hello_node` 是编译出的可执行程序名。它们可以不同。

类是类型，节点对象是该类型的实例，可执行程序是操作系统加载的文件。一个进程里可以有多个节点；不要把这三个概念都叫“节点文件”。

## 52.2 先看完整、只有一个功能的节点

**文件：`ros2_ws/src/cpp_tutorial/src/hello_node.cpp`**

```cpp
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

class HelloNode final : public rclcpp::Node {
public:
    HelloNode() : Node("cpp_hello")
    {
        timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {
            RCLCPP_INFO(get_logger(), "C++ timer callback");
        });
    }
private:
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HelloNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

这个文件只创建500ms定时回调，不读取或控制飞控。ROS练习未在本次编写环境中编译或运行；其API依据Humble官方资料与源码核对，需在你们安装了Humble的机器上验证。[R18]

## 52.3 按真实执行顺序阅读

程序先进入main，rclcpp::init处理ROS上下文和参数；make_shared分配并构造HelloNode；构造HelloNode之前，先构造Node基类；构造函数体创建timer；构造完成后spin进入执行器循环；到期的timer回调才有机会执行。

所以“构造时登记一个回调”不等于“构造时一直运行回调”。一旦这个区别明确，ROS节点就不再像魔法。

## 52.4 把这一句拆成五部分

```cpp
// 语法片段，与上方完整节点相同的模式。
timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {
    RCLCPP_INFO(get_logger(), "C++ timer callback");
});
```

第一，timer_是成员变量，保存定时器资源的共享指针。第二，create_wall_timer是继承来的成员函数，不是语言关键字。第三，milliseconds(500)构造时间间隔对象。第四，`[this]{...}`构造一个保存当前对象地址的可调用对象。第五，调用函数返回后才把返回的资源句柄赋给timer_。

这里涉及赋值、函数调用、duration、Lambda、this、智能指针，前面各篇已经分别解释。

## 52.5 为什么timer不能只是随手创建的局部临时对象

资源的寿命必须覆盖后续执行期。把创建结果保存为成员，是清晰表达“节点活着时希望保有这个timer”的做法。

`[this]`不会延长节点寿命。这个简单单线程程序让node共享指针覆盖spin全过程，退出spin后不再并发执行回调，再销毁对象。若自行增加后台线程，就需要另外设计停止、等待和销毁顺序。

## 52.6 Node基类不是自己写的 `void tick()`

继承Node后，能调用create_publisher、get_logger、declare_parameter等框架成员；但Node不会自动发现你恰好有一个叫tick的函数。必须显式把tick绑定为回调。

这也是为什么函数名可以换，绑定关系才决定实际调用哪个函数。

<a id="ch53"></a>
# 第 53 章：消息、发布和订阅的类型，逐层解开

## 53.1 从最外层向内读长类型

```cpp
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
```

`nav_msgs::msg::Odometry`是消息类型；`rclcpp::Subscription<该类型>`是模板实例化得到的订阅者类型；最后的`::SharedPtr`是这个类型内部的共享指针别名；subscription_是变量名。

改写为别名后意思不变：

```cpp
// 类型别名片段。
using Odom = nav_msgs::msg::Odometry;
using OdomSubscription = rclcpp::Subscription<Odom>;
OdomSubscription::SharedPtr subscription_;
```

这不是复制对象，只是给类型另起名字。不要把`::`误读成“拿到消息里的某个成员字段”，字段通常用`.`或`->`访问。

## 53.2 深层字段来自嵌套对象，不是四个连续函数

`msg->pose.pose.position.x`中，msg是指针；第一层pose是带协方差的姿态包装；第二层pose是实际Pose；position是Point；x是数值字段。

读字段时可以借用局部引用：`const auto& point=msg->pose.pose.position`，减少重复。这个引用的使用寿命应在msg仍有效期间，不能存成一个比消息活得更久的悬空引用。

## 53.3 消息指针的const究竟放哪里

`const Odom::SharedPtr msg`：不能在本回调里给这个局部shared_ptr变量重新赋值，但通常仍可修改指向的消息。

`Odom::ConstSharedPtr msg`：共享的是const Odom，通过它不能修改消息字段。

本书观察器使用第二种。v11的一些回调采用第一种写法，解释源码时必须讲清语法本义，而不是一概称为“只读消息”。

## 53.4 合成里程计发布器

**文件：`ros2_ws/src/cpp_tutorial/src/fake_odom.cpp`**

```cpp
#include <chrono>
#include <cmath>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class FakeOdom final : public rclcpp::Node {
public:
    FakeOdom() : Node("cpp_fake_odom")
    {
        publisher_ = create_publisher<nav_msgs::msg::Odometry>(
            "/cpp_tutorial/odom", rclcpp::SensorDataQoS());
        timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
            nav_msgs::msg::Odometry msg;
            msg.header.stamp = now();
            msg.header.frame_id = "map";
            msg.child_frame_id = "base_link";
            phase_ += 0.05;
            msg.pose.pose.position.x = std::sin(phase_);
            msg.pose.pose.position.z = 1.3;
            msg.pose.pose.orientation.w = 1.0;
            publisher_->publish(msg);
        });
    }
private:
    double phase_ = 0.0;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeOdom>());
    rclcpp::shutdown();
    return 0;
}
```

phase_是持续保存的相位，不是位姿源的真实时间；sin生成一条教学曲线，z固定1.3，四元数w=1表示本实验的单位姿态。它与真实飞机完全无关。

`publish(msg)`传入的是消息对象。函数如何序列化、复制或在进程内优化，由框架和配置决定；初学者不需要假设“永远零拷贝”才能正确编程。

## 53.5 观察器与虚拟目标发布器

**文件：`ros2_ws/src/cpp_tutorial/src/observer_target.cpp`**

```cpp
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class ObserverTarget final : public rclcpp::Node {
public:
    ObserverTarget() : Node("cpp_observer")
    {
        offset_m_ = declare_parameter<double>("offset_m", 1.0);
        if (!std::isfinite(offset_m_) || std::abs(offset_m_) > 5.0) {
            throw std::invalid_argument("offset_m must be finite and within [-5,5]");
        }
        publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/cpp_tutorial/target", 10);
        subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            "/cpp_tutorial/odom", rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                const auto& point = msg->pose.pose.position;
                if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                    !std::isfinite(point.z)) {
                    latest_.reset();
                    received_.reset();
                    return;
                }
                latest_ = msg;
                received_ = Clock::now();
            });
        timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
            if (!latest_ || !received_ ||
                Clock::now() - *received_ > std::chrono::milliseconds(300)) {
                return;
            }
            geometry_msgs::msg::PoseStamped target;
            target.header.stamp = now();
            target.header.frame_id = "map";
            target.pose.position = latest_->pose.pose.position;
            target.pose.position.x += offset_m_;
            target.pose.orientation.w = 1.0;
            publisher_->publish(target);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "tutorial only: target_x=%.3f", target.pose.position.x);
        });
    }
private:
    using Clock = std::chrono::steady_clock;
    double offset_m_ = 1.0;
    nav_msgs::msg::Odometry::ConstSharedPtr latest_;
    std::optional<Clock::time_point> received_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<ObserverTarget>());
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("cpp_observer"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
```

回调只验证并保存最新消息及接收时刻；timer检查新鲜度后生成虚拟target。测量latest_与命令target分开；target始终发布到`/cpp_tutorial/target`，不是飞行控制话题。

这里的frame_id约定是教学发布器固定给出的map。把它接到任意外部消息前，应验证frame_id和坐标约定，而不是照搬。

## 53.6 QoS的最低限度理解

发布端提供QoS，订阅端请求QoS，两者需要兼容。不是“topic名称相同且C++编译通过就一定能收到”。SensorDataQoS常用于允许部分丢帧的传感器链路；本实验两个端点都采用它。

队列深度10不是10Hz，也不是保证只延迟10毫秒。消息频率、队列容量、可靠性和应用处理时间是不同概念。

## 53.7 编译与运行命令

在已安装ROS2 Humble的Ubuntu环境，进入配套工程内的ros2_ws目录：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select cpp_tutorial
source install/setup.bash
export ROS_DOMAIN_ID=87
export ROS_LOCALHOST_ONLY=1
ros2 run cpp_tutorial hello_node
```

另开终端时，每个终端都source同样的环境，并设置相同的实验隔离变量，然后分别运行：

```bash
ros2 run cpp_tutorial fake_odom
ros2 run cpp_tutorial observer_target
ros2 topic echo /cpp_tutorial/target
```

87只是本书实验示例域号，不是安全认证机制。仍需断开真实飞控，确认没有把实验话题重映射到`/mavros/...`。结束程序用Ctrl+C。

## 53.8 完整包配置

**文件：`ros2_ws/src/cpp_tutorial/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.8)
project(cpp_tutorial)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
foreach(node hello_node fake_odom observer_target mock_server mock_client)
  add_executable(${node} src/${node}.cpp)
  target_compile_features(${node} PUBLIC cxx_std_17)
  ament_target_dependencies(${node} rclcpp nav_msgs geometry_msgs example_interfaces)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${node} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()
ament_package()
```

**文件：`ros2_ws/src/cpp_tutorial/package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>cpp_tutorial</name>
  <version>2.0.0</version>
  <description>Isolated ROS 2 exercises accompanying the CUADC C++ tutorial.</description>
  <maintainer email="maintainer@example.invalid">CUADC tutorial maintainer</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>example_interfaces</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

CMake中的每个add_executable对应一个main。不能把五个都含main的cpp合并链接成一个可执行文件。package.xml声明包依赖；头文件include、CMake链接、package.xml依赖是三个相关但不相同的层次。

<a id="ch54"></a>
# 第 54 章：参数不是全局变量——声明、加载、验证和生效

## 54.1 一个参数至少有三层

ROS参数服务保存的值；构造时读取到offset_m_的成员值；真正被控制公式使用的值。这三者可能相同，也可能因只读一次、限幅或profile覆盖而不同。

本书ObserverTarget在构造时读一次offset_m_。运行期间执行`ros2 param set`不等于自动修改这个成员。除非代码注册参数回调或在计算前重新读取，否则不能声称运行逻辑已更新。

## 54.2 模板参数与运行参数不要混淆

`declare_parameter<double>("offset_m",1.0)`中的double在编译时决定类型；字符串是运行时参数名；1.0是缺少覆盖时使用的默认值。

`get_parameter(...).as_double()`则是先获得参数对象，再请求按double读取。类型不匹配不应靠猜测；应查看YAML数字写法、节点名和实际声明。

## 54.3 参数验证应先检查有限性，再检查边界

```cpp
// 验证片段。
if (!std::isfinite(offset_m_) || std::abs(offset_m_) > 5.0) {
    throw std::invalid_argument("invalid offset_m");
}
```

对NaN仅做`value<min || value>max`可能无法拒绝它。clamp也不是通用输入验证器。构造失败时本书main捕获异常，记录并返回非零退出码。

## 54.4 组合约束不能靠单参数范围解决

fine_alt≤coarse_alt、payload_count与offset数组长度一致、舵机数组均含两个元素，这些是组合契约。

检查数组下标之前，必须先验证尺寸。利用&&的短路可以安全串联，但新人更适合先写几个清晰的if，让错误日志直接指向原因。

## 54.5 v11实际存在profile覆盖

原代码load_parameters先读取声明参数，再做限幅，然后在rescue_reliability_profile为true时强制赋一批高度、速度和门限。最终行为以成员变量为准，而不只是YAML或参数服务器可见值。[B1]

教学改进建议是把“用户输入配置”和“解析后的有效配置”分成两个对象，启动时打印有效配置。这是新的设计建议，不是说v11已经实现了两套配置结构。

## 54.6 学会设计可诊断的参数错误

不要只输出“config invalid”。更适合维护的错误包括字段名、收到的值、允许范围、是否被profile覆盖。飞行使能仍应默认关闭，教学代码则完全不提供飞行使能或真实动作入口。

<a id="ch55"></a>
# 第 55 章：执行器、回调组和对象寿命，为什么程序会卡住

## 55.1 回调是一段待调用代码，执行器负责调度

C++中保存一个Lambda，只是保存可调用对象。ROS执行器等待事件，发现订阅消息、timer或service响应已准备好，再安排对应回调。

回调不是天然一个独立线程；单线程spin里，一个耗时回调会延迟其他回调。多线程执行器也不是自动把所有数据竞争问题解决了。[R19]

## 55.2 用时间轴看阻塞问题

```text
timer开始 ---- 发送service ---- 同步等待响应 ..............
单线程执行器                 无法返回去处理响应
```

这和C++语法无关，而是执行模型错误。解决方案不是“再套一个while”，而是把等待拆到后续tick，或者正确设计执行器、回调组和异步完成处理。

## 55.3 默认回调组会影响多线程行为

互斥回调组的回调不会在同一组内并发运行。即使创建多线程执行器，全部回调仍在同一互斥组，也不能假设它们就会互相抢占。

把回调分组或设为可重入后，需要自己保护共享position、history、state等对象。原先单线程安全的“查vector然后erase”未必还能无锁使用。[R20]

## 55.4 保存成员资源，不保存危险悬空引用

subscription_、publisher_、timer_、client_常保存为节点成员，是因为任务要在构造函数结束后持续使用它们。

回调捕获this只保存地址。把局部配置按引用捕获进长期订阅回调，函数返回后就可能失效。捕获shared_ptr可以延长被指对象寿命，但也可能形成“节点拥有timer，timer的回调又拥有节点”的引用环。

## 55.5 什么时候选择weak_ptr

当回调可能比对象活得久，但不应该阻止对象销毁时，可以捕获weak_ptr，在执行时lock。构造阶段直接shared_from_this通常不成立，因为对象还没完成共享所有权建立；应在工厂或构造后初始化阶段建立这种回调。

这是较复杂的生命周期设计。简单单线程教学节点不用为了“显得安全”而机械地塞入shared_from_this。

## 55.6 日志也消耗时间

在20Hz循环里无节制输出长文本，可能影响调度和存储。日志应围绕状态转换、错误原因和节流状态观测。

RCLCPP_INFO不等于printf的完全替代：它使用logger、严重级别和可配置输出。`%zu`对应size_t，`%s`传c_str()，不要把类型不匹配留给可变参数宏猜测。[R11]

<a id="ch56"></a>
# 第 56 章：ROS异步服务，完整示范pending请求的生命周期

## 56.1 先用加法服务，不用解锁飞机做语法实验

本书采用example_interfaces::srv::AddTwoInts，服务名`/cpp_tutorial/add`。客户端发送20和22，服务端回应42。这个练习没有飞控依赖。

返回42不代表网络时延满足飞行要求；目的只是观察Request、Response、Future和request_id如何协作。

## 56.2 完整模拟服务端

**文件：`ros2_ws/src/cpp_tutorial/src/mock_server.cpp`**

```cpp
#include <memory>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <rclcpp/rclcpp.hpp>

class MockServer final : public rclcpp::Node {
public:
    using Service = example_interfaces::srv::AddTwoInts;
    MockServer() : Node("cpp_mock_server")
    {
        service_ = create_service<Service>("/cpp_tutorial/add",
            [this](const std::shared_ptr<Service::Request> request,
                   std::shared_ptr<Service::Response> response) {
                // 限制本实验输入域，避免有符号整数相加溢出。
                if (request->a < -1000000 || request->a > 1000000 ||
                    request->b < -1000000 || request->b > 1000000) {
                    response->sum = 0;
                    RCLCPP_WARN(get_logger(), "input outside tutorial domain; response=0");
                    return;
                }
                response->sum = request->a + request->b;
            });
    }
private:
    rclcpp::Service<Service>::SharedPtr service_;
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockServer>());
    rclcpp::shutdown();
    return 0;
}
```

两个回调参数分别是请求和响应的共享指针；服务端给response->sum赋值。加法前限制本实验输入范围，避免int64相加溢出。范围外回应0并告警只是本教学服务的约定，生产接口应有独立错误字段。

## 56.3 完整非阻塞客户端

**文件：`ros2_ws/src/cpp_tutorial/src/mock_client.cpp`**

```cpp
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <optional>
#include <exception>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <rclcpp/rclcpp.hpp>

class MockClient final : public rclcpp::Node {
public:
    using Service = example_interfaces::srv::AddTwoInts;
    using Client = rclcpp::Client<Service>;
    using Clock = std::chrono::steady_clock;
    MockClient() : Node("cpp_mock_client")
    {
        client_ = create_client<Service>("/cpp_tutorial/add");
        timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { tick(); });
    }
private:
    struct Pending {
        std::int64_t id;
        Client::SharedFuture future;
        Clock::time_point sent;
    };
    void tick()
    {
        if (done_) { return; }
        if (!pending_) {
            if (!client_->service_is_ready()) { return; }
            auto request = std::make_shared<Service::Request>();
            request->a = 20;
            request->b = 22;
            try {
                auto result = client_->async_send_request(request);
                pending_ = Pending{result.request_id, result.future.share(), Clock::now()};
            } catch (const std::exception& error) {
                RCLCPP_ERROR(get_logger(), "send failed: %s", error.what());
                done_ = true;
            }
            return;
        }
        if (pending_->future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            try {
                const auto response = pending_->future.get();
                RCLCPP_INFO(get_logger(), "sum=%lld", static_cast<long long>(response->sum));
            } catch (const std::exception& error) {
                RCLCPP_ERROR(get_logger(), "response error: %s", error.what());
            }
            pending_.reset();
            done_ = true;
        } else if (Clock::now() - pending_->sent >= std::chrono::seconds(1)) {
            (void)client_->remove_pending_request(pending_->id);
            pending_.reset();
            done_ = true;
            RCLCPP_ERROR(get_logger(), "local request timed out; no remote cancellation implied");
        }
    }
    bool done_ = false;
    std::optional<Pending> pending_;
    Client::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockClient>());
    rclcpp::shutdown();
    return 0;
}
```

客户端保留pending三件套：请求编号、共享future、发送时刻。首次发现服务可用时发送一次，随后每50ms检查；准备好则get；超时则remove_pending_request并清空pending。

运行方式是在两个隔离终端分别执行`ros2 run cpp_tutorial mock_server`与`ros2 run cpp_tutorial mock_client`。客户端只完成一次请求，之后保持节点运行，按Ctrl+C退出。

## 56.4 把关键链式表达式展开

v11常见：

```cpp
// 源码形式片段，不是本实验的真实飞控调用入口。
future_ = client_->async_send_request(request).future.share();
```

等价阅读方式：先调用async_send_request；得到一个包装对象；访问它的future字段；调用std::future::share；把返回的shared_future移动/赋给成员。

Humble这个包装对象还带request_id。完整教学版把它保存起来，而不是在这一长行里丢掉。[R17]

## 56.5 为什么超时要清理client内部pending，而不只是清future_

清空自己的future句柄只是在释放一个引用。客户端还可能保存请求对应的promise或回调状态。Humble源码明确提供remove_pending_request来清理无响应请求。

因此新教学实现用request_id清理；原v11的部分超时分支只将future置空，这一点不能被描述成已经完成内部清理。[B1]、[R17]

## 56.6 清理不等于撤销

网络另一端可能已计算完加法；对于舵机，更可能已经执行动作。remove_pending_request只表示“本客户端不再等待这条请求”，不是远端撤销命令。

设计物理动作时应额外讨论幂等性、迟到响应、重复请求及动作状态不确定。对于已发出的投放请求，不应仅因没ACK就自动假定“没有投”，再毫无约束地重复。

## 56.7 `get()`与异常

future准备好后，get可能得到响应，也可能重新抛出共享状态中保存的异常。客户端catch std::exception并记录原因。业务层还需检查响应字段；不能把“没抛异常”等价为“动作接受”。

模拟服务成功/超时的运行行为尚需你们ROS环境验证；纯C++future机制已由前面的离线程序实际验证。

<a id="ch57"></a>
# 第 57 章：MAVROS只是换了消息与服务类型，不能省掉业务判断

## 57.1 四层状态分开看

ROS客户端请求提交成功；MAVROS收到/返回服务响应；飞控接受命令；物理世界确实达到目标。这四层相关，但没有任何一层能自动证明后面全部成立。

例如takeoff请求返回success，不代表已经到4m；CommandLong回应接受，不代表水瓶已脱落；land请求接受，不代表已经可以安全切断电机。

## 57.2 从已学过的AddTwoInts迁移

| 教学服务 | v11中的类似结构 |
|---|---|
| `AddTwoInts::Request` | `CommandBool::Request`、`CommandTOL::Request`、`CommandLong::Request` |
| `request->a` | `request->value`、`request->altitude`、`request->param1` |
| `rclcpp::Client<Service>` | 相同的模板客户端模式 |
| 返回sum | 返回success/result等字段，再由任务解释 |
| 纯计算 | 可能影响真实飞行与载荷，需更多安全门禁 |

本教程只展示源码阅读，不另提供一键解锁和真实投放练习。[B1]

## 57.3 原文件实际使用的输入/输出

输入包括`/mavros/state`、`/mavros/local_position/odom`、`/mavros/global_position/compass_hdg`、`/cuadc/nav30_ready`、`/perception/drop_buckets_body`与侦察照片ACK；输出包括位置setpoint、侦察模式与抓拍请求，服务用于解锁、起降和舵机。

准确拼写是`/cuadc/...`，不是`/cudac/...`。话题字符串是运行时通信地址，拼错时C++编译器通常无法发现。[B1]

## 57.4 `request_allowed()`不是飞行安全的全部

v11通过最近请求时间限制服务发送频率，避免20Hz每轮都发命令。这是限频，不代表它已经验证每个动作的物理前提。起飞、解锁、上锁、投放还应分别检查状态与传感器条件。

## 57.5 位置命令与姿态控制的职责边界

任务程序发布目标位置与固定航向，ArduPilot完成内部跟踪控制。本C++中几何函数计算的是“希望哪个参考点到哪里”，不是直接算电机输出的PID。

不要为了调位置跟踪误差，首先在任务回调里随意加电机控制逻辑。先定位误差来自坐标、时间、目标规划还是底层响应。

<a id="ch58"></a>
# 第 58 章：阅读4345行v11的正确顺序，不是从第1行一路硬啃

## 58.1 第一遍，只建立目录

先找main、类定义、构造函数、enum class State、tick、enter、所有update_*。在纸上写出“哪些回调输入事实、哪些函数产生目标、哪些函数发送请求”。暂时不要深入所有数字参数。

附录提供从实际文件提取的函数行号索引。行号只适用于附录SHA-256标识的文件，后续改文件后要重新定位。

## 58.2 第二遍，从main走到构造函数

main初始化ROS并创建VisualDropMissionNode。构造函数声明/读取参数，建立订阅者、发布者和服务客户端，初始化时间点，建立50ms的tick定时器。

其中创建subscription并不会立即保证数据已经收到。have_odom、have_compass、have_vision_message等字段就是用来区分“对象创建好了”与“外部事实出现了”。

## 58.3 第三遍，先看一轮tick做什么

原文件顺序大致是：pending视觉→视觉健康→服务响应→舵机响应→故障收回→初始化机构→发布已有target→总超时和状态日志→switch推进当前状态。[B1]

注意原代码把发布放在状态更新之前，所以在本轮update_*中计算的新target通常到后续tick才发布。教学代码或重构可以采用其他顺序，但必须显式标注差异，不能一边讲原源码一边悄悄换执行顺序。

## 58.4 第四遍，沿正常状态链只走一次

```text
WAIT_FCU → WAIT_NAV_STABLE → PRESTREAM → WAIT_GUIDED → WAIT_ARM
→ TAKEOFF → SEARCH → ALIGN_COARSE → ALIGN_FINE → RELEASE
→ 第二目标的COARSE/FINE/RELEASE
→ 原版六点侦察 → 原地爬升 → 返航 → LAND → DISARM → DONE
```

对每条箭头，只回答三个问题：什么输入让条件成立；等待期间保存了什么；超时或输入失效去哪。先不要把每条错误分支一起展开。

## 58.5 第五遍，再补横切保护

人工切出GUIDED、导航过期、搜索视觉失效、总超时、舵机ACK迟到、落地确认丢失。这些不是正常链里的新“航点”，而是能从多个状态进入的保护流程。

正常链看懂以后补保护，能避免读者把所有if混成一个没有结构的大分支树。

## 58.6 构造/门禁位置有一个重要区分

navigation_ready_to_lock要求导航、航向、视觉等就绪，但不直接检查servos_initialized。舵机初始化ACK在WAIT_ARM及自动解锁条件中另行要求。

所以应说“起飞前流程整体还会等机构初始化”，不能把所有要求都说成同一个函数里的布尔表达式。[B1]


# 第九篇：带着语言知识逐段阅读正式源码

<a id="ch59"></a>
# 第 59 章：源码精读一——移动队列元素、查找轨迹与忽略返回值

## 59.1 `process_pending_vision_frames` 原函数

**基准源码原文：`process_pending_vision_frames`，第1205—1233行。**

```cpp
void process_pending_vision_frames()
{
  while (!pending_vision_frames_.empty()) {
    const PendingVisionFrame & front = pending_vision_frames_.front();
    const auto nav = navigation_sample_at(front.stamp);
    const bool wait_expired = steady_age_s(front.arrival) >= vision_pending_buffer_s_;

    if (!nav.has_value() && !wait_expired) {
      return;
    }

    PendingVisionFrame frame = std::move(pending_vision_frames_.front());
    pending_vision_frames_.pop_front();

    if (!nav.has_value()) {
      if (nav30_ready_fresh()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "HARD_SYNC_REJECT_AFTER_WAIT: no bracketing odom within %.0fms "
          "for capture stamp; nav_gap_limit=%.0fms",
          vision_pending_buffer_s_ * 1000.0,
          nav_interpolation_max_gap_s_ * 1000.0);
      }
      continue;
    }

    process_time_aligned_vision_frame(frame, *nav);
  }
}
```

这段是原v11摘录，不是教学重写。请从五个语法点逐一分析：deque首元素引用、optional返回、短路逻辑、move后pop、while中的return/continue。[B1]

## 59.2 `const PendingVisionFrame& front` 只暂时借用

front指向队首对象，避免为了检查时间戳就复制整帧检测数组。此时队首仍存在，因此可以读取front.stamp和front.arrival。

随后构造局部frame并pop之后，front不能继续使用。原函数后续改用frame，这一点是正确的生命周期分工。

## 59.3 `nav`为什么按值接收

navigation_sample_at返回optional<NavigationSample>。`const auto nav=...`在当前作用域拥有一份查询结果，和队首对象寿命无关。

后面`*nav`是取出其中的NavigationSample；必须先由has_value分支保证存在。这里的星号是optional的解引用运算，不是乘法，也不是自动new了一个导航对象。

## 59.4 `std::move`与pop是两件事

move允许frame采用队首对象的移动构造，把检测vector等资源接过来；pop_front随后销毁仍在队列中的那个已被移动对象。少掉pop会让队列一直保留已处理条目；反过来先pop又会失去数据。

准确描述应该是“将元素移入局部对象后移除队首”，而不是“move会把deque里的元素删掉”。

## 59.5 不可对齐且还没过期，为什么return

因为当前队首还在等待后续导航样本。return结束这轮处理，之后odom回调或下一次tick会重试。这个函数没有阻塞等待导航。

如果已经过期，则弹出这帧后continue，允许继续处理后续队列项。由此可见，两个关键字影响的控制流不同，不能互换。

## 59.6 原工程的`value_at`与“兜底”策略

**基准源码原文：`value_at`，第206—213行。**

```cpp
template<typename T>
T value_at(const std::vector<T> & values, std::size_t index, const T & fallback)
{
  if (index < values.size()) {
    return values[index];
  }
  return values.empty() ? fallback : values.back();
}
```

`values.empty() ? fallback : values.back()`明确表示：空数组用fallback；非空但index越界用末项。比如数组只有一个动作持续时间，后续payload沿用最后一个值。

这个策略不天然适用于任何参数。通道号若越界就沿用最后一个通道，可能意味着两个动作打到同一输出；正式代码还会对关键数组做额外配置校验。教学里不能只把该函数称为“越界返回fallback”。

## 59.7 查找轨迹的Lambda

原逻辑用find_if寻找`track.id==active_bucket_->id`。Lambda捕获this，访问成员active_bucket_；算法传入每个BucketTrack的const引用；find_if返回iterator；检查不等于end后才解引用。

这一条语句其实组合了六个已学概念。看不懂时先为类型起别名，再展开Lambda为具名小函数，而不是整行死记。

## 59.8 `(void)try_reacquire...`是什么意思

把表达式转换为void，明确表示调用函数但不使用返回值。函数仍然执行，其修改active_bucket_等副作用仍然发生。

这与删除函数调用不同，也不等于返回false。工程上应确认忽略返回值是否合理；例如调用后再次检查目标身份，就是一种清晰的业务判定方式。

<a id="ch60"></a>
# 第 60 章：源码精读二——粗对准、细对准与RELEASE门禁

## 60.1 先画三个阶段的数据流

粗对准持续使用可信active_bucket坐标接近目标；粗对准完成时保存coarse参考；细对准在限制范围内接受新视觉修正；进入RELEASE时冻结最后可信目标；RELEASE几何仍根据实时姿态改变飞机参考点目标。[B1]

三个阶段不是三个互不相关的PID，而是任务逻辑在不同高度和容差下使用同一几何关系。

## 60.2 原函数怎样反算参考点目标

**基准源码原文：`desired_vehicle_pose_for_release_target`，第1477—1500行。**

```cpp
Point3 desired_vehicle_pose_for_release_target(
  const Point3 & target_local, double relative_altitude_m) const
{
  const Point3 release_body =
    vector3_at(release_offsets_, payload_index_, Point3{});

  Point3 rotated;
  if (release_geometry_compensation_enabled_) {
    // Rigid-body geometry: p_release = p_vehicle + R_WB * r_release^B.
    // Solve p_vehicle_xy so the actual outlet, including the -Z lever arm,
    // remains over the frozen bucket while the aircraft tilts into wind.
    rotated = rotate_body_vector_to_local(
      release_body, current_roll_, current_pitch_, current_geometry_yaw());
  } else {
    rotated = rotate_body_vector_to_local(
      Point3{release_body.x, release_body.y, 0.0},
      0.0, 0.0, mission_yaw_);
  }

  return Point3{
    target_local.x - rotated.x,
    target_local.y - rotated.y,
    home_->z + relative_altitude_m};
}
```

release_body从当前payload的三元组外参取值；enabled分支用当前roll/pitch/geometry_yaw进行完整旋转；关闭补偿时仅取水平偏置、按任务yaw旋转；最后返回桶x/y减偏置，z仍是home_->z+设定高度。

函数末尾const表示不修改节点的普通成员状态，并不表示它返回的Point3是全局永远不变的常量。当前姿态成员变化后，下次计算结果也会变化。

## 60.3 粗对准的代码结构

先验证active目标与开始时间；超时则重试或返航；计算desired；根据距目标距离选速度；slew更新setpoint；检查身份；比较机体中心、出瓶点、高度误差；连续稳定后进入细对准。

学习重点不是背0.35，而是每一个early return分别排除哪一种不成立条件。稳定计时中间失效会reset，不能把不同时间片拼接成一次稳定。

## 60.4 细对准不是无条件等待新视觉

fine_alignment_visual_valid检查目标身份、新鲜度、与粗对准参考的偏移。视觉有效时更新fine目标；失效时记录丢失起点，并继续使用最后可信目标下降/修正。

达到丢失宽限且落入较宽释放几何包络时可fallback；细对准超时也有受条件约束的fallback。正常路径的细对准稳定门与fallback路径并非完全同一个判据，所以不能简单说“每次一定达到12cm稳定后才RELEASE”。[B1]

## 60.5 RELEASE原函数

**基准源码原文：`update_release`，第3346—3465行。**

```cpp
void update_release()
{
  if (!active_bucket_.has_value() || !direct_release_active_) {
    abort_or_land("Release entered without a frozen direct-release target");
    return;
  }

  // RELEASE target coordinate is frozen, but aircraft-center setpoint is not:
  // continuously compensate the body-fixed outlet lever arm for current attitude.
  direct_release_pose_ = desired_vehicle_pose_for_release_target(
    direct_release_target_local_, fine_align_alt_m_);
  if (!release_started_ && !release_command_pending_) {
    slew_target_toward(
      direct_release_pose_, release_geometry_correction_speed_m_s_);
  } else {
    target_ = direct_release_pose_;
  }

  const Point3 release_point = release_point_local_current();
  update_release_point_velocity_estimator(release_point);
  const Point3 predicted_release_point =
    predicted_release_point_local(release_point);

  if (!release_started_ && !release_command_pending_) {
    const bool identity_ok =
      target_selection_locked_ &&
      payload_index_ < selected_target_ids_.size() &&
      active_bucket_->id == selected_target_ids_[payload_index_] &&
      !target_id_already_used(active_bucket_->id);
    const bool area_ok =
      inside_release_area(direct_release_target_local_) &&
      inside_release_area(release_point) &&
      inside_release_area(predicted_release_point);
    const double center_xy_error = distance_xy(position_, direct_release_pose_);
    const double release_point_xy_error =
      distance_xy(release_point, direct_release_target_local_);
    const double predicted_release_point_xy_error =
      distance_xy(predicted_release_point, direct_release_target_local_);
    const double height_error = std::abs(position_.z - direct_release_pose_.z);
    const bool motion_ok =
      horizontal_speed_m_s_ <= direct_release_max_horizontal_speed_m_s_ &&
      std::abs(vertical_speed_m_s_) <= direct_release_max_vertical_speed_m_s_;
    const bool attitude_ok =
      std::abs(current_roll_) <= direct_release_max_tilt_rad_ &&
      std::abs(current_pitch_) <= direct_release_max_tilt_rad_ &&
      angular_rate_rad_s_ <= direct_release_max_angular_rate_rad_s_;
    const bool geometry_ok =
      center_xy_error <= direct_release_xy_tolerance_m_ &&
      release_point_xy_error <= direct_release_point_xy_tolerance_m_ &&
      predicted_release_point_xy_error <= direct_release_point_xy_tolerance_m_;
    const bool gate_ok =
      identity_ok && area_ok && geometry_ok &&
      height_error <= direct_release_height_tolerance_m_ &&
      motion_ok && attitude_ok;

    if (!gate_ok) {
      direct_release_stable_since_.reset();
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "Direct release waiting: identity=%s area=%s center_xy=%.3f/%.3f "
        "release_xy=%.3f/%.3f predicted=%.3f/%.3f z=%.3f/%.3f "
        "hspeed=%.3f/%.3f vz=%.3f/%.3f roll=%.1f pitch=%.1f",
        identity_ok ? "yes" : "no", area_ok ? "yes" : "no",
        center_xy_error, direct_release_xy_tolerance_m_,
        release_point_xy_error, direct_release_point_xy_tolerance_m_,
        predicted_release_point_xy_error, direct_release_point_xy_tolerance_m_,
        height_error, direct_release_height_tolerance_m_,
        horizontal_speed_m_s_, direct_release_max_horizontal_speed_m_s_,
        std::abs(vertical_speed_m_s_), direct_release_max_vertical_speed_m_s_,
        current_roll_ * 180.0 / kPi, current_pitch_ * 180.0 / kPi);

      if (steady_age_s(state_enter_time_) >= direct_release_timeout_s_) {
        retry_alignment_or_return("DIRECT_RELEASE_TIMEOUT");
      }
      return;
    }

    if (!direct_release_stable_since_.has_value()) {
      direct_release_stable_since_ = SteadyClock::now();
      RCLCPP_WARN(
        get_logger(),
        "Direct release gates valid for payload %zu; holding %.2fs",
        payload_index_ + 1U, direct_release_stable_s_);
      return;
    }
    if (steady_age_s(*direct_release_stable_since_) <
      direct_release_stable_s_)
    {
      return;
    }

    // Keep holding the attitude-compensated outlet target while the servo
    // command is in flight; do not replace it with the instantaneous center.
    target_ = direct_release_pose_;
    release_command_pending_ =
      send_servo(payload_index_, true, ServoPurpose::RELEASE);
    if (!release_command_pending_) {
      abort_or_land("Unable to send release command");
    }
    return;
  }

  const double duration =
    value_at<double>(release_duration_s_, payload_index_, 0.7);
  if (release_started_ && return_to_stowed_ && !return_sent_ &&
    !return_command_pending_ &&
    steady_age_s(release_start_time_) >= duration)
  {
    return_command_pending_ =
      send_servo(payload_index_, false, ServoPurpose::RETURN_STOWED);
    if (!return_command_pending_) {
      abort_or_land("Unable to send stowed-return command after release");
    }
    return;
  }

  if (release_started_ && (!return_to_stowed_ || return_sent_)) {
    finish_payload_release();
  }
}
```

## 60.6 把布尔表达式当成一张决策表

identity_ok检查锁定计划、索引合法、当前ID匹配且未使用；area_ok检查冻结桶坐标、当前出瓶点、预测出瓶点均在内缩区；geometry_ok比较误差；motion_ok检查水平与垂直速度；attitude_ok检查roll/pitch和总角速度。

这个阶段的identity_ok并未重新执行active_target_identity_valid里全部位置/直径判断；那些约束主要在此前对准阶段约束目标。阅读时应区分“名字类似”与“表达式相同”。

## 60.7 `release_started_`和`release_command_pending_`不能合并

pending表示请求已发出但等待响应；started表示得到接受响应、开始按软件动作时序计时。二者不是同一个事实。

代码仅在两者都为false时执行发令前的完整释放门禁与连续稳定检查。一旦请求正在飞行中或已开始，就进入等待/动作时序，而不是每轮重新执行相同发令门。

因此不能承诺“门禁一直实时控制到水瓶物理脱离”，更不能承诺发令后姿态变差能撤回已经完成的机械动作。

## 60.8 setpoint限速分支也有边界

尚未发令时，原函数调用slew_target_toward，以几何修正速度渐进更新；请求pending或started时，直接令target_=direct_release_pose_。

所以“RELEASE全程都严格按0.30m/s限速修正”并不是原代码的准确描述。本书保留原行为并指出差别，不直接改生产源码。

## 60.9 两个水平误差在某些条件下数学上相同

若补偿开启，当前姿态相同，desired_xy=bucket_xy-(Rr)_xy，则：

\[
p_{vehicle,xy}-p_{desired,xy}
=p_{vehicle,xy}+(Rr)_{xy}-p_{bucket,xy}.
\]

两边分别是中心到补偿目标的误差向量和出瓶点到桶的误差向量，因此其范数相同。保留两个日志字段有语义解释价值，但它们不是两个独立传感器提供的交叉验证。这是由源码公式得到的推论。

## 60.10 预测项与真正弹道的区别

predicted_release_point把当前出瓶点加上估计速度×命令到脱离延迟，默认延迟为0。它不是水瓶落地后的第一落点预测，也没有直接估计风场和空气阻力。

低速度、低高度减少某些误差，但不能把这些门限当成命中保证。理解代码能力边界本身就是维护者的责任。

<a id="ch61"></a>
# 第 61 章：失败处理、返航与原版侦察，别把日志当物理事实

## 61.1 三种“结束”

正常DONE、任务失败后的受控返航/降落、飞手切出GUIDED后的PILOT_OVERRIDE，处理目标不同。人工接管分支不应再偷偷切回自动模式。

原代码在接管期间仍可能尝试机构安全收回，这是对已发生机械动作的善后，不是继续飞行任务。最终退出条件也会检查pending相关状态。[B1]

## 61.2 ACK超时后的“不确定”是一种必要状态

超时说明软件没有在期限内得到期望反馈。它不能证明远端未执行；迟到ACK也不能让已经进入故障流程的任务随便补记为正常成功。

新教学模拟因此把超时写为STOP+unknown，正式v11则有更复杂的收回、返航或LAND路径。模拟STOP不是可以直接移植到真机的安全策略。

## 61.3 原版侦察仍是六张RGB照片

build_recon_route固定六个field航点，触发拍照后不在每点等待写盘ACK。recon_completed表示航线/触发阶段结束，不等于照片全部已保存。

DONE的正常成功判定还检查照片ACK数量、两瓶流程、未失败、已上锁与落地确认。即使所有请求都发了，也可能因为缺照片ACK不被记为完整成功。[B1]

## 61.4 O4和12m/s在哪里

它们属于此前讨论的改版设想。上传的基准文件仍保留原版六点侦察、原速度参数和照片ACK依赖。本书没有把提案写成已实现功能。

若以后实现，应更新状态定义、完成条件、图传可读覆盖、速度/制动验证，以及代码版本和测试；不能只修改几个数字就改称“理论最快”。

## 61.5 落地门禁读实际表达式，不只看默认参数

v11落地确认函数会对相对高度、水平速度、垂直速度再用min取更严格上限，稳定时间用max保证下限。最终生效值不是声明参数表上几个默认数字的直接照抄。

另一方面，odom低高度低速度依然是一种软件推断，不等于独立的接地传感器证据。正式飞行验证与飞控保护仍必不可少。

## 61.6 一个日志应该回答什么

“ALIGN失败”不足以定位；应区分身份不满足、时间过期、几何误差、速度过高、姿态过大或请求问题。

但日志如“飞控确认投瓶”只能说明代码在某个ACK分支打印了这句话。阅读者要知道它的证据是ACK，不是瓶子脱离传感器。变量和日志命名不能替代物理观测。

<a id="ch62"></a>
# 第 62 章：重构时先保留行为，再谈模块漂亮

## 62.1 新人不要同时重写全部架构和参数

同时改坐标、状态、门限、速度、数据协议，会导致行为变化无法归因。更可控的方法是先抽取不含ROS的纯函数，维持输入输出一致，建立回归测试，再逐步调整结构。

例如先抽出distance_xy、角度归一化、投放口旋转；下一步抽出Segment；再抽出时间插值；最后才拆复杂有状态任务。

## 62.2 头文件的责任是稳定接口

types.hpp只定义类型；geometry.hpp声明数学函数；tracker.hpp暴露更新和查询；mission.hpp组织决策。实现细节留在cpp，但模板需要在使用处可见，不能毫无计划地拆到cpp导致链接失败。

别名、include guard、命名空间和const接口，是前面基础章节直接服务于模块化的地方。

## 62.3 保持行为的测试应包含什么

对数学函数：正常值、边界值、非法值、反向变换。对时间：空历史、等号、乱序、重复、空洞。对状态：正常迁移、重复事件、错配ACK、超时、迟到响应、终态不再启动。

对原业务关键行为：三桶选小二、先近后远、第二目标冻结、profile覆盖、RELEASE发令前门禁，以及原版照片ACK完成条件。

## 62.4 接口要说清所有权和单位

`const std::vector<Track>& tracks() const`是只读借用内部集合，不是快照。调用者不能在内部容器后续修改后继续假设迭代器有效。

`std::vector<Track> select_two(...) const`返回独立值，更适合冻结计划。`double`参数最好在命名中写m、s、rad，避免厘米、米、度、弧度混淆。

## 62.5 不要把“代码重复”当唯一重构依据

两个阶段都写速度门限，不一定说明应合并；一个可能是发令前判断，一个是落地确认，生命周期不同。抽象应统一确实相同的规则，而不是为了减少行数把不同语义混起来。

在新人阶段，一段能按时间线解释的重复代码，往往比一个过度通用、充满模板参数的框架更容易验证。

<a id="ch63"></a>
# 第 63 章：常见错误字典——从症状倒查C++知识点

下表中的错误短语是典型形式，不保证不同编译器逐字一致。先读第一条有意义的报错，再看随后连锁报错。

| 症状或诊断 | 常见原因 | 应检查的知识点/做法 |
|---|---|---|
| `g++: command not found` | 编译器未安装或PATH不对 | 第2章，先确认工具，不改源码 |
| `No such file or directory` | 当前目录/路径/头文件路径错误 | pwd、ls、引号、-I与依赖 |
| `expected ';'` | 上一条声明或语句缺分号 | 第3章，不只看箭头所在行 |
| `expected '}'` | 花括号配对错误 | 从函数/类边界检查 |
| `was not declared in this scope` | 尚未声明、拼错或作用域外 | 第14、17章 |
| `undefined reference` | 声明存在，但定义未链接或签名不一致 | 第4、41章 |
| `multiple definition` | 头文件放非inline定义被多次链接 | ODR、声明/定义 |
| `multiple definition of main` | 多个练习cpp一起链接 | 每个main单独建可执行目标 |
| `no matching function` | 参数类型/个数不符合任何重载 | 第14章，从最内层模板错误看起 |
| `cannot bind ... reference` | 引用const性或值类别不匹配 | 第15、24章 |
| `use of deleted function` | 复制unique_ptr或显式禁用复制 | 第24、25章 |
| `invalid use of non-static member` | 没有对象却调用成员 | this、成员函数指针、bind |
| `this was not captured` | Lambda需要访问成员却未捕获this | 第30章 |
| `no type named ...` | 类型名/命名空间/依赖版本不匹配 | 别名、模板、API版本 |
| `override ... does not override` | 签名与基类虚函数不同 | const、参数、virtual/override |
| `assignment of read-only...` | 尝试经const路径修改 | 第15、16、21章 |
| `bad_optional_access` | 对空optional调用value | 检查has_value，不把空当默认 |
| `future_error` | 无效future、重复get或promise生命周期问题 | 第44章 |
| `out_of_range` | at读取越界 | size、index、signed转换 |
| 程序不崩但数值很怪 | 未初始化、越界、悬空、溢出 | UB，使用检查与Sanitizer |
| 2.5变成2 | 整数除法/浮点转整数 | 第6、7章 |
| 循环永远不结束 | 条件没变化、无符号递减、continue跳过更新 | 第11章 |
| range-for改了但原数组没变 | 使用auto值复制元素 | auto&与const auto& |
| sort后相等项顺序变动 | sort不保证稳定；比较器无tie规则 | 第37章 |
| sort行为异常 | `<=`比较器、NaN或不满足严格弱序 | 明确排序前提 |
| vector.reserve后访问仍越界 | reserve只改变容量，不创建元素 | 第32章 |
| shared_ptr内数据出现竞争 | 引用计数保护不保护对象内容 | mutex/一致快照 |
| std::move后元素还在容器 | move不是erase | 第24、33章 |
| 回调晚一点突然崩溃 | 捕获悬空引用或对象已析构 | 第30、55章 |
| class成员初始化顺序告警 | 初始化列表顺序与声明顺序不一致 | 第22章 |
| ROS收不到消息但能编译 | 话题/类型/QoS/域号不同 | 第53章 |
| ROS参数set成功但行为没变 | 只在构造时加载或profile覆盖 | 第54章 |
| ROS服务回调内一直等待 | 执行器/回调组自阻塞 | 第44、55、56章 |
| 超时后内存持续增长 | 未清client内部pending | request_id/remove_pending_request |
| release计数增长两次 | 一次请求被多轮重复处理 | 幂等状态、active请求清理 |
| 一个false之后仍立刻通过稳定门 | 没reset稳定起点 | 第46章 |
| 目标越飞越偏 | 坐标/时间/外参/实际跟踪误差 | 不应首先归咎于C++编译器 |
| 运行新代码却打印旧日志 | 编译失败后跑旧binary，或source旧install | 构建返回码与环境顺序 |

## 63.1 排错的四句话

我期望什么；实际发生什么；最早在哪一步两者不同；这一步的输入是什么。只有“又报错了”无法帮助自己定位。

## 63.2 保存最小复现

把错误缩到一个十几行main或一个独立测试。输入要固定，编译命令要完整，错误文本不要只截最后一行。修复后把这个最小复现保留为回归测试，而不是删掉当作没发生过。

<a id="ch64"></a>
# 第 64 章：学习安排与总验收——不是读完多少行，而是能独立解释多少行为

## 64.1 按依赖分成八轮，而不是按日历强行一天一篇

| 轮次 | 章节 | 不看答案必须做到 |
|---|---|---|
| 1 | 1—12 | 创建cpp、编译、输入输出、写条件和循环，分清int/double |
| 2 | 13—20 | 设计函数，解释值/引用/指针，使用struct与enum |
| 3 | 21—28 | 自己写类，解释构造析构、复制移动、所有权和成员回调 |
| 4 | 29—38 | 读懂模板与Lambda，正确使用容器、optional和算法 |
| 5 | 39—44 | 拆文件、检查错误、使用chrono、理解future不阻塞检查 |
| 6 | 45—51 | 运行并修改纯C++任务，给稳定门/几何/时间/ACK补测试 |
| 7 | 52—58 | 在隔离ROS环境编译节点，解释每一条长类型和回调关系 |
| 8 | 59—64 | 对照v11指出数据生命周期、正常/故障链和实现边界 |

每轮都需要“预测输出→运行→解释差异→从空文件重写一个小变化”。不能只复制编译成功。

## 64.2 统一答题格式

对一段代码，写出输入、局部对象、跨调用状态、返回值/副作用、失败前提。下面的综合题使用这个格式；章节里的小练习仍保留在原处，不需要翻到书末才知道答案。

## 64.3 基础综合题与答案

**题1**：`double x=5/2;`为什么是2？

右边两个操作数都是整数，整数除法先产生2，再转换为double。改成5.0/2，或把操作数之一转为double。不能只看左边变量类型。

**题2**：`std::vector<int> a(2,9)`和`a{2,9}`区别？

前者含两个9；后者含2和9。圆括号匹配数量+初值构造，花括号优先考虑initializer_list。读容器初始化必须注意括号种类。

**题3**：`int& r=x; r=y;`会让r指向y吗？

不会。r已绑定x，赋值修改x的值。要表达可改绑，通常使用指针或其他显式间接结构。

**题4**：`const std::shared_ptr<T> p`意味着T只读吗？

不意味着。const约束智能指针变量本身；要通过该指针只读T，采用shared_ptr<const T>。两者也可以同时const。

**题5**：函数返回局部Point3的值会悬空吗？

按值返回不会因局部源对象离开作用域就变成悬空引用；返回对象独立存在，可能有复制消除或移动。返回局部Point3&才是典型生命周期错误。

**题6**：一个指针非空，能保证安全解引用吗？

不能。它可能指向已销毁对象或失效容器元素。空值检查与寿命证明是两件事。

**题7**：为什么析构函数不应该随意发送“降落命令”？

析构可能发生在异常、通信失效或资源部分销毁期间，无法承担完整物理安全协议。析构适合释放自己拥有的资源；安全任务流程应显式管理并验证。

**题8**：为什么不把所有函数都声明noexcept？

noexcept是异常不得逸出的承诺，异常仍逸出会终止程序。需要根据实际调用链和契约使用，不是“让程序忽略错误”。

**题9**：vector扩容后旧元素引用是否总有效？

不是。发生重分配时引用/指针/迭代器会失效。应避免跨结构修改长期借用，或采用独立值、稳定ID与重新查询。

**题10**：`std::sort`比较器为什么不用`a.d<=b.d`？

相同对象会比较为true，违反严格弱序要求的非自反性。应采用严格小于，并在确需稳定规则时处理相等情况。

**题11**：`all_of`作用于空范围返回true，是错误吗？

不是，它表示不存在不满足谓词的元素。但业务若要求“至少一个目标而且全部有效”，还要先检查非空。

**题12**：`const auto x=container.front()`为什么可能复制大对象？

auto默认按值推导，这里得到一个const值副本；若借用则写const auto&，但要继续保证容器首项寿命。

## 64.4 时间、异步和任务综合题

**题13**：20Hz轮询是不是观测数据也20Hz？

不是。一次旧观测可能被读很多次。数据的新鲜度和时间推进必须用观测/接收时间及链路统计证明。

**题14**：future.valid为true，get一定立即返回吗？

不一定。valid表示关联了共享状态，ready才表示结果已准备好。ready后get仍可能抛出保存的异常。

**题15**：超时后future={}能取消远端动作吗？

不能。还可能需要清理客户端pending；远端是否已执行需独立处理。对投放尤其不能把本地不再等待当作水瓶仍挂着的证据。

**题16**：为什么第42个练习不会睡真实60秒？

它注入模拟时间，每轮把时间数值推进100ms。测试事件逻辑不依赖真实经过时间，所以可以快速、确定性运行。

**题17**：两个body向量能直接与local位置相加吗？

只有方向基准一致才可以。机体系位移要先旋转到local方向，再与local位置相加。数据都用Point3并不代表坐标系相同。

**题18**：调用new_target=desired后，position是否也应更新？

不应。position来自观测，target来自任务。模拟器可以另外根据运动模型更新观测，但不能在任务层伪造到达。

**题19**：为什么冻结第二目标常用值快照而不是vector元素指针？

值快照明确拥有数据，不受原容器扩容/删除影响。它仍可能过时，所以要记录观测时间和任务使用条件，而不是假装是实时视觉。

**题20**：RELEASE中的0.5秒是什么性质的时间？

它是发令前条件连续满足的稳定门，不是ACK往返时延，也不是水瓶自由落体时间。舵机保持时间又是另一条时序。

**题21**：v11的“时间硬同步”是否证明硬件曝光和IMU已同步？

没有。代码主要在统一主机接收时钟域内进行历史夹逼插值，仍需独立处理采样、通信和推理时延。

**题22**：原版照片触发6次与6张照片ACK都收到一样吗？

不一样。请求数量与保存完成数量是不同证据。DONE还会检查后者。

## 64.5 最后的开放题

从空文件写一个三状态程序：WAIT_DATA、WAIT_ACK、DONE。输入时刻可注入；数据超时拒绝启动；请求只能发一次；ACK必须ID匹配；迟到/重复ACK不能重复完成；至少写五个测试。

然后解释怎样把其中的数据输入换成ROS订阅、输出换成模拟service，而不修改业务状态机。做到这一点，才能说明你掌握的不只是语法名称，而是代码的组织方式和可验证行为。

本书的学习目标不是背下4345行源文件，而是看到任何一个成员、表达式和回调，都能说清它为什么存在、什么时候有效，以及失败时谁负责处理。


<a id="appendix-a"></a>
# 附录 A：同一个符号为什么有不同意思

遇到符号先判断“现在是在声明、表达式、模板、预处理还是字符串中”，不要只看它长什么样。

| 符号 | 情境 | 意义 | 小例子 |
|---|---|---|---|
| `*` | 声明 | 指针类型的一部分 | `double* p` |
| `*` | 一元表达式 | 解引用 | `*p`、`*home`、`*iterator`，后两者可由运算符重载实现 |
| `*` | 二元表达式 | 乘法 | `speed * dt` |
| `&` | 声明 | 左值引用 | `const Point3& p` |
| `&` | 一元表达式 | 取地址 | `&height` |
| `&` | 二元表达式 | 按位与 | `mask & 1U` |
| `&` | Lambda捕获列表 | 按引用捕获 | `[&count]` |
| `&&` | 表达式 | 逻辑与且短路 | `ready && valid` |
| `&&` | 声明 | 右值引用/满足条件时为转发引用 | `T&& value`，具体要看T如何确定 |
| `::` | 名称查找 | 命名空间或类作用域 | `std::vector`、`State::SEARCH` |
| `.` | 对象表达式 | 成员访问 | `point.x` |
| `->` | 指针/重载访问 | 间接成员访问 | `msg->data`、`home_->x` |
| `->` | 函数/Lambda声明 | 尾置返回类型 | `auto f() -> double` |
| `:` | 类定义 | 继承基类列表开始 | `class Node : public Base` |
| `:` | 构造函数 | 初始化列表开始 | `Counter() : count_(0)` |
| `:` | 条件表达式 | `?:`的第三操作数前 | `valid ? x : y` |
| `:` | 范围for | 分隔循环变量与范围 | `for (auto& p : route)` |
| `:` | switch标签/访问控制 | 标签或访问区域 | `case State::WAIT:`、`public:` |
| `< >` | 比较表达式 | 小于、大于 | `error < tolerance` |
| `< >` | 模板使用 | 包围类型/非类型模板实参 | `vector<Point3>` |
| `< >` | include指令 | 头文件名定界 | `#include <vector>` |
| `[]` | 数组/容器表达式 | 下标访问 | `route[index]` |
| `[]` | 声明 | 数组类型 | `double values[3]` |
| `[]` | Lambda | 捕获列表 | `[this]` |
| `[[]]` | 声明属性 | 标准属性 | `[[nodiscard]]`、`[[maybe_unused]]` |
| `{}` | 语句 | 作用域代码块 | `if (ready) { ... }` |
| `{}` | 初始化 | 列表初始化/聚合初始化 | `Point3{1,2,3}` |
| `{}` | 赋值右侧 | 用空列表构造合适的值再赋值 | `future = {}` |
| `()` | 函数名之后 | 调用 | `tick()` |
| `()` | 声明 | 参数列表 | `double f(double x)` |
| `()` | 表达式周围 | 指定结合 | `(a+b)*c` |
| `<<` | 整数表达式 | 左移 | `1U << 2U` |
| `<<` | 输出流表达式 | 流插入 | `std::cout << value` |
| `>>` | 整数表达式 | 右移 | `mask >> 1U` |
| `>>` | 输入流表达式 | 流提取 | `std::cin >> height` |
| `>>` | 嵌套模板 | 两层模板结束 | `vector<vector<int>>` |
| `!` | 布尔表达式 | 逻辑非 | `!ready` |
| `~` | 整数表达式 | 按位取反 | `~mask` |
| `~` | 类成员名 | 析构函数前缀 | `~Counter()` |

**声明阅读练习**：`const T* const p`从p往外读：p是不可改绑的指针，指向只允许通过该访问路径读取的T。`const T&`则是const引用，不是“一个不可改绑的引用变量”——普通引用本来就不能改绑。

<a id="appendix-b"></a>
# 附录 B：标准库与ROS常用接口速查

这里不是完整标准库手册，而是本书和v11所需接口的可执行阅读清单。括号中的类型T表示“替换成实际类型”；这些签名是为了阅读而简化，精确重载以C++17草案或Humble接口为准。[R03][R17]

## B.1 数值、字符串与类型

| 接口/关键字 | 头文件或类别 | 输入→输出/作用 | 必须注意 |
|---|---|---|---|
| `std::isfinite(x)` | `<cmath>` | 浮点→bool | 拒绝NaN和无穷；不是范围检查 |
| `std::abs(x)` | `<cmath>`等 | 数值→绝对值 | 先考虑类型，整数最小值绝对值可能不可表示 |
| `std::hypot(x,y)` | `<cmath>` | 两边→欧氏长度 | 较手写平方求和有更好的数值缩放处理；输入仍需合法 |
| `std::sqrt(x)` | `<cmath>` | 非负值→平方根 | 对非法域不能期待正常实数 |
| `std::sin/cos(x)` | `<cmath>` | 弧度→三角值 | 不直接传角度制数值 |
| `std::atan2(y,x)` | `<cmath>` | 两分量→角度 | 注意参数顺序，利用象限信息 |
| `std::asin(x)` | `<cmath>` | [-1,1]→弧度 | 四元数计算先验证再对微小数值越界作处理 |
| `std::fmod(x,y)` | `<cmath>` | 浮点余数 | 负数结果不自动落入[0,y)；除数不可零 |
| `std::clamp(v,lo,hi)` | `<algorithm>` | 引用形式返回边界内值 | lo≤hi是前提；NaN不能靠它充分验证；别长期保存指向临时量的返回引用 |
| `std::min/max(a,b)` | `<algorithm>` | 较小/较大值的引用形式 | 类型推导一致性、临时对象引用寿命 |
| `numeric_limits<T>::infinity()` | `<limits>` | 返回该类型支持的无穷表示 | 不是任意类型都支持；区分max与infinity |
| `numeric_limits<T>::max()` | `<limits>` | 最大有限值 | 浮点min不是最负数；lowest才表示最低有限值 |
| `std::string::size()` | `<string>` | 返回字符单元数 | UTF-8通常是字节数，不是汉字数量 |
| `c_str()` | `<string>` | 返回以零结束的字符指针 | 字符串修改/销毁后可能失效 |
| `std::to_string(value)` | `<string>` | 数值→string | 不用于精确控制小数格式 |
| `std::getline(stream,line)` | `<string>` | 读一行，更新line并返回流 | 可用于条件判断；关注提取换行后的行为 |
| `std::istringstream` | `<sstream>` | 从字符串解析 | 检查失败与尾部垃圾字符 |
| `std::fixed/setprecision(n)` | `<iomanip>`等 | 设置流格式 | 会影响同一流后续输出，直到更改 |
| `static_cast<T>(x)` | 语言 | 显式类型转换 | 不是验证，不保证值转换后仍合业务意义 |
| `decltype(expr)` | 语言 | 表达式→类型 | 某些括号会改变推导结果 |
| `static_assert(cond)` | 语言 | 编译期条件检查 | cond需满足常量表达式要求 |

## B.2 容器与算法

| 接口 | 作用 | 前提/副作用 |
|---|---|---|
| `vector<T>` | 连续动态序列 | 扩容可能搬迁元素 |
| `deque<T>` | 双端序列 | 不保证整体连续；迭代器规则不等同vector |
| `array<T,N>` | 编译期固定数量元素 | N是类型的一部分 |
| `size()/empty()` | 长度/是否空 | size不是capacity |
| `reserve(n)` | 预留vector容量 | 不生成n个可访问元素 |
| `resize(n)` | 改变元素数量 | 可能构造或销毁元素 |
| `push_back(x)` | 末尾插入一个元素 | 可能复制或移动并引起重分配 |
| `emplace_back(args...)` | 用参数构造末尾元素 | 不保证比push_back永远快，也不免除扩容影响 |
| `front()/back()` | 返回首/末元素引用 | 先保证非空 |
| `operator[](i)` | 不做一般边界异常检查的访问 | 先保证i<size；map的[]有不同插入语义 |
| `at(i)` | 带越界异常的访问 | 仍需设计如何处理异常 |
| `pop_front()/pop_back()` | 销毁首/末元素 | 对应容器支持且非空；不返回已移除元素 |
| `clear()` | 移除全部元素 | 之前元素引用失效；vector容量未必归零 |
| `erase(it)` | 删除位置上的元素并返回后继iterator | 按具体容器处理失效规则 |
| `begin()/end()` | 半开范围起止 | end不可解引用 |
| `find(first,last,value)` | 查找等值元素 | 没找到返回last |
| `find_if(first,last,pred)` | 查找首个谓词为真元素 | 谓词应接受合适元素类型 |
| `any_of` | 是否至少一项满足 | 空范围false |
| `all_of` | 是否所有项满足 | 空范围true |
| `none_of` | 是否没有项满足 | 空范围true |
| `lower_bound` | 第一个不小于目标的位置 | 输入按对应比较关系有序/分区 |
| `sort` | 重新排列元素 | 需随机访问迭代器、严格弱序比较器 |
| `stable_sort` | 对等元素保留原相对顺序 | 仍需合法比较器 |
| `remove_if` | 把保留元素移到前面并给出逻辑新末端 | 不改变vector.size；需配合erase |
| `accumulate` | 按初值类型累计 | 初值0与0.0会影响累计类型 |
| `map::find` | 有序映射查询 | 不因查询缺项而插入 |
| `map::operator[]` | 获取映射值，缺失时可能创建 | 不是纯只读查询 |
| `set::insert` | 插入不重复键 | 重复插入不增加元素数 |

## B.3 生命周期与异步

| 接口 | 作用 | 最容易误解的点 |
|---|---|---|
| `std::move(x)` | 把表达式转换成可用于移动的值类别 | 它本身不搬资源，不删容器元素 |
| `std::forward<T>(x)` | 在适当模板上下文保留转发语义 | 不是比move更高级的通用替代 |
| `make_unique<T>(...)` | 创建独占所有权对象 | unique_ptr不可复制，可移动 |
| `make_shared<T>(...)` | 创建共享所有权对象 | 多句柄共享对象，不是多份T |
| `ptr.get()` | 获得借用裸指针 | 不转移所有权，禁止重复delete |
| `unique_ptr::release()` | 放弃管理并返回裸指针 | 不等于释放对象内存；新所有者必须接手 |
| `reset()` | 改变/清空所持资源 | optional、shared_ptr、unique_ptr各有具体语义 |
| `weak_ptr::lock()` | 尝试取得临时共享所有权 | 返回空时对象已不存在 |
| `optional<T>` | 就地管理有/无一个T | 无值并不等于T为0 |
| `has_value()` | 是否有值 | 不判断数据新鲜度 |
| `value_or(fallback)` | 按值取实际值或备用值 | 不适合用无意义默认值掩盖必需输入缺失 |
| `std::function<R(Args...)>` | 类型擦除的可调用对象包装 | 空时调用会抛bad_function_call；可能有额外开销 |
| `std::bind` | 绑定函数和部分参数 | 成员函数还需对象；引用需显式考虑ref/cref |
| `std::chrono::steady_clock` | 单调时钟 | 不用来表达UTC日历时间 |
| `duration<double>(delta).count()` | 时间差转double秒 | count单位来自duration类型 |
| `std::promise<T>` | 提供共享结果的一端 | 只能完成一次结果设置，异常是另一种完成方式 |
| `std::future<T>` | 单个结果读取句柄 | 不可复制，get通常消费状态 |
| `std::shared_future<T>` | 可复制的结果读取句柄 | 共享同一个状态，不自动复制远端任务 |
| `valid()` | 是否关联共享状态 | 不等于ready |
| `wait_for(0s)` | 立即查询等待状态 | 可能ready/timeout/deferred，依具体来源 |
| `get()` | 读取结果/异常 | 未准备好可能等待；可能抛异常 |
| `share()` | 把future转换成shared_future | 原future不再保有所转移的状态 |
| `std::lock_guard` | 作用域内持锁 | 对象要有名字并覆盖临界区，临时对象会马上销毁 |
| `std::thread::join()` | 等待线程结束 | 可阻塞，不要无期限放进高频控制回调 |

## B.4 ROS2 Humble的阅读入口

| 接口 | 在节点中的作用 | 与C++知识的联系 |
|---|---|---|
| `rclcpp::init(argc,argv)` | 初始化上下文 | main命令行参数、函数调用 |
| `rclcpp::spin(node)` | 驱动执行器处理节点事件 | 对象寿命、事件循环 |
| `rclcpp::shutdown()` | 关闭ROS上下文 | 显式资源/进程退出流程，不是飞机降落 |
| `create_wall_timer(period,callback)` | 登记定时回调 | chrono、可调用对象、shared_ptr |
| `create_subscription<Msg>(...)` | 建立类型化订阅 | 模板、回调、消息指针 |
| `create_publisher<Msg>(...)` | 建立类型化发布器 | 模板、对象资源 |
| `publisher->publish(msg)` | 发布消息 | 箭头成员访问、参数传递 |
| `declare_parameter<T>` | 声明参数及默认值 | 编译期类型与运行时值 |
| `get_parameter(...).as_double()` | 读取具体类型的参数值 | 链式调用、运行时类型契约 |
| `create_client<Service>` | 建立服务客户端 | 类模板、Request/Response嵌套类型 |
| `service_is_ready()` | 当前是否发现服务可用 | 查询而非保证随后的网络成功 |
| `async_send_request(request)` | 提交请求并返回future/编号 | 异步共享状态 |
| `remove_pending_request(id)` | 清理客户端等待状态 | 本地管理，不代表取消远端动作 |
| `get_logger()` | 返回日志对象 | 基类成员调用 |
| `RCLCPP_INFO_THROTTLE` | 节流日志 | 宏、格式参数、clock |
| `Node::now()` | 节点ROS时间 | 可能受use_sim_time影响；不同于steady_clock |

<a id="appendix-c"></a>
# 附录 C：与原版教程相比，哪些地方明确澄清或改变了

本表的“原版”指用户上传教程[B0]及本项目此前的概括性解释；“基准源码”指[B1]。讲清差异，不等于已经修改了真实任务程序。

| 主题 | 本次处理 | 类型 |
|---|---|---|
| 初学顺序 | 把指针、引用、生命周期、复制/移动放在智能指针和回调之前 | 教学重组 |
| 第一份状态机 | 不再要求尚未学会Lambda/chrono的读者先读完整异步FSM | 教学重组 |
| 练习可执行性 | 补全main、头文件、构建命令、真实捕获输出和错误路径 | 教学补充 |
| `value_at` | 明确非空越界返回back，空才返回fallback | 基准事实澄清 |
| `const SharedPtr` | 区分句柄const与消息const | C++语义澄清 |
| 时间夹逼边界 | 首样本等号拒绝，中/末样本等号可接受 | 基准事实澄清 |
| 视觉时间同步 | 明确主机接收时刻不等价硬件采样时刻 | 基准事实与能力边界澄清 |
| 旋转描述 | 固定列向量RzRyRx约定，避免“新的Y轴”等混合表述 | 数学约定澄清 |
| `std::move` | 转换表达式值类别，不是自动搬走/删除 | C++语义澄清 |
| future | 区分valid/ready/get异常、deferred和async析构等待 | C++语义补充 |
| ROS服务超时 | 新教学客户端保存request_id并remove_pending_request | 教学实现改进，原cpp未修改 |
| 稳定门 | 新教学StableGate加入最大检查间隔与时间回跳重置 | 教学实现改进，非逐字复刻 |
| 扁平三元组访问 | 新教学函数先做除法判界避免index×3溢出 | 教学实现改进 |
| slewing dt | 按源码说明实际测量dt与默认回退，而非永远固定50ms | 基准事实澄清 |
| tick发布顺序 | 明确v11先发布已有目标，再更新状态目标 | 基准事实澄清 |
| 机构初始化门禁 | 不把WAIT_ARM的ACK门禁移述到navigation_ready_to_lock | 基准事实澄清 |
| RELEASE补偿速度 | 区分发令前slew和pending/started后的直接赋值 | 基准事实澄清 |
| RELEASE门禁时机 | 指出完整门禁主要在发令前，不能撤销已物理发生动作 | 基准事实与能力边界澄清 |
| 成功语义 | ACK、软件流程完成、物理脱离、比赛得分分别说明 | 工程语义澄清 |
| O4/12m/s | 始终标为讨论方案，不标为上传v11现成功能 | 版本边界保持 |
| 教学Tracker | 用较短EMA关联实现，明确未复刻全部v11质量/重绑定逻辑 | 显式教学简化 |
| 教学LabMission | 假观测、小航点、200ms稳定门，不接任何真实控制接口 | 显式教学简化 |

<a id="appendix-d"></a>
# 附录 D：配套纯C++核心模块完整文件

下面是第36—42个例程依赖的完整实现。它们都是本书教学代码，不是从v11提取出的可直接替换飞行节点。阅读时可以先看头文件了解接口，再只进入当前章节需要的函数。

## D.1 `core.hpp`

**文件：`include/cuadc_lab/core.hpp`**

```cpp
#ifndef CUADC_LAB_CORE_HPP
#define CUADC_LAB_CORE_HPP

#include <chrono>
#include <cstddef>
#include <deque>
#include <optional>
#include <vector>

namespace cuadc_lab {

// 纯教学算法：没有网络、串口、ROS 或执行器接口。
constexpr double pi = 3.14159265358979323846;
struct Point3 { double x = 0.0; double y = 0.0; double z = 0.0; };
struct Attitude { double roll = 0.0; double pitch = 0.0; double yaw = 0.0; };

bool finite(const Point3& point);
double radians(double degrees);
double normalize_angle(double value);
double distance_xy(const Point3& left, const Point3& right);
Point3 rotate_body_to_local(const Point3& body, const Attitude& attitude);
Point3 field_to_local(const Point3& home, double yaw, const Point3& field);
Point3 local_to_field(const Point3& home, double yaw, const Point3& local);
Point3 release_point(const Point3& vehicle, const Attitude& attitude, const Point3& offset);
Point3 desired_vehicle(const Point3& bucket, const Attitude& attitude,
                       const Point3& offset, double vehicle_z);

// 保留 v11 的实际策略：非空但越界时返回末项；只有空容器才返回 fallback。
template<typename T>
T value_at(const std::vector<T>& values, std::size_t index, const T& fallback)
{
    if (index < values.size()) {
        return values[index];
    }
    return values.empty() ? fallback : values.back();
}

// 与 v11 的扁平三元组思想相同；先除法判界，避免 index*3 无符号溢出。
std::optional<Point3> vector3_at_checked(const std::vector<double>& values, std::size_t index);

using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;
using Milliseconds = std::chrono::milliseconds;
Time at_ms(long long elapsed_ms);

class StableGate {
public:
    StableGate(Milliseconds required, Milliseconds max_gap);
    bool update(bool candidate, Time now);
    void reset();
private:
    Milliseconds required_;
    Milliseconds max_gap_;
    std::optional<Time> since_;
    std::optional<Time> last_;
};

struct Segment {
    Point3 start;
    Point3 end;
    double duration_s = 1.0;
    Point3 sample(double elapsed_s) const;
};
Segment make_segment(const Point3& start, const Point3& end,
                     double max_speed_m_s, double min_duration_s = 1.0);
Point3 slew(const Point3& previous, const Point3& desired, double max_speed_m_s, double dt_s);

struct NavigationSample {
    double stamp_s = 0.0; // 同一个教学时钟域内、从实验开始累计的秒数。
    Point3 position;
    Attitude attitude;
};
std::optional<NavigationSample> interpolate_navigation(
    const std::deque<NavigationSample>& history, double stamp_s, double max_gap_s = 0.05);

double median(std::vector<double> values);
double mad(const std::vector<double>& values);

struct Detection { Point3 position; double diameter = 0.0; };
struct Track {
    std::size_t id = 0;
    Point3 position;
    double diameter = 0.0;
    std::size_t confirmations = 0;
    double last_seen_s = 0.0;
};
class Tracker {
public:
    void update(const std::vector<Detection>& frame, double now_s);
    const std::vector<Track>& tracks() const { return tracks_; }
    std::vector<Track> select_two(const Point3& vehicle, double now_s, bool route_finished) const;
private:
    std::vector<Track> tracks_;
    std::size_t next_id_ = 1;
    std::optional<double> last_update_s_;
};

} // namespace cuadc_lab
#endif
```

## D.2 `core.cpp`

**文件：`src/core.cpp`**

```cpp
#include "cuadc_lab/core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace cuadc_lab {
namespace {
void require_finite(double value)
{
    if (!std::isfinite(value)) {
        throw std::invalid_argument("non-finite scalar");
    }
}
void require_point(const Point3& point)
{
    if (!finite(point)) {
        throw std::invalid_argument("non-finite point");
    }
}
Point3 add(const Point3& a, const Point3& b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
}

bool finite(const Point3& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}
double radians(double degrees)
{
    require_finite(degrees);
    return degrees * pi / 180.0;
}
double normalize_angle(double value)
{
    require_finite(value);
    return std::atan2(std::sin(value), std::cos(value));
}
double distance_xy(const Point3& left, const Point3& right)
{
    require_point(left);
    require_point(right);
    return std::hypot(left.x - right.x, left.y - right.y);
}
Point3 rotate_body_to_local(const Point3& body, const Attitude& a)
{
    require_point(body);
    require_finite(a.roll);
    require_finite(a.pitch);
    require_finite(a.yaw);
    const double cr = std::cos(a.roll), sr = std::sin(a.roll);
    const double cp = std::cos(a.pitch), sp = std::sin(a.pitch);
    const double cy = std::cos(a.yaw), sy = std::sin(a.yaw);
    const double xr = body.x;
    const double yr = cr * body.y - sr * body.z;
    const double zr = sr * body.y + cr * body.z;
    const double xp = cp * xr + sp * zr;
    const double yp = yr;
    const double zp = -sp * xr + cp * zr;
    return {cy * xp - sy * yp, sy * xp + cy * yp, zp};
}
Point3 field_to_local(const Point3& home, double yaw, const Point3& field)
{
    require_point(home);
    return add(home, rotate_body_to_local(field, {0.0, 0.0, yaw}));
}
Point3 local_to_field(const Point3& home, double yaw, const Point3& local)
{
    require_point(home);
    require_point(local);
    return rotate_body_to_local(
        {local.x - home.x, local.y - home.y, local.z - home.z}, {0.0, 0.0, -yaw});
}
Point3 release_point(const Point3& vehicle, const Attitude& attitude, const Point3& offset)
{
    require_point(vehicle);
    return add(vehicle, rotate_body_to_local(offset, attitude));
}
Point3 desired_vehicle(const Point3& bucket, const Attitude& attitude,
                       const Point3& offset, double vehicle_z)
{
    require_point(bucket);
    require_finite(vehicle_z);
    const auto rotated = rotate_body_to_local(offset, attitude);
    return {bucket.x - rotated.x, bucket.y - rotated.y, vehicle_z};
}
std::optional<Point3> vector3_at_checked(const std::vector<double>& values, std::size_t index)
{
    if (index >= values.size() / 3U) {
        return std::nullopt;
    }
    const std::size_t offset = index * 3U;
    Point3 result{values[offset], values[offset + 1U], values[offset + 2U]};
    return finite(result) ? std::optional<Point3>{result} : std::nullopt;
}

Time at_ms(long long elapsed_ms) { return Time{} + Milliseconds(elapsed_ms); }
StableGate::StableGate(Milliseconds required, Milliseconds max_gap)
    : required_(required), max_gap_(max_gap)
{
    if (required_ < Milliseconds::zero() || max_gap_ <= Milliseconds::zero()) {
        throw std::invalid_argument("invalid gate duration");
    }
}
void StableGate::reset() { since_.reset(); last_.reset(); }
bool StableGate::update(bool candidate, Time now)
{
    if (last_ && (now < *last_ || now - *last_ > max_gap_)) {
        since_.reset();
    }
    last_ = now;
    if (!candidate) {
        since_.reset();
        return false;
    }
    if (!since_) {
        since_ = now;
    }
    return now - *since_ >= required_;
}

Segment make_segment(const Point3& start, const Point3& end,
                     double max_speed_m_s, double min_duration_s)
{
    require_point(start);
    require_point(end);
    require_finite(max_speed_m_s);
    require_finite(min_duration_s);
    if (max_speed_m_s <= 0.0 || min_duration_s <= 0.0) {
        throw std::invalid_argument("speed and duration must be positive");
    }
    const double length = std::hypot(std::hypot(end.x - start.x, end.y - start.y),
                                     end.z - start.z);
    return {start, end, std::max(min_duration_s, length / (0.8 * max_speed_m_s))};
}
Point3 Segment::sample(double elapsed_s) const
{
    require_point(start);
    require_point(end);
    require_finite(elapsed_s);
    require_finite(duration_s);
    if (duration_s <= 0.0) {
        throw std::invalid_argument("duration must be positive");
    }
    const double u = std::clamp(elapsed_s / duration_s, 0.0, 1.0);
    double progress = 0.0;
    if (u < 0.2) {
        progress = 3.125 * u * u;
    } else if (u > 0.8) {
        const double remaining = 1.0 - u;
        progress = 1.0 - 3.125 * remaining * remaining;
    } else {
        progress = 1.25 * u - 0.125;
    }
    return {start.x + progress * (end.x - start.x),
            start.y + progress * (end.y - start.y),
            start.z + progress * (end.z - start.z)};
}
Point3 slew(const Point3& previous, const Point3& desired, double max_speed_m_s, double dt_s)
{
    require_point(previous);
    require_point(desired);
    require_finite(max_speed_m_s);
    require_finite(dt_s);
    if (max_speed_m_s < 0.0 || dt_s < 0.0) {
        throw std::invalid_argument("negative speed or dt");
    }
    const double dx = desired.x - previous.x;
    const double dy = desired.y - previous.y;
    const double dz = desired.z - previous.z;
    const double length = std::hypot(std::hypot(dx, dy), dz);
    if (length == 0.0) {
        return desired;
    }
    const double ratio = std::min(1.0, max_speed_m_s * dt_s / length);
    return {previous.x + ratio * dx, previous.y + ratio * dy, previous.z + ratio * dz};
}

std::optional<NavigationSample> interpolate_navigation(
    const std::deque<NavigationSample>& history, double stamp_s, double max_gap_s)
{
    if (!std::isfinite(stamp_s) || !std::isfinite(max_gap_s) || max_gap_s <= 0.0 || history.size() < 2U) {
        return std::nullopt;
    }
    // 教学防御：不把无序或重复时间戳交给 lower_bound。
    for (std::size_t index = 0; index < history.size(); ++index) {
        const auto& sample = history[index];
        if (!std::isfinite(sample.stamp_s) || !finite(sample.position) ||
            !std::isfinite(sample.attitude.roll) || !std::isfinite(sample.attitude.pitch) ||
            !std::isfinite(sample.attitude.yaw) ||
            (index > 0 && sample.stamp_s <= history[index - 1U].stamp_s)) {
            return std::nullopt;
        }
    }
    auto upper = std::lower_bound(history.begin(), history.end(), stamp_s,
        [](const NavigationSample& sample, double value) { return sample.stamp_s < value; });
    // 保留 v11 边界语义：首样本精确命中拒绝；中间/末样本精确命中可接受。
    if (upper == history.begin() || upper == history.end()) {
        return std::nullopt;
    }
    const auto& after = *upper;
    const auto& before = *(upper - 1);
    const double gap = after.stamp_s - before.stamp_s;
    if (gap > max_gap_s) {
        return std::nullopt;
    }
    const double ratio = (stamp_s - before.stamp_s) / gap;
    auto lerp = [ratio](double a, double b) { return a + ratio * (b - a); };
    auto lerp_angle = [ratio](double a, double b) {
        return normalize_angle(a + ratio * normalize_angle(b - a));
    };
    return NavigationSample{stamp_s,
        {lerp(before.position.x, after.position.x),
         lerp(before.position.y, after.position.y),
         lerp(before.position.z, after.position.z)},
        {lerp_angle(before.attitude.roll, after.attitude.roll),
         lerp_angle(before.attitude.pitch, after.attitude.pitch),
         lerp_angle(before.attitude.yaw, after.attitude.yaw)}};
}

double median(std::vector<double> values)
{
    if (values.empty()) {
        throw std::invalid_argument("median of empty input");
    }
    for (double value : values) { require_finite(value); }
    std::sort(values.begin(), values.end());
    const auto middle = values.size() / 2U;
    return values.size() % 2U == 0U ?
        values[middle - 1U] * 0.5 + values[middle] * 0.5 : values[middle];
}
double mad(const std::vector<double>& values)
{
    const double center = median(values);
    std::vector<double> deviations;
    deviations.reserve(values.size());
    for (double value : values) { deviations.push_back(std::abs(value - center)); }
    return median(std::move(deviations));
}

void Tracker::update(const std::vector<Detection>& frame, double now_s)
{
    require_finite(now_s);
    if (last_update_s_ && now_s <= *last_update_s_) {
        throw std::invalid_argument("frame time must increase");
    }
    last_update_s_ = now_s;
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
        [now_s](const Track& track) { return now_s - track.last_seen_s > 120.0; }), tracks_.end());
    std::vector<Detection> detections;
    for (const auto& detection : frame) {
        if (finite(detection.position) && std::isfinite(detection.diameter) &&
            detection.diameter >= 0.08 && detection.diameter <= 0.35) {
            detections.push_back(detection);
        }
    }
    struct Association { std::size_t track; std::size_t detection; double cost; };
    std::vector<Association> choices;
    for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
        for (std::size_t di = 0; di < detections.size(); ++di) {
            const double position_delta = distance_xy(tracks_[ti].position, detections[di].position);
            const double diameter_delta = std::abs(tracks_[ti].diameter - detections[di].diameter);
            if (position_delta <= 0.45 && diameter_delta <= 0.08) {
                choices.push_back({ti, di, position_delta / 0.45 + diameter_delta / 0.08});
            }
        }
    }
    std::sort(choices.begin(), choices.end(), [](const Association& a, const Association& b) {
        if (a.cost != b.cost) { return a.cost < b.cost; }
        if (a.track != b.track) { return a.track < b.track; }
        return a.detection < b.detection;
    });
    // 故意不用 vector<bool>，让新人先掌握普通元素引用语义。
    std::vector<unsigned char> track_used(tracks_.size(), 0);
    std::vector<unsigned char> detection_used(detections.size(), 0);
    for (const auto& choice : choices) {
        if (track_used[choice.track] != 0 || detection_used[choice.detection] != 0) { continue; }
        auto& track = tracks_[choice.track];
        const auto& detection = detections[choice.detection];
        if (now_s - track.last_seen_s > 0.6) {
            track.position = detection.position;
            track.diameter = detection.diameter;
            track.confirmations = 1;
        } else {
            constexpr double alpha = 0.25;
            track.position.x += alpha * (detection.position.x - track.position.x);
            track.position.y += alpha * (detection.position.y - track.position.y);
            track.position.z += alpha * (detection.position.z - track.position.z);
            track.diameter += alpha * (detection.diameter - track.diameter);
            ++track.confirmations;
        }
        track.last_seen_s = now_s;
        track_used[choice.track] = 1;
        detection_used[choice.detection] = 1;
    }
    for (std::size_t di = 0; di < detections.size(); ++di) {
        if (detection_used[di] == 0) {
            const auto& detection = detections[di];
            tracks_.push_back({next_id_++, detection.position, detection.diameter, 1, now_s});
        }
    }
}
std::vector<Track> Tracker::select_two(const Point3& vehicle, double now_s, bool route_finished) const
{
    require_point(vehicle);
    require_finite(now_s);
    std::vector<Track> candidates;
    for (const auto& track : tracks_) {
        const double age = now_s - track.last_seen_s;
        if (track.confirmations >= 3U && age >= 0.0 && age <= 10.0) {
            candidates.push_back(track);
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](const Track& a, const Track& b) {
        if (a.confirmations != b.confirmations) { return a.confirmations > b.confirmations; }
        return a.id < b.id;
    });
    std::vector<Track> distinct;
    for (const auto& candidate : candidates) {
        if (std::all_of(distinct.begin(), distinct.end(), [&candidate](const Track& accepted) {
            return distance_xy(candidate.position, accepted.position) >= 0.14;
        })) {
            distinct.push_back(candidate);
        }
    }
    if (distinct.size() < (route_finished ? 2U : 3U)) { return {}; }
    std::sort(distinct.begin(), distinct.end(), [](const Track& a, const Track& b) {
        if (a.diameter != b.diameter) { return a.diameter < b.diameter; }
        return a.id < b.id;
    });
    distinct.resize(2U);
    std::sort(distinct.begin(), distinct.end(), [&vehicle](const Track& a, const Track& b) {
        const double da = distance_xy(a.position, vehicle), db = distance_xy(b.position, vehicle);
        return da != db ? da < db : a.id < b.id;
    });
    return distinct; // 返回值快照；以后原 tracks_ 更新不会改变这个 vector。
}

} // namespace cuadc_lab
```

## D.3 `mission.hpp`

**文件：`include/cuadc_lab/mission.hpp`**

```cpp
#ifndef CUADC_LAB_MISSION_HPP
#define CUADC_LAB_MISSION_HPP
#include "cuadc_lab/core.hpp"
#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace cuadc_lab {
// 纯离线实验。小尺度假航点、理想运动模型，不是可刷入或可连接飞机的任务程序。
enum class LabState {
    WAIT, TAKEOFF, COARSE, FINE, WAIT_RELEASE, RELEASE_HOLD,
    WAIT_STOW, RECON, RETURN, LAND, DONE, STOP
};
const char* name(LabState state);
struct Observation {
    Time stamp;
    Point3 position;
    double horizontal_speed = 0;
    double vertical_speed = 0;
    bool ready = false;
};
enum class LabCommand { RELEASE, STOW };
struct Request { std::uint64_t id; LabCommand command; std::size_t payload; };
struct Ack { std::uint64_t id; bool accepted; };

class LabMission {
public:
    explicit LabMission(Time start);
    void tick(Time now, const Observation& observation, std::optional<Ack> ack = std::nullopt);
    std::optional<Request> take_request();
    LabState state() const { return state_; }
    Point3 target() const { return target_; }
    std::size_t completed_payloads() const { return payload_; }
    const std::string& stop_reason() const { return stop_reason_; }
private:
    void enter(LabState next, Time now);
    void stop(std::string reason, Time now);
    void issue(LabCommand command, Time now);
    bool reached(const Observation& observation, double tolerance, Time now);
    LabState state_ = LabState::WAIT;
    Time entered_;
    Point3 home_{};
    Point3 target_{};
    // 教学数据，绝不是比赛场地坐标；本练习不做真实搜索、视觉或识别。
    std::array<Point3, 2> targets_{{{1, 0, 0}, {2, 1, 0}}};
    std::size_t payload_ = 0;
    std::uint64_t next_id_ = 1;
    std::optional<Request> active_;
    std::optional<Request> outgoing_;
    StableGate gate_{Milliseconds(200), Milliseconds(150)};
    std::string stop_reason_;
};
} // namespace cuadc_lab
#endif
```

## D.4 `mission.cpp`

**文件：`src/mission.cpp`**

```cpp
#include "cuadc_lab/mission.hpp"
#include <cmath>
#include <utility>

namespace cuadc_lab {
const char* name(LabState state)
{
    switch (state) {
        case LabState::WAIT: return "WAIT";
        case LabState::TAKEOFF: return "TAKEOFF";
        case LabState::COARSE: return "COARSE";
        case LabState::FINE: return "FINE";
        case LabState::WAIT_RELEASE: return "WAIT_RELEASE";
        case LabState::RELEASE_HOLD: return "RELEASE_HOLD";
        case LabState::WAIT_STOW: return "WAIT_STOW";
        case LabState::RECON: return "RECON";
        case LabState::RETURN: return "RETURN";
        case LabState::LAND: return "LAND";
        case LabState::DONE: return "DONE";
        case LabState::STOP: return "STOP";
    }
    return "UNKNOWN";
}
LabMission::LabMission(Time start) : entered_(start) {}
void LabMission::enter(LabState next, Time now)
{
    state_ = next;
    entered_ = now;
    gate_.reset();
}
void LabMission::stop(std::string reason, Time now)
{
    stop_reason_ = std::move(reason);
    // 教学 STOP 只停止模拟；真实系统还需飞控模式、落地和机构善后协议。
    active_.reset();
    outgoing_.reset();
    enter(LabState::STOP, now);
}
void LabMission::issue(LabCommand command, Time now)
{
    active_ = Request{next_id_++, command, payload_};
    outgoing_ = active_;
    enter(command == LabCommand::RELEASE ? LabState::WAIT_RELEASE : LabState::WAIT_STOW, now);
}
std::optional<Request> LabMission::take_request()
{
    auto result = outgoing_;
    outgoing_.reset();
    return result;
}
bool LabMission::reached(const Observation& observation, double tolerance, Time now)
{
    const bool at_target = distance_xy(observation.position, target_) <= tolerance &&
        std::abs(observation.position.z - target_.z) <= tolerance;
    const bool motion_ok = observation.horizontal_speed <= 0.15 &&
        std::abs(observation.vertical_speed) <= 0.10;
    return gate_.update(at_target && motion_ok, now);
}
void LabMission::tick(Time now, const Observation& obs, std::optional<Ack> ack)
{
    if (state_ == LabState::DONE || state_ == LabState::STOP) { return; }
    if (!finite(obs.position) || !std::isfinite(obs.horizontal_speed) ||
        !std::isfinite(obs.vertical_speed) || obs.horizontal_speed < 0.0 ||
        obs.stamp > now || now - obs.stamp > Milliseconds(250)) {
        stop("observation invalid or stale", now);
        return;
    }
    if (now < entered_) { stop("clock moved backwards", now); return; }
    const auto elapsed = now - entered_;
    if (state_ != LabState::WAIT && elapsed > Milliseconds(20000)) {
        stop("stage timeout", now);
        return;
    }
    switch (state_) {
        case LabState::WAIT:
            if (obs.ready) {
                home_ = obs.position;
                target_ = {home_.x, home_.y, home_.z + 2.0};
                enter(LabState::TAKEOFF, now);
            }
            break;
        case LabState::TAKEOFF:
            if (reached(obs, 0.05, now)) {
                target_ = {home_.x + targets_[payload_].x,
                           home_.y + targets_[payload_].y, home_.z + 1.7};
                enter(LabState::COARSE, now);
            }
            break;
        case LabState::COARSE:
            if (reached(obs, 0.10, now)) {
                target_.z = home_.z + 1.3;
                enter(LabState::FINE, now);
            }
            break;
        case LabState::FINE:
            if (reached(obs, 0.05, now)) { issue(LabCommand::RELEASE, now); }
            break;
        case LabState::WAIT_RELEASE:
        case LabState::WAIT_STOW:
            // 超时优先；迟到 ACK 不能使已经过期的请求在本实验中恢复成功。
            if (elapsed >= Milliseconds(1000)) {
                stop("ACK timeout: physical action unknown", now);
            } else if (ack && active_ && ack->id == active_->id) {
                if (!ack->accepted) { stop("command rejected", now); break; }
                const LabCommand command = active_->command;
                active_.reset();
                if (command == LabCommand::RELEASE) {
                    enter(LabState::RELEASE_HOLD, now);
                } else {
                    ++payload_;
                    if (payload_ < targets_.size()) {
                        target_ = {home_.x + targets_[payload_].x,
                                   home_.y + targets_[payload_].y, home_.z + 1.7};
                        enter(LabState::COARSE, now);
                    } else {
                        target_ = {home_.x + 3.0, home_.y, home_.z + 2.0};
                        enter(LabState::RECON, now);
                    }
                }
            }
            break;
        case LabState::RELEASE_HOLD:
            if (elapsed >= Milliseconds(700)) { issue(LabCommand::STOW, now); }
            break;
        case LabState::RECON:
            if (reached(obs, 0.05, now)) {
                target_ = {home_.x, home_.y, home_.z + 2.0};
                enter(LabState::RETURN, now);
            }
            break;
        case LabState::RETURN:
            if (reached(obs, 0.05, now)) {
                target_ = home_;
                enter(LabState::LAND, now);
            }
            break;
        case LabState::LAND:
            if (reached(obs, 0.02, now)) { enter(LabState::DONE, now); }
            break;
        case LabState::DONE:
        case LabState::STOP:
            break;
    }
}
} // namespace cuadc_lab
```

## D.5 根目录CMake

**文件：`CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.16)
project(cuadc_cpp_basics LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
option(ENABLE_UBSAN "Enable undefined-behaviour checks for GCC/Clang" OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
  if(ENABLE_UBSAN)
    add_compile_options(-fsanitize=undefined -fno-sanitize-recover=all -g)
    add_link_options(-fsanitize=undefined)
  endif()
endif()
find_package(Threads REQUIRED)
add_library(cuadc_lab src/core.cpp src/mission.cpp)
target_include_directories(cuadc_lab PUBLIC include)
enable_testing()
file(GLOB EXAMPLES CONFIGURE_DEPENDS "examples/*.cpp")
foreach(SOURCE IN LISTS EXAMPLES)
  get_filename_component(STEM "${SOURCE}" NAME_WE)
  set(TARGET "ex_${STEM}")
  add_executable(${TARGET} "${SOURCE}")
  target_link_libraries(${TARGET} PRIVATE cuadc_lab Threads::Threads)
  # 输入/argv/输出比对由 tools/verify.py 统一执行。
endforeach()
add_executable(core_tests tests/core_tests.cpp)
target_link_libraries(core_tests PRIVATE cuadc_lab)
add_test(NAME core_tests COMMAND core_tests)
add_test(NAME offline_mission_nominal COMMAND ex_42_mission)
add_test(NAME offline_mission_ack_loss COMMAND ex_42_mission --drop-ack)
```

实际运行时建议先执行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j 4
ctest --test-dir build --output-on-failure
python3 tools/verify.py --build build
```

不要给这些练习添加真实FCU_URL，不要把ROS实验topic重映射到控制话题。它们的测试目标是语言与软件逻辑。

<a id="appendix-e"></a>
# 附录 E：单元测试不是注释——完整测试文件

这些检查不用assert宏，所以不会因为定义NDEBUG就全部消失。测试只覆盖列出的输入与合成情形，不能证明所有可能输入或硬件行为正确。

**文件：`tests/core_tests.cpp`**

```cpp
#include "cuadc_lab/core.hpp"
#include "cuadc_lab/mission.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
int passed = 0, failed = 0;
void check(const std::string& label, bool condition)
{
    if (condition) { ++passed; std::cout << "PASS " << label << '\n'; }
    else { ++failed; std::cerr << "FAIL " << label << '\n'; }
}
bool near(double a, double b, double eps = 1.0e-9) { return std::abs(a - b) <= eps; }
void rejects(const std::string& label, const std::function<void()>& operation)
{
    try { operation(); check(label, false); }
    catch (const std::invalid_argument&) { check(label, true); }
    catch (...) { check(label, false); }
}
}
int main()
{
    using namespace cuadc_lab;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    check("finite point", finite({1, 2, 3}));
    check("NaN rejected", !finite({nan, 0, 0}));
    check("infinity rejected", !finite({0, inf, 0}));
    check("degree conversion", near(radians(180), pi));
    check("angle wrapping", near(normalize_angle(radians(361)), radians(1)));
    rejects("invalid angle", [=] { (void)normalize_angle(nan); });
    check("XY ignores Z", near(distance_xy({0,0,100}, {3,4,-100}), 5));
    rejects("distance invalid input", [=] { (void)distance_xy({nan,0,0}, {}); });
    check("yaw90 x to y", near(rotate_body_to_local({1,0,0}, {0,0,pi/2}).y, 1));
    check("roll90 y to z", near(rotate_body_to_local({0,1,0}, {pi/2,0,0}).z, 1));
    check("pitch90 x to minus z", near(rotate_body_to_local({1,0,0}, {0,pi/2,0}).z, -1));
    const Point3 rotated = rotate_body_to_local({1,2,3}, {.1,.2,.3});
    check("rotation keeps norm", near(std::hypot(std::hypot(rotated.x,rotated.y),rotated.z), std::sqrt(14)));
    const Point3 home{3,4,5}, field{30,-2,1.3};
    const Point3 restored = local_to_field(home, .7, field_to_local(home, .7, field));
    check("field inverse XY", distance_xy(restored, field) < 1e-9);
    check("field inverse Z", near(restored.z, field.z));
    const Point3 bucket{30,2,.3}, offset{.026,-.065,-.32};
    const Attitude tilt{.1,-.07,.4};
    const Point3 vehicle = desired_vehicle(bucket, tilt, offset, 1.3);
    check("rigid outlet aligned", distance_xy(release_point(vehicle,tilt,offset),bucket) < 1e-9);
    check("vehicle Z not outlet Z", near(vehicle.z, 1.3));
    check("horizontal offset sign", near(desired_vehicle(bucket,{},offset,1.3).x, bucket.x-offset.x));
    check("value_at in range", value_at(std::vector<int>{9,10},0,99) == 9);
    check("value_at nonempty fallback is back", value_at(std::vector<int>{9,10},8,99) == 10);
    check("value_at empty fallback", value_at(std::vector<int>{},8,99) == 99);
    check("triplet second", near(vector3_at_checked({1,2,3,4,5,6},1)->x,4));
    check("triplet incomplete", !vector3_at_checked({1,2},0));
    check("triplet huge index", !vector3_at_checked({1,2,3},std::numeric_limits<std::size_t>::max()));
    check("triplet nonfinite", !vector3_at_checked({1,nan,3},0));
    StableGate gate{Milliseconds(200), Milliseconds(100)};
    check("stable first starts", !gate.update(true,at_ms(0)));
    check("stable before threshold", !gate.update(true,at_ms(100)));
    check("stable equality passes", gate.update(true,at_ms(200)));
    check("stable false resets", !gate.update(false,at_ms(250)));
    check("stable restart", !gate.update(true,at_ms(300)));
    check("stable gap resets", !gate.update(true,at_ms(1000)));
    check("stable backwards resets", !gate.update(true,at_ms(900)));
    gate.reset();
    check("stable explicit reset", !gate.update(true,at_ms(5000)));
    check("stable duplicate no advance", !gate.update(true,at_ms(5000)));
    rejects("invalid stable gap", [] { StableGate invalid{Milliseconds(1),Milliseconds(0)}; });
    const auto segment = make_segment({0,0,0},{10,0,0},2);
    check("segment duration", near(segment.duration_s,6.25));
    check("segment start", near(segment.sample(0).x,0));
    check("segment end", near(segment.sample(6.25).x,10));
    check("segment clamp earlier", near(segment.sample(-2).x,0));
    check("segment clamp later", near(segment.sample(100).x,10));
    check("segment midpoint", near(segment.sample(3.125).x,5));
    check("segment accel boundary", near(segment.sample(1.25).x,1.25));
    check("segment decel boundary", near(segment.sample(5).x,8.75));
    check("segment min duration", near(make_segment({},{.1,0,0},2).duration_s,1));
    rejects("segment invalid speed", [] { (void)make_segment({},{1,0,0},0); });
    rejects("segment invalid sample", [=] { (void)segment.sample(nan); });
    double last_x = 0, maximum_speed = 0;
    bool monotonic = true;
    for (int index = 1; index <= 1000; ++index) {
        const double t = segment.duration_s * static_cast<double>(index) / 1000;
        const double x = segment.sample(t).x;
        monotonic = monotonic && x >= last_x;
        maximum_speed = std::max(maximum_speed,(x-last_x)/(segment.duration_s/1000));
        last_x = x;
    }
    check("segment monotonic", monotonic);
    check("segment bounded peak", maximum_speed <= 2 + 1e-9);
    check("slew ordinary", near(slew({}, {1,0,0},.25,.05).x,.0125));
    check("slew no overshoot", near(slew({}, {.001,0,0},.25,.05).x,.001));
    check("slew zero dt", near(slew({}, {1,0,0},.25,0).x,0));
    rejects("slew negative dt", [] { (void)slew({}, {1,0,0},1,-1); });
    const std::deque<NavigationSample> history{
        {0.0,{0,0,0},{0,0,radians(179)}}, {.04,{.04,0,0},{0,0,radians(-179)}},
        {.08,{.08,0,0},{0,0,radians(-177)}}};
    auto middle = interpolate_navigation(history,.02);
    check("history middle exists", middle.has_value());
    check("history position interpolated", middle && near(middle->position.x,.02));
    check("history yaw shortest arc", middle && near(std::abs(middle->attitude.yaw),pi));
    check("history first equality rejected", !interpolate_navigation(history,0));
    check("history last equality allowed", interpolate_navigation(history,.08).has_value());
    check("history no extrapolation", !interpolate_navigation(history,.09));
    check("history no before extrapolation", !interpolate_navigation(history,-.01));
    check("history gap rejected", !interpolate_navigation(history,.02,.03));
    check("history empty", !interpolate_navigation({},.02));
    auto unordered = history; unordered[1].stamp_s = -1;
    check("history unordered", !interpolate_navigation(unordered,.02));
    auto duplicate = history; duplicate[1].stamp_s = 0;
    check("history duplicate", !interpolate_navigation(duplicate,.02));
    auto invalid = history; invalid[1].attitude.pitch = nan;
    check("history attitude nonfinite", !interpolate_navigation(invalid,.02));
    check("median odd", near(median({3,1,2}),2));
    check("median even", near(median({4,1,3,2}),2.5));
    check("MAD outlier resistance", near(mad({.2,.21,.19,.7,.2}),.01));
    rejects("median empty", [] { (void)median({}); });
    rejects("median NaN", [=] { (void)median({1,nan}); });
    Tracker tracker;
    std::vector<Detection> frame{{{1,0,0},.15},{{3,0,0},.2},{{5,0,0},.25}};
    tracker.update(frame,0);
    check("track first confirmation", tracker.tracks().size()==3 && tracker.tracks()[0].confirmations==1);
    check("track no premature selection", tracker.select_two({},0,false).empty());
    tracker.update(frame,.1); tracker.update(frame,.2);
    const auto plan = tracker.select_two({4,0,0},.2,false);
    check("track stable plan", plan.size()==2);
    check("track select smallest two near first", plan.size()==2 && plan[0].id==2 && plan[1].id==1);
    frame[1].position.x=3.2; tracker.update(frame,.3);
    check("track EMA", near(tracker.tracks()[1].position.x,3.05));
    check("track snapshot frozen", near(plan[0].position.x,3));
    check("track stale not selected", tracker.select_two({},20,false).empty());
    rejects("track duplicate time", [&] { tracker.update(frame,.3); });
    tracker.update(frame,1.0);
    check("track long gap reset confirmations", tracker.tracks()[0].confirmations==1);
    Tracker two;
    const std::vector<Detection> pair{{{1,0,0},.15},{{3,0,0},.20}};
    two.update(pair,0);two.update(pair,.1);two.update(pair,.2);
    check("two not enough midroute",two.select_two({},.2,false).empty());
    check("two enough route end",two.select_two({},.2,true).size()==2);
    Tracker bad;bad.update({{{nan,0,0},.2},{{0,0,0},.7}},0);
    check("bad detections ignored",bad.tracks().empty());
    LabMission mission{at_ms(0)};
    Observation obs{at_ms(0),{},0,0,true};
    mission.tick(at_ms(0),obs);
    check("mission ready starts takeoff",mission.state()==LabState::TAKEOFF);
    mission.tick(at_ms(1000),obs);
    check("mission stale stops",mission.state()==LabState::STOP);
    mission.tick(at_ms(1100),{at_ms(1100),{},0,0,true},Ack{1,true});
    check("mission late ACK cannot restart STOP",mission.state()==LabState::STOP);
    LabMission wait{at_ms(0)};wait.tick(at_ms(0),{at_ms(0),{},0,0,false});
    check("mission not ready waits",wait.state()==LabState::WAIT);
    LabMission future{at_ms(0)};future.tick(at_ms(0),{at_ms(100),{},0,0,true});
    check("mission future observation rejected",future.state()==LabState::STOP);
    std::cout << "RESULT passed=" << passed << " failed=" << failed << '\n';
    return failed == 0 ? 0 : 1;
}
```

使用UndefinedBehaviorSanitizer再运行一次：

```bash
cmake -S . -B build_ubsan -DCMAKE_BUILD_TYPE=Debug -DENABLE_UBSAN=ON
cmake --build build_ubsan -j 4
ctest --test-dir build_ubsan --output-on-failure
python3 tools/verify.py --build build_ubsan
```

Sanitizer只能捕获它支持、且本次执行实际触发的问题，不是形式证明。这里没有把“无报错”写成“已达到飞行安全认证”。[R13]

<a id="appendix-f"></a>
# 附录 F：故意写错，再读懂编译器

配套目录compile_fail中的六个文件故意不能完成构建，分别练习缺分号、修改const、复制unique_ptr、作用域外访问、引用绑定错误、缺少函数定义。

它们没有加入正常CMake构建；验证脚本只调用编译器检查是否拒绝，不执行这些程序。

```bash
python3 tools/check_compile_fail.py --output compile_fail_report.txt
```

这里特别保留了“缺少定义”：它能经过部分编译阶段，最终链接失败，帮助你区分语法错误与链接错误。读完报错后自己修正一个副本，再单独编译验证。

<a id="appendix-g"></a>
# 附录 G：实践索引与知识点覆盖

| 例程 | 所在章节 | 文件 | 练习内容 |
|---:|---|---|---|
| [01](#ex01) | [第3章](#ch03) | `01_hello.cpp` | 第一个程序与输出 |
| [02](#ex02) | [第5章](#ch05) | `02_values.cpp` | 变量、初始化与字符串 |
| [03](#ex03) | [第7章](#ch07) | `03_numbers.cpp` | 整数/浮点与转换 |
| [04](#ex04) | [第8章](#ch08) | `04_input.cpp` | 输入验证与退出码 |
| [05](#ex05) | [第10章](#ch10) | `05_branches.cpp` | 条件分支 |
| [06](#ex06) | [第11章](#ch11) | `06_loops.cpp` | 循环和蛇形顺序 |
| [07](#ex07) | [第13章](#ch13) | `07_functions.cpp` | 函数参数与返回值 |
| [08](#ex08) | [第15章](#ch15) | `08_references.cpp` | 值、引用与原对象 |
| [09](#ex09) | [第16章](#ch16) | `09_pointers.cpp` | 指针、改绑与解引用 |
| [10](#ex10) | [第17章](#ch17) | `10_scope.cpp` | 作用域、constexpr和static |
| [11](#ex11) | [第18章](#ch18) | `11_arrays_strings.cpp` | 数组、字符串和argv |
| [12](#ex12) | [第19章](#ch19) | `12_struct_enum.cpp` | struct与enum class |
| [13](#ex13) | [第20章](#ch20) | `13_waypoints.cpp` | 航点验证小项目 |
| [14](#ex14) | [第21章](#ch21) | `14_class.cpp` | 类、成员和对象 |
| [15](#ex15) | [第22章](#ch22) | `15_constructors.cpp` | 构造顺序、explicit与析构 |
| [16](#ex16) | [第23章](#ch23) | `16_raii.cpp` | RAII与异常清理 |
| [17](#ex17) | [第24章](#ch24) | `17_move.cpp` | 复制、移动与对象状态 |
| [18](#ex18) | [第25章](#ch25) | `18_unique.cpp` | unique_ptr所有权 |
| [19](#ex19) | [第26章](#ch26) | `19_shared_weak.cpp` | shared_ptr和weak_ptr |
| [20](#ex20) | [第27章](#ch27) | `20_inheritance.cpp` | 继承、虚函数与override |
| [21](#ex21) | [第28章](#ch28) | `21_callables.cpp` | 可调用对象与bind |
| [22](#ex22) | [第29章](#ch29) | `22_auto_types.cpp` | auto/decltype和编译期类型 |
| [23](#ex23) | [第30章](#ch30) | `23_lambdas.cpp` | Lambda捕获与寿命 |
| [24](#ex24) | [第31章](#ch31) | `24_templates.cpp` | 模板实例化 |
| [25](#ex25) | [第32章](#ch32) | `25_vector.cpp` | vector/array与边界 |
| [26](#ex26) | [第33章](#ch33) | `26_deque.cpp` | deque滑动窗口 |
| [27](#ex27) | [第34章](#ch34) | `27_optional_variant.cpp` | optional/pair/variant |
| [28](#ex28) | [第35章](#ch35) | `28_iterators.cpp` | 迭代器与erase |
| [29](#ex29) | [第37章](#ch37) | `29_algorithms.cpp` | 排序/查找/累计 |
| [30](#ex30) | [第38章](#ch38) | `30_maps.cpp` | map/set与位运算 |
| [31](#ex31) | [第39章](#ch39) | `31_math.cpp` | 数值函数与NaN |
| [32](#ex32) | [第40章](#ch40) | `32_file.cpp` | 文件与严格数字解析 |
| [33](#ex33) | [第43章](#ch43) | `33_chrono.cpp` | chrono时间点/间隔 |
| [34](#ex34) | [第44章](#ch44) | `34_future.cpp` | promise与future |
| [35](#ex35) | [第44章](#ch44) | `35_thread.cpp` | 线程、mutex和deferred |
| [36](#ex36) | [第41章](#ch41) | `36_modules.cpp` | 头源文件链接 |
| [37](#ex37) | [第46章](#ch46) | `37_stable_gate.cpp` | 连续稳定门 |
| [38](#ex38) | [第47章](#ch47) | `38_geometry.cpp` | 三维投放口反解 |
| [39](#ex39) | [第48章](#ch48) | `39_trajectory.cpp` | 轨迹与slew |
| [40](#ex40) | [第49章](#ch49) | `40_history.cpp` | 时间夹逼与角度插值 |
| [41](#ex41) | [第50章](#ch50) | `41_tracking.cpp` | 关联、三桶选二与快照 |
| [42](#ex42) | [第51章](#ch51) | `42_mission.cpp` | 离线任务与丢ACK分支 |

完整例程均列出实际捕获的标准输出。输出中的数值精度、库错误文字等可能因工具链而有差异；不应将字符串逐字不同都当成业务算法错误。本次工具链和实际测试结果在附录J说明。

C++17以外的协程、concepts、ranges、std::span、std::jthread、std::format等不是本书基础代码依赖。它们可在掌握本书后学习，不应在Humble/C++17项目里不加说明地混用。

<a id="appendix-h"></a>
# 附录 H：基准v11函数与实际行号索引

下表以此次读取的基准文件生成。描述用于定位，不代替函数体；原函数名、返回语义和是否更新状态以源码为准。没有把本书教学新增函数混进原工程列表。

| 函数 | 基准行号 | 主要职责 |
|---|---:|---|
| `normalize_angle` | 84—87 | atan2(sin,cos)实现周期角归一化，边界符号取决于数值结果 |
| `normalize_degrees` | 89—93 | 把角度归一化到 0~360° |
| `distance_xy` | 102—105 | 计算平面距离 |
| `distance_xyz` | 107—110 | 计算三维距离 |
| `value_at` | 206—213 | 按值读取；非空越界返回末项，空数组才返回fallback |
| `vector3_at` | 215—223 | 从扁平 double 数组读取第 N 个三维向量 |
| `VisualDropMissionNode` | 230—342 | 构造节点：参数、ROS资源、时间状态与50ms定时器 |
| `declare_parameters` | 344—502 | 注册所有 ROS 参数及默认值 |
| `load_parameters` | 504—814 | 读取、限幅、覆盖可靠性 profile，并建立配置契约 |
| `nav30_ready_callback` | 816—821 | 接收导航 30 Hz 健康状态 |
| `recon_capture_ack_callback` | 823—844 | 接收侦察照片保存 ACK |
| `state_callback` | 846—883 | 更新 FCU 模式/armed，并处理 GUIDED 人工接管 |
| `odom_callback` | 885—967 | 更新位置、速度、姿态、角速度并写导航历史 |
| `extended_state_callback` | 969—978 | 保存 ExtendedState 诊断信息 |
| `compass_callback` | 980—999 | 更新罗盘和航向稳定样本 |
| `navigation_sample_at` | 1001—1105 | 按主机ROS时间历史夹逼插值；等号与最大间隔规则见正文 |
| `steady_age_s` | 1107—1110 | 计算稳定时钟经过秒数 |
| `vision_health_ready` | 1112—1115 | heartbeat + time alignment 都正常 |
| `reset_visual_target_memory_after_clock_rebase` | 1117—1128 | ROS 视觉时钟回跳时清历史和目标 |
| `bucket_callback` | 1130—1203 | 验证视觉时间戳并把视觉帧放入等待队列 |
| `process_pending_vision_frames` | 1205—1233 | 等候后续 odom，处理/拒绝 pending 视觉帧 |
| `process_time_aligned_vision_frame` | 1235—1311 | 把通过时间同步的视觉检测转为 local 并进入 tracking |
| `try_reacquire_active_target_from_frame` | 1313—1376 | ID 重建时按位置/直径/排他门限重绑定 |
| `accepting_visual_targets` | 1378—1385 | SEARCH/COARSE/FINE 接收视觉，RELEASE 冻结 |
| `geometry_yaw_from_odom` | 1387—1396 | 任务绝对 yaw + odom 相对 yaw 漂移 |
| `current_geometry_yaw` | 1398—1401 | 当前几何 yaw |
| `rotate_body_vector_to_local` | 1403—1425 | `Rz*Ry*Rx` 旋转机体系向量 |
| `body_to_local` | 1427—1438 | 使用历史插值位姿将body点转到local；采样时刻精度受打戳策略限制 |
| `local_to_body_current` | 1440—1463 | 当前 local 点反变换到 body |
| `release_point_local_current` | 1465—1475 | 当前真实投放口 local 坐标 |
| `desired_vehicle_pose_for_release_target` | 1477—1500 | 已知桶坐标和当前姿态，反算飞机中心应去哪里 |
| `field_to_local` | 1502—1518 | 任务场地坐标 → local |
| `local_to_field` | 1520—1533 | local → 任务场地坐标 |
| `inside_drop_area` | 1535—1541 | 点是否在投放区（可带 extra） |
| `bucket_already_used` | 1543—1550 | 某位置是否接近已投目标 |
| `target_id_already_used` | 1552—1557 | 某 mission ID 是否已投 |
| `released_position` | 1559—1566 | 检测是否落在已释放目标附近 |
| `median` | 1568—1579 | 中值 |
| `median_absolute_deviation` | 1581—1589 | MAD |
| `smooth_track` | 1591—1652 | 位置/直径/置信度平滑并更新 confirmations |
| `merge_frame_detections` | 1654—1717 | detection ↔ known track 数据关联 |
| `track_ready_for_selection` | 1719—1727 | track 是否达到可选质量 |
| `try_lock_target_plan` | 1729—1822 | 优先 3 桶选 2；航线结束可 2 桶直接用 |
| `best_bucket` | 1824—1846 | 返回当前 payload 对应的已选目标 |
| `odom_fresh` | 1848—1851 | odom 是否新鲜 |
| `compass_fresh` | 1853—1856 | 罗盘是否新鲜 |
| `extended_state_fresh` | 1858—1862 | ExtendedState 是否新鲜 |
| `on_ground_reported` | 1864—1869 | 诊断型 ON_GROUND 判断 |
| `nav30_ready_fresh` | 1871—1875 | 30 Hz 导航锁是否新鲜 |
| `vision_heartbeat_fresh` | 1877—1881 | 视觉 topic 最近是否仍有消息 |
| `vision_time_alignment_fresh` | 1883—1887 | 最近是否有成功完成时间对齐的视觉 |
| `vision_ready_for_takeoff` | 1889—1894 | 检查对齐帧数量及视觉心跳/对齐新鲜度，不保证检测到桶 |
| `state_requires_vision` | 1896—1901 | 当前仅 SEARCH 强制持续视觉 |
| `landing_candidate` | 1903—1919 | odom低相对高度/低速度候选；实际阈值受min限幅 |
| `reset_landing_confirmation` | 1921—1924 | 清落地稳定计时 |
| `update_landing_confirmation` | 1926—1938 | 连续落地确认 |
| `landing_confirmation_ready` | 1940—1946 | 查询连续落地判据；稳定时间受max下限约束 |
| `mark_mission_failure` | 1948—1959 | 保存首个终止原因 |
| `begin_visual_failsafe` | 1961—1988 | 标记任务失败并返航/降落 |
| `monitor_visual_health` | 1990—2045 | SEARCH 视觉短断时悬停等待，超时 failsafe |
| `mean_heading` | 2047—2056 | 对航向样本做圆均值 |
| `heading_stable` | 2058—2074 | 航向窗口是否都在允许波动内 |
| `navigation_ready_to_lock` | 2076—2087 | 配置、连接、未解锁、导航/航向/视觉/静止门禁；舵机ACK另在WAIT_ARM检查 |
| `lock_frame` | 2089—2116 | 锁 home、mission yaw、odom yaw baseline，并构建航线 |
| `build_search_route` | 2118—2150 | 建立 7 lane 搜索航线 |
| `build_recon_route` | 2152—2178 | 基准v11六个固定RGB拍照航点，不是O4新版单线方案 |
| `publish_recon_photo_mode` | 2180—2192 | 通知视觉进程切换 PHOTO_ONLY |
| `request_recon_photo` | 2194—2217 | 发布第 N 个侦察拍照触发 |
| `start_recon_phase` | 2219—2250 | 两瓶完成后启动侦察 |
| `update_recon_transit` | 2252—2268 | 高速转场到侦察入口 |
| `update_recon_descend` | 2270—2288 | 下降进入第一拍照点 |
| `update_recon_scan` | 2290—2312 | 推进 6 点并触发拍照 |
| `start_recon_return_climb` | 2314—2333 | 扫描完成后原地爬升 |
| `update_recon_return_climb` | 2335—2355 | 到返航高度后启动高速回家 |
| `update_recon_return` | 2357—2370 | 到 HOME 上空后进入 LAND |
| `tick` | 2372—2715 | 20 Hz 总调度器 |
| `flight_gate_ok` | 2717—2748 | 飞行中共同安全门禁 |
| `start_search` | 2750—2783 | 初始化搜索阶段和第一个 segment |
| `update_search_lateral_guard` | 2785—2843 | 超出 field_y 软边界时先拉回 |
| `update_search` | 2845—2911 | 锁桶、推进航点、整条航线结束后的 2 桶 fallback |
| `frozen_target_for_payload` | 2913—2935 | 构造第二瓶的冻结任务目标 |
| `desired_release_pose` | 2937—2945 | active bucket 的包装函数 |
| `release_error` | 2947—2955 | 当前投放口到桶的 XY 误差 |
| `reset_release_point_velocity_estimator` | 2957—2962 | 清投放口速度估计器 |
| `update_release_point_velocity_estimator` | 2964—2988 | 用位置差估计投放口速度并低通 |
| `predicted_release_point_local` | 2990—2999 | 按机械延迟预测脱离时投放口位置 |
| `active_target_identity_valid` | 3001—3018 | active bucket 是否仍符合冻结任务目标身份 |
| `inside_release_area` | 3020—3028 | 点是否在最终释放安全内缩区 |
| `retry_alignment_or_return` | 3030—3065 | 单次重试或失败返航 |
| `fine_alignment_visual_valid` | 3068—3082 | 细对准实时视觉是否新鲜且未偏离粗参考 |
| `finish_fine_alignment` | 3084—3125 | 冻结最后可信桶坐标并进入 RELEASE |
| `update_alignment` | 3127—3217 | 1.7 m 粗对准 |
| `update_fine_alignment` | 3219—3344 | 1.3 m 细对准和低空视觉 fallback |
| `update_release` | 3346—3465 | 冻结目标的姿态补偿、发令前门禁和释放/收回请求时序 |
| `finish_payload_release` | 3467—3519 | 按软件ACK/动作时序记完成并切目标/侦察，不直接感知物理脱离 |
| `start_return_home` | 3521—3532 | 故障返航航段 |
| `update_return` | 3534—3541 | 故障返航结束后 LAND |
| `start_segment` | 3543—3559 | 建立一个有时长的梯形速度航段 |
| `slew_target_toward` | 3561—3601 | 测量调用dt，异常时回退0.05s，对动态目标进行3D限步更新 |
| `sample_segment` | 3603—3642 | 按当前时间采样航段 setpoint |
| `segment_complete` | 3644—3648 | 时间到且真实飞机进入终点半径 |
| `initialize_servos_if_ready` | 3650—3675 | 未解锁时把两个舵机初始化到 stowed |
| `send_servo` | 3677—3709 | 发送 DO_SET_SERVO 异步请求 |
| `check_servo_results` | 3711—3789 | 处理机构请求ACK、迟到响应及不确定状态；不是物理位置传感器 |
| `handle_release_abort_stow` | 3791—3819 | failsafe/人工接管后持续尝试安全收回 |
| `publish_setpoint` | 3821—3832 | 发布 `target_` 和锁定 yaw |
| `request_allowed` | 3834—3837 | 统一 1 Hz 请求限频 |
| `mark_request` | 3839—3842 | 记录最近请求时间 |
| `request_takeoff` | 3844—3861 | 发 CommandTOL 起飞 |
| `request_land` | 3863—3876 | 发 CommandTOL 降落 |
| `request_arm` | 3878—3897 | 满足安全门禁后自动解锁 |
| `request_disarm` | 3899—3914 | 落地确认后显式上锁 |
| `check_service_results` | 3916—3983 | 非阻塞检查ACK与超时；原实现部分超时仅清本地future句柄 |
| `relative_altitude` | 3985—3988 | 当前高度相对 home |
| `is_automatic_flight_state` | 3990—3999 | 判断是否处于自动飞行流程 |
| `abort_or_land` | 4001—4006 | armed 则 LAND，否则 ABORT |
| `enter` | 4008—4049 | 统一状态切换和阶段变量初始化 |
| `state_name` | 4051—4077 | State → 字符串 |
| `main` | 4339—4345 | 初始化 ROS、spin 节点、退出 ROS |

<a id="appendix-i"></a>
# 附录 I：如何读“看起来最复杂的一行”

## I.1 例子一：订阅回调绑定

```cpp
// 基准工程的写法模式。
std::bind(&VisualDropMissionNode::odom_callback,
          this, std::placeholders::_1)
```

先问绑定谁：成员函数odom_callback。再问哪个对象：this指向当前节点。最后问还缺什么实参：_1代表调用这个包装器时收到的第一项，也就是消息。

它大致对应`[this](auto msg){ this->odom_callback(msg); }`的调用意图，但std::bind的衰减复制、占位符转发等细节并非与所有Lambda形式完全等价。这里解释具体用途，不泛化成两者永远相同。

## I.2 例子二：异步结果类型

```cpp
rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future;
```

CommandLong是Service类型；Client<Service>是客户端模板实例；SharedFuture是客户端内部公开的类型别名；future是变量。底层承载“将来得到共享Response”的共享状态，不是整个无人机任务。

## I.3 例子三：把元素移出队列

```cpp
PendingVisionFrame frame = std::move(pending_vision_frames_.front());
```

从内向外读：队列front返回首项引用；move把这个表达式标为允许移动；用它初始化一个新的frame对象；队列中的原对象尚未被移除；之后另有pop_front。

## I.4 例子四：三元表达式不是缩写版任意if块

```cpp
const int pwm = release ? 1500 : 1200;
```

先计算release，根据结果只求值一个分支，整个表达式产生一个值供初始化。复杂带副作用的多个动作，不应为了少写行数硬塞进三元表达式。原源码的业务值仍由配置数组读取，不是建议在新程序里直接硬编码控制输出。

## I.5 例子五：一个const成员查询

```cpp
bool odom_fresh() const;
```

返回bool；函数名odom_fresh；无显式实参；末尾const限制通过this修改普通成员；分号说明这里只有声明。它没有说明内部是否看系统时钟，也不表示结果每次都一样。const函数仍可能因时间或外部状态变化返回不同结果。

<a id="appendix-j"></a>
# 附录 J：来源、版本与验证边界

## J.1 用户材料与本文新增内容

<a id="source-b0"></a>
**[B0] 原教程**：用户本次上传的《CUADC_Cpp_ROS2_MAVROS_从零到完整状态机教程.md》。本文保留其“C++17→ROS2→MAVROS→CUADC任务”的目标与主要工程术语，但按用户要求重组先修顺序，扩展基础语法与实践，而不是只做摘要。

<a id="source-b1"></a>
**[B1] 基准源码**：用户此前上传的`cuadc_full_mission_node_3_v11(1).cpp`，版本字符串为`full-mission-v11-geometric-fine-alignment-2026-09-03`。本文直接读取本地文件，关键函数采用原文摘录，其余用明确的解释/推论/教学改进标签区分。

| 材料 | 实际行数 | SHA-256 |
|---|---:|---|
| B0原教程 | 4722 | `527ebbdcc52bc25bd88714e3c0e1c29bac801db9b213dbb6db499f1ffd9dde71` |
| B1基准C++ | 4345 | `fdbdebf85a52b1817faab4ecca17215efd973f7a199de8edd3ddfb3f56fea0f0` |

未获得该机实际YAML、完整视觉节点及本次实飞日志，不能由此证明真实硬件参数、生效配置或飞行表现。本文没有修改生产C++文件、飞控参数或真实飞行服务。

## J.2 语言与框架依据

[R03]采用C++17公开工作草案N4659的HTML版本，用于查语言和标准库规则，不把它冒称为付费ISO正式出版物。标准规则与C++ Core Guidelines工程建议是不同层次。

ROS依据Humble官方文档及rclcpp的humble分支源码。部分docs.ros.org页面在此次访问时返回访问限制，因此同时核对ROS官方GitHub文档源文件；没有声称已经运行ROS API。

| 编号 | 官方/原始资料 | 本文用途 |
|---|---|---|
| [R01] | [GCC：构建阶段与选项](https://gcc.gnu.org/onlinedocs/gcc/Overall-Options.html) | -E/-c/-o、编译与链接 |
| [R02] | [GCC：警告选项](https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html) | 诊断范围，警告不是正确性证明 |
| [R03] | [C++17公开工作草案N4659](https://timsong-cpp.github.io/cppwp/n4659/) | 语言/库规则；HTML排版的公开草案 |
| [R04] | [ISO C++ FAQ：引用](https://isocpp.org/wiki/faq/references) | 引用与参数语义 |
| [R05] | [ISO C++ FAQ：const正确性](https://isocpp.org/wiki/faq/const-correctness) | const访问路径与成员函数 |
| [R06] | [N4659：对象生命周期](https://timsong-cpp.github.io/cppwp/n4659/basic.life) | 生命周期规范 |
| [R07] | [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines) | RAII/所有权等工程建议，非ISO规范 |
| [R08] | [N4659：Lambda](https://timsong-cpp.github.io/cppwp/n4659/expr.prim.lambda) | 捕获与闭包 |
| [R09] | [N4659：vector修改操作](https://timsong-cpp.github.io/cppwp/n4659/vector.modifiers) | 插入/删除与失效规则 |
| [R10] | [N4659：optional](https://timsong-cpp.github.io/cppwp/n4659/optional) | 可选值语义 |
| [R11] | [ROS官方Humble日志文档源](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Logging.rst) | 已核对官方源文件；含格式日志和节流 |
| [R12] | [CMake 3.22官方教程](https://cmake.org/cmake/help/v3.22/guide/tutorial/index.html) | 项目、目标和构建；本书不依赖新版CMake特性 |
| [R13] | [Clang：UndefinedBehaviorSanitizer](https://clang.llvm.org/docs/UndefinedBehaviorSanitizer.html) | UB检测能力边界；本次实际使用GCC对应选项 |
| [R14] | [GNU GDB官方手册](https://sourceware.org/gdb/current/onlinedocs/gdb.html/) | 断点、单步、栈回溯；本书未执行交互调试验证 |
| [R15] | [ROS2时钟与时间设计](https://design.ros2.org/articles/clock_and_time.html) | ROS/system/steady时间语义 |
| [R16] | [N4659：future与promise](https://timsong-cpp.github.io/cppwp/n4659/futures) | 共享状态、等待与异常 |
| [R17] | [rclcpp Humble client.hpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/client.hpp) | 已核对FutureAndRequestId和remove_pending_request |
| [R18] | [ROS官方Humble C++发布订阅文档源](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.rst) | 已核对Node、timer和pub/sub模式 |
| [R19] | [ROS官方Humble执行器文档源](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Executors.rst) | 已核对spin与执行器模型 |
| [R20] | [ROS官方Humble回调组文档源](https://github.com/ros2/ros2_documentation/blob/humble/source/How-To-Guides/Using-callback-groups.rst) | 已核对默认互斥组和死锁条件 |
| [R21] | [Clang：AddressSanitizer](https://clang.llvm.org/docs/AddressSanitizer.html) | 延伸阅读；本次未运行ASan |

核对的rclcpp client.hpp内容blob SHA：`d1751ae12040373b12a4b8fcf2833833e734498f`。这是**文件blob标识，不是仓库commit**；humble分支可能继续更新，部署前应核对实际安装版本。

## J.3 本次实际测试记录


核对日期：2026-09-05。

**已实际执行**

- GCC/G++ 14.2.0（Debian 14.2.0-19），C++17模式。
- CMake 3.31.6，Debug构建。
- 42个纯C++完整例程编译成功，启用Wall/Wextra/Wpedantic/Wconversion/Wshadow，构建日志未见warning/error。
- 42个例程逐一运行并捕获标准输出；另检查基础输入程序的2条非法输入路径。
- 核心测试程序85项检查通过。
- CTest的3项测试通过：核心检查、正常离线任务、丢ACK离线任务。
- 用UndefinedBehaviorSanitizer重新构建并重复上述42例程与3项CTest，未报告所检测到的UB。
- 六个故意错误的编译/链接练习均被工具链拒绝；未执行错误程序。

**未执行**

- 没有在Ubuntu22.04默认GCC版本上再次复测。
- 没有ROS2环境，未执行colcon、ROS通信、服务超时注入、QoS与多线程ROS联调。
- 未运行AddressSanitizer、ThreadSanitizer、Valgrind或形式验证。
- 没有SITL、Gazebo、CUAV飞控、相机、舵机或实飞验证。
- 原生产v11源码未重新编译或改写，本次构建的是新教学例程。

这些测试不能证明所有输入、所有线程调度、所有平台以及任何真实飞行安全性。模块包含输入检查，但没有宣称支持任意接近浮点极限的坐标或无限运行时间。


## J.4 新人交接时应该额外保存什么

保留练习修改前后源码、完整编译命令、编译器版本、输入数据、预期输出、实际输出、最小复现和修复原因。任何涉及真机的后续工作，还需独立保留固件、有效配置、坐标标定、日志与安全验证记录。

一本教程能缩短理解路径，但不能把没有做过的硬件测试变成已经完成的测试。能准确说明“我验证到了哪一层”，同样是工程能力的一部分。


[B0]: #source-b0
[B1]: #source-b1
[R01]: https://gcc.gnu.org/onlinedocs/gcc/Overall-Options.html
[R02]: https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
[R03]: https://timsong-cpp.github.io/cppwp/n4659/
[R04]: https://isocpp.org/wiki/faq/references
[R05]: https://isocpp.org/wiki/faq/const-correctness
[R06]: https://timsong-cpp.github.io/cppwp/n4659/basic.life
[R07]: https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines
[R08]: https://timsong-cpp.github.io/cppwp/n4659/expr.prim.lambda
[R09]: https://timsong-cpp.github.io/cppwp/n4659/vector.modifiers
[R10]: https://timsong-cpp.github.io/cppwp/n4659/optional
[R11]: https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Logging.rst
[R12]: https://cmake.org/cmake/help/v3.22/guide/tutorial/index.html
[R13]: https://clang.llvm.org/docs/UndefinedBehaviorSanitizer.html
[R14]: https://sourceware.org/gdb/current/onlinedocs/gdb.html/
[R15]: https://design.ros2.org/articles/clock_and_time.html
[R16]: https://timsong-cpp.github.io/cppwp/n4659/futures
[R17]: https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/client.hpp
[R18]: https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.rst
[R19]: https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Executors.rst
[R20]: https://github.com/ros2/ros2_documentation/blob/humble/source/How-To-Guides/Using-callback-groups.rst
[R21]: https://clang.llvm.org/docs/AddressSanitizer.html
