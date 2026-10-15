# 从零开始的Linux与开发工具

本篇面向第一次使用Linux终端、编译器和Git的读者。目标是能从空目录建立小实验，解释程序如何启动，读懂失败信息，并把结果交给另一个人复现。

教学基线为Ubuntu 22.04、Bash、Python 3.10、C++17和CMake 3.22。选择固定基线是为了统一例子；其他系统应先核对工具版本。本文不要求ROS、相机或飞控。

所有动手练习在个人目录`~/cuadc_foundation_lab`中完成。命令块不带终端提示符；“预期输出”不是需要输入的命令。占位名称会明确说明，不能直接当作实际设备路径使用。

## 学习路线

| 阶段 | 章节 | 完成后能做什么 |
|---|---|---|
| 找到文件 | 1—7 | 解释终端、路径、文件、权限和文本处理 |
| 启动程序 | 8—13 | 管理环境、解释器、进程、退出码与日志 |
| 构建与调试 | 14—19 | 区分源码、目标文件、链接、CMake与调试器 |
| 接入系统 | 20—23 | 观察网络、设备、时钟与资源 |
| 保存证据 | 24—28 | 用Git、配置与最小复现组织实验 |

读到陌生命令，先看它的作用对象和输出，再决定是否执行。学习终端的核心是理解每一步的状态变化。

---

## 1. Linux、发行版、终端与Shell

Linux内核管理进程、内存、设备等底层资源。发行版把内核、工具、包管理与默认配置组合起来；Ubuntu是其中一种。

终端是文字交互界面，Shell是读取并解释命令的程序。Bash是常见Shell；同一台机器也可能安装其他Shell。

程序运行时有当前目录、环境变量、输入输出和权限。终端里输入命令，是让Shell按这些条件查找并启动程序。

```text
你的输入 → Shell解析 → 找到命令/程序 → 创建进程
                                      ↓
                              输出、退出码、文件变化
```

Windows PowerShell与Bash不是同一种语法。把Linux命令粘贴到PowerShell之前，应先确认自己在哪种终端。

WSL和虚拟机可用于软件练习，但USB、网络、图形与服务管理条件可能与独立Linux主机不同。能够运行纯计算程序不等于硬件透传已经配置好。

## 2. 先认识自己的环境

在Linux终端依次执行：

```bash
uname -a
cat /etc/os-release
id
printf '%s\n' "$SHELL"
python3 --version
git --version
```

`uname`显示内核相关信息，`os-release`描述发行版，`id`显示当前用户与组。`$SHELL`通常表示账号默认Shell，不保证就是当前正在运行的每一层命令解释器。

如果某条命令不存在，先区分“软件未安装”与“PATH中找不到”。不要因为一个工具缺失就先重装系统。

需要安装教学工具时，在了解会修改系统软件包的前提下，可以使用Ubuntu包管理器：

```bash
sudo apt update
sudo apt install python3 python3-venv python3-pip build-essential cmake git gdb
```

这些命令需要管理权限与可用的软件源。已经具备所需工具时不必重复安装，也不应为了照抄版本号盲目升级整套环境。

## 3. 当前目录：命令从哪里出发

`pwd`打印当前工作目录。相对路径以当前目录为起点，绝对路径从根目录`/`开始。

```bash
mkdir -p "$HOME/cuadc_foundation_lab"
cd "$HOME/cuadc_foundation_lab"
pwd
mkdir -p data scripts src config reports
ls
```

`$HOME`是当前账号家目录的环境变量；这里读取它，不改变它。`~`也常用于表示家目录。根目录`/`和管理员账号root是不同概念。

进入`data`后，`../reports`表示先回到上级，再进入reports。文件名的大小写通常有区别，`Data`与`data`不能随意替换。

练习中每次“找不到文件”，先执行`pwd`，再检查路径和大小写。不要先复制一份文件到所有可能目录。

## 4. 文件名、空格与引号

Shell通常把未加引号的空格当作参数分隔。路径中有空格时，应把整个路径放在引号中。

```bash
mkdir -p "data/sample set"
printf '%s\n' 'sample,ok' > "data/sample set/status.txt"
cat "data/sample set/status.txt"
```

单引号倾向于保留字面文本；双引号允许变量展开，但把展开结果保留为一个参数。下面两种写法含义不同：

```bash
printf '%s\n' '$HOME'
printf '%s\n' "$HOME"
```

反引号和`$(...)`可执行命令替换。来自网页或消息的未知字符串不应直接拼接进Shell命令；程序需要传文件参数时，优先使用明确参数与安全引用。

以`-`开头的文件名可能被工具当作选项。一些工具用`--`结束选项解析，例如`ls -- 文件名`；是否支持应查该命令帮助。

## 5. 建立一份可读的数据

在练习根目录执行：

```bash
cat > data/samples.csv <<'CSV'
time_s,x_m
0.0,0.00
0.1,0.02
0.2,0.05
0.3,0.09
CSV
```

`>`把输出写入文件，已有同名文件会被截断；本例只创建个人练习数据。`>>`表示追加，也不能不检查路径就使用。

`<<'CSV'`是带字面引用的here-document，Shell将后续行作为输入传给`cat`，直到独立一行的`CSV`。结束标记要保持一致。

CSV第一行是列名，`time_s`表示秒，`x_m`表示米。文件扩展名不会自动赋予单位或格式，列名与教程约定才说明如何解释。

文本通常采用UTF-8和LF换行。Windows文件带CRLF时，Shell脚本的解释器行可能多出不可见字符；编辑器应明确显示编码与换行设置。

## 6. 阅读、搜索和重定向

常见读取方式：

```bash
cat data/samples.csv
head -n 3 data/samples.csv
wc -l data/samples.csv
```

`cat`适合短文件；`head`只看开头；`wc -l`统计换行数，不能直接等同数据样本数。上面的CSV有表头，实际数据为4行。

安装了ripgrep时，可以用：

```bash
rg -n 'time_s|x_m' data/samples.csv
```

这里的`|`在正则表达式中表示“或”，因为在引号内，不作为Shell管道。没有`rg`时可以使用`grep -nE`完成这个简单搜索。

标准输出stdout通常用于程序结果，标准错误stderr通常用于诊断。`>`只重定向stdout；`2>`单独重定向stderr；`2>&1`将stderr指向当前stdout的去处，顺序会影响结果。

不把屏幕上“没文字”直接判断为成功，后面还要检查退出码和预期文件。

## 7. 用户与权限

```bash
ls -l data/samples.csv
id
```

`ls -l`显示所有者、组和权限等信息。文件的读、写、执行分别表示读取内容、修改内容和尝试作为程序执行；目录的执行权限主要关系到路径遍历，不等同于“执行目录”。

`sudo`改变执行权限，不修复程序逻辑。把所有文件改为任意人可写，或总用管理员启动程序，可能掩盖原本应该解决的归属与权限问题。

自己创建的脚本可以只增加自己的执行位：`chmod u+x scripts/某脚本.sh`。路径中的“某脚本”是示意名称，需替换为实际存在的文件。

设备访问通常由设备节点的所有者、组、ACL与规则决定。先查看权限，再查系统约定，不把chmod大范围放权作为默认排障方法。

### 阶段练习A

1. `cat data/samples.csv`失败时，最先检查什么？
2. `>`与`>>`分别改变什么？
3. 文件能读取，为什么`./script.py`仍可能不能执行？

答案：先检查当前目录与路径；前者截断重写、后者追加；直接执行还涉及执行权限、解释器行与解释器是否存在，而`python3 script.py`由解释器读取文件。

## 8. 命令从哪里来：PATH与which

输入`python3`时，Shell会按自己的规则解析别名、函数、内建命令与外部程序。对于外部程序，PATH是一组用于搜索的目录。

```bash
type python3
command -v python3
printf '%s\n' "$PATH"
python3 -c 'import sys; print(sys.executable)'
```

`command -v`说明当前Shell怎样找到命令；`sys.executable`说明Python实际运行的解释器。两者在复杂环境下都值得记录。

`./program`显式请求当前目录中的程序。当前目录不需要放入PATH；这也帮助区分“刚编译的程序”与系统里另一个同名程序。

修改PATH只改变查找条件，不会自动重新构建程序，也不能保证动态库来自同一套环境。

## 9. 环境变量、命令行参数与配置文件

环境变量由父进程传给子进程。普通Shell变量并不都自动进入子进程，`export`用于将变量加入随后启动进程的环境。

```bash
export CUADC_LAB_LABEL=desk_test
python3 -c 'import os; print(os.environ.get("CUADC_LAB_LABEL", "unset"))'
```

本例使用任务专属变量名，不覆盖HOME、PATH或系统目录变量。关闭终端后，新终端不一定继承这一设置。

可以只对单次启动传环境：

```bash
CUADC_LAB_LABEL=one_run python3 -c 'import os; print(os.environ["CUADC_LAB_LABEL"])'
```

参数、配置文件与环境变量的优先级由程序定义。不要仅凭文件写了某个值，就认为它在运行时一定生效。程序最好打印或保存最终有效配置。

## 10. Python虚拟环境

虚拟环境帮助一个项目维护独立的Python包集合，不是完整虚拟机。它仍依赖基础解释器、操作系统库与某些外部驱动。

在练习根目录执行：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -c 'import sys; print(sys.executable); print(sys.version)'
python -m pip --version
```

激活主要改变当前Shell的环境。也可以不激活，直接使用`.venv/bin/python`。安装包时使用`python -m pip`，有助于保证pip属于当前解释器。

本篇计算示例只用标准库，无需额外安装包。需要离开环境时执行`deactivate`。

不要复制另一台机器的`.venv`就当完成部署；应记录依赖并在目标机器重建。ROS等带系统Python扩展的环境，还要核对ABI与系统包兼容性。[Python 3.10 venv说明](https://docs.python.org/3.10/library/venv.html)

## 11. 从参数到结果：一个完整Python小程序

将下面代码保存为`scripts/summarize.py`。它读取第5章的CSV，检查时间顺序和有限数，再计算平均速度；不打开任何硬件。

```python
import argparse
import csv
import json
import math
import sys
from pathlib import Path


def summarize(path: Path) -> dict[str, float | int]:
    rows: list[tuple[float, float]] = []
    with path.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != ["time_s", "x_m"]:
            raise ValueError("expected columns: time_s,x_m")
        for line, row in enumerate(reader, start=2):
            if None in row:
                raise ValueError(f"line {line}: too many columns")
            try:
                t = float(row["time_s"])
                x = float(row["x_m"])
            except (TypeError, ValueError) as error:
                raise ValueError(f"line {line}: expected two numbers") from error
            if not math.isfinite(t) or not math.isfinite(x):
                raise ValueError(f"line {line}: non-finite number")
            if t < 0 or (rows and t <= rows[-1][0]):
                raise ValueError(f"line {line}: time must strictly increase")
            rows.append((t, x))
    if len(rows) < 2:
        raise ValueError("at least two samples are required")
    duration = rows[-1][0] - rows[0][0]
    displacement = rows[-1][1] - rows[0][1]
    return {"samples": len(rows), "duration_s": duration,
            "displacement_m": displacement,
            "average_velocity_m_s": displacement / duration}


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize a teaching CSV")
    parser.add_argument("csv", type=Path)
    args = parser.parse_args()
    try:
        result = summarize(args.csv)
    except (OSError, UnicodeError, ValueError) as error:
        print(f"input error: {error}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

从练习根目录运行：

```bash
python3 scripts/summarize.py data/samples.csv
```

应得到4个样本、0.3 s时长、0.09 m位移，平均速度约0.3 m/s。它是整个时间段的平均速度，不是每个瞬间的速度；浮点打印也可能出现末位差异。

路径在当前目录下解析，不是自动相对于脚本位置。换一个当前目录，命令中的相对输入路径也需要相应改变。

## 12. 退出码与日志

进程结束时返回整数状态。Shell里`$?`表示刚结束的前台命令状态，通常0表示成功，非0的含义由程序定义。

```bash
python3 scripts/summarize.py data/samples.csv > reports/summary.json 2> reports/summary.err
printf 'exit=%s\n' "$?"
cat reports/summary.json
```

再运行一个预期失败的输入：

```bash
python3 scripts/summarize.py data/not_here.csv
printf 'exit=%s\n' "$?"
```

本程序应在stderr解释输入错误并返回2。失败命令可能没有生成预期结果，不能只看到旧的`summary.json`还在就认为这次运行成功。

可以用`tee`同时显示与记录输出，但默认管道状态通常取最后一条命令。Bash的`set -o pipefail`会改变管道失败的传播方式；写自动脚本时要明确定义策略。

日志记录应包含输入文件、实际参数、时间和错误原文。不要只截图最后一行“程序退出”。

## 13. 进程、前台与停止

把下面程序保存为`scripts/heartbeat.py`，再用`python3 -u scripts/heartbeat.py`运行：

```python
import os
import time


def main() -> None:
    start = time.monotonic()
    print(f"pid={os.getpid()}", flush=True)
    try:
        for sequence in range(10):
            print(f"sequence={sequence}, elapsed_s={time.monotonic() - start:.3f}",
                  flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("interrupted by user", flush=True)


if __name__ == "__main__":
    main()
```

它正常约两秒结束；也可以按Ctrl+C请求中断。`-u`减少Python标准流缓冲，方便及时观察；打印间隔仍受调度影响，不能当作精确时钟。

另一终端可用`ps -u "$USER" -o pid,ppid,etime,args`观察自己的进程。PID是进程身份，程序名不一定唯一。

`Ctrl+Z`通常挂起前台任务，不能等同于干净退出。恢复、终止或清理进程前，确认它是自己本次启动的对象，不根据名称批量结束其他人的程序。

## 14. C++源码怎样变成程序

可以先把构建理解为预处理、编译、汇编和链接几个阶段。

```text
源文件.cpp + 头文件.hpp
        ↓ 编译
     目标文件.o
        ↓ 与其他目标文件/库链接
      可执行程序
```

头文件找不到、语法错误、链接符号缺失、运行时库找不到，发生在不同阶段。错误都叫“编不过”会丢失重要信息。

保存为`src/speed.cpp`：

```cpp
#include <iostream>

int main()
{
    const double displacement_m = 3.0;
    const double duration_s = 2.0;
    std::cout << "average_velocity_m_s="
              << displacement_m / duration_s << '\n';
    return 0;
}
```

从练习根目录编译并运行：

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -O0 src/speed.cpp -o reports/speed
./reports/speed
```

预期输出为`average_velocity_m_s=1.5`。`-o`指定输出文件，`-g`保留调试信息，`-O0`降低优化以便初学者观察；这些并不改变单位或证明物理模型正确。

## 15. 编译错误与链接错误

编译器通常给出文件、行列、诊断和相关上下文。先处理第一处主要错误，后面的报错可能只是连锁结果。

在个人练习副本中，故意删掉一个分号，观察编译错误；还原后再编译。不要将损坏副本覆盖唯一可用示例。

链接错误则可能发生在语法已经正确之后。下面分三个完整文件理解声明与定义。

`src/average.hpp`：

```cpp
#pragma once

double average_velocity(double displacement_m, double duration_s);
```

`src/average.cpp`：

```cpp
#include "average.hpp"
#include <cmath>
#include <stdexcept>

double average_velocity(double displacement_m, double duration_s)
{
    if (!std::isfinite(displacement_m) || !std::isfinite(duration_s) || duration_s <= 0.0) {
        throw std::invalid_argument("finite displacement and positive duration required");
    }
    return displacement_m / duration_s;
}
```

`src/average_main.cpp`：

```cpp
#include "average.hpp"
#include <iostream>

int main()
{
    std::cout << average_velocity(3.0, 2.0) << '\n';
    return 0;
}
```

正确编译时把两个实现源文件一起传给编译器：

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -O0 src/average_main.cpp src/average.cpp -o reports/average
./reports/average
```

如果只传`average_main.cpp`，编译阶段可能成功，但链接器找不到`average_velocity`的定义。找到声明，不等于已经链接实现。

## 16. CMake管理构建规则

CMake生成构建系统，不是C++编译器本身。它负责描述目标、源文件、选项和依赖，再调用对应工具链完成构建。

在练习根目录保存`CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.22)
project(foundation_lab LANGUAGES CXX)

add_executable(speed src/speed.cpp)
add_executable(average src/average_main.cpp src/average.cpp)

foreach(target IN ITEMS speed average)
    target_compile_features(${target} PRIVATE cxx_std_17)
    set_target_properties(${target} PROPERTIES CXX_EXTENSIONS OFF)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()
```

该例为Ubuntu下GCC/Clang风格选项；其他编译器应重新核对。然后执行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
./build/speed
./build/average
```

`-S`指定源码根目录，`-B`指定生成目录。构建产物放在build，有助于把源码与机器相关文件分开。[CMake 3.22官方教程](https://cmake.org/cmake/help/v3.22/guide/tutorial/index.html)

改代码后需要重新构建，旧可执行文件不会因为源文件变动自动获得新行为。构建失败后，也不要误运行残留的旧程序当作新结果。

## 17. 动态库、Python包与系统包

三个层次经常同时出现：系统软件包、Python包、C/C++库。相似名称不代表可以互相替代。

| 问题表现 | 首先确认 |
|---|---|
| `command not found` | 命令是否安装、PATH与当前Shell |
| `ModuleNotFoundError` | 实际Python解释器、环境与包名 |
| 编译时头文件缺失 | 开发头文件、包含路径与版本 |
| 链接时报未定义符号 | 实现/库是否参与链接、接口是否一致 |
| 启动时共享库缺失 | 运行时搜索路径与实际库文件 |

`pip`安装Python包不保证补齐系统驱动，复制一个`.so`也不保证ABI兼容。先确定问题在哪个阶段，再选择包管理或构建方法。

学习安装时保留命令与版本，避免为了“试试看”在系统环境、用户目录和多个虚拟环境里重复混装。

## 18. 用调试器回答具体问题

对第15章程序，使用带调试信息的构建，在练习根目录启动：

```bash
gdb --args ./build/average
```

进入GDB后依次输入：

```text
break average_velocity
run
print displacement_m
print duration_s
bt
next
continue
quit
```

断点让程序在指定位置暂停；`print`观察当前作用域中的值；`bt`查看调用栈。优化后的程序可能内联函数或消除变量，因此最初用Debug构建学习。

调试器看到一个值，只能说明暂停时的状态。它会改变时序，不适合直接证明一个高频并发系统从未出现竞态。

对纯计算Python程序，也可以使用`python3 -m pdb scripts/summarize.py data/samples.csv`学习断点、步进与堆栈。只设置能回答问题的观察点，不以反复单步代替假设。

## 19. 从“能运行”到“解释输出”

第11章算出的平均速度为0.3 m/s，只来自首尾位置与时间差。换一份中间曲线剧烈振荡、首尾相同的数据，它可能得到相同结果。

由此可以区分：程序运行成功、输入格式满足约定、计算公式实现正确、统计量适合问题，属于不同层次。

给程序写检查时，先选能够区分错误实现的输入。例如非等间隔时间、重复时间、非有限数与不足两个样本，能检验比“只运行一次正常例子”更多的条件。

不必为每行代码写一个镜像测试。重点检查边界、单位、状态变化和错误路径是否符合定义，并保存预期与实际差异。

### 阶段练习B

1. 源码修改后直接运行旧程序，为什么输出没变？
2. 包已安装但Python仍找不到，先检查什么？
3. 包含了头文件，为什么还可能链接失败？

答案：未成功重新构建或运行了另一个路径的程序；先检查解释器与环境；头文件声明并不替代目标文件或库中的定义。

## 20. 网络：地址、端口与传输

IP地址标识网络接口，端口帮助传输层区分端点。`127.0.0.1`是本机回环地址，不能让另一台机器通过它连接到当前主机。

Linux只读观察命令：

```bash
ip address
ip route
ss -lntup
```

`ss`显示套接字；部分进程信息可能受权限限制。看到某端口监听，不代表应用层消息一定正确。

TCP提供有序字节流，不保留每次发送操作的消息边界；UDP保留数据报边界，但应用要处理丢失、重复和乱序等情况。二者都不能只凭“网络连上”证明实时性。

若需要离线学习服务启动，先建立只放公开练习文件的目录，保持当前目录仍在练习根目录：

```bash
mkdir -p "$HOME/cuadc_foundation_lab/public_web"
printf '%s\n' 'hello from the local lab' > "$HOME/cuadc_foundation_lab/public_web/index.txt"
python3 -m http.server 8000 --bind 127.0.0.1 --directory "$HOME/cuadc_foundation_lab/public_web"
```

运行时用同机浏览器访问`http://127.0.0.1:8000/`，或在另一个终端用`curl http://127.0.0.1:8000/index.txt`读取内容。`curl`是可选工具，未安装时用浏览器即可。

服务运行在前台，返回该终端按Ctrl+C停止，再执行`cd "$HOME/cuadc_foundation_lab"`确认回到练习根目录。此服务会按指定目录提供文件，不能把目录换成包含凭据或私人文件的路径。

## 21. USB与串口设备

在有相应工具的Linux主机上，可以先观察：

```bash
lsusb
ls -l /dev/serial/by-id
id
```

没有`/dev/serial/by-id`目录可能表示当前没有对应串口设备或规则未建立，不应先创建一个同名普通文件“修复”。

`/dev/ttyUSB0`这类枚举名可能随插入顺序变化。可用时优先登记设备唯一身份与稳定别名，同时核对实际指向。

串口占用属于进程问题；权限拒绝属于访问条件；打开成功但消息错误属于链路或协议问题。三者不要混在一起。

观察内核事件可使用`journalctl -k -n 50 --no-pager`，权限和日志可用性依系统设置而异。本文不向任何真实串口写数据，也不通过修改权限掩盖接线或协议问题。

## 22. 时间：墙上时间与经过时间

系统日历时间适合标注日期，单调时间适合测量本机经过时长。系统校时可能使日历时间调整，不能无条件用相邻日历读数之差做稳定超时。

```bash
date --iso-8601=seconds
python3 -c 'import time; print("unix_s=", time.time()); print("monotonic_s=", time.monotonic())'
```

单调时钟起点没有通用日历含义，不能把不同机器的数值直接相减。相机、飞控与主机还可能处于不同测量时钟域。

日志中给时间字段命名时，区分采集、接收、计算和写入。`timestamp`一个名字不足以说明物理含义。

复现跨机器问题时，记录时区、时间同步机制、运行起止与时钟映射状态。提高消息频率不能自动解决时间域不一致。

## 23. CPU、内存、磁盘与服务

卡顿可能来自计算、内存压力、磁盘、USB、网络或等待锁，不能只看某一秒CPU占用。

```bash
free -h
df -h .
du -sh reports
ps -u "$USER" -o pid,etime,pcpu,pmem,args
```

可用空间不足可能使日志中断；内存不足可能使进程被终止。`free`中的可用内存与简单的“完全空闲”不是同一指标。

在使用systemd的主机上，`systemctl --failed`观察失败单元，`journalctl -b -n 50 --no-pager`读取本次启动日志。容器或未启用systemd的环境不一定适用。

开机自启不只是把命令放进一个文件：还涉及工作目录、用户、环境、设备就绪、失败恢复与退出行为。先让手动运行可解释，再学习创建服务。

## 24. Git记录的是哪些变化

Git帮助保存文件状态与变化历史，不等于云盘，也不会默认把本地内容上传。

```text
工作区文件 ── git add ── 暂存区 ── git commit ── 本地历史
```

工作区是当前文件；暂存区说明下一次提交准备包含什么；提交把该状态放进本地历史。远端操作是另一组动作，本篇练习不配置远端，也不推送。

在自己的练习根目录中，确认这里不是其他项目后执行：

```bash
git init -b main
git status --short
```

为这个练习仓库设置身份。下方为教学占位，正式记录应换成自己的身份，不覆盖全局配置：

```bash
git config --local user.name 'Student Example'
git config --local user.email 'student@example.invalid'
```

再创建`.gitignore`：

```gitignore
.venv/
build/
reports/
__pycache__/
*.pyc
local.env
```

忽略规则不是秘密扫描器，已经跟踪的文件也不会因为新增规则自动退出历史。日志、凭据、账号、私人坐标与设备信息，应先判断能否公开再决定保存位置。

## 25. 一次可审阅的本地提交

把教学源码和固定例题数据纳入记录，运行报告和构建目录保持在忽略范围：

```bash
git add .gitignore CMakeLists.txt src scripts data/samples.csv
git diff --cached --stat
git diff --cached
git commit -m 'Add standalone computation exercises'
git status --short
```

提交前看暂存区实际内容，不只看编辑器里打开的文件。`git diff`默认比较工作区与暂存区；`git diff --cached`比较暂存区与当前提交。[Git diff官方说明](https://git-scm.com/docs/git-diff)

第一次提交后，可以创建实验分支：

```bash
git switch -c experiment/data-checks
git log --oneline -n 3
```

分支只是本地历史引用，不会自动生成另一个物理副本。切换分支前先看工作区变化；尚未提交的文件不一定符合你对“分支隔离”的直觉。

修改例题后先用`git diff`审阅。需要取消暂存某文件，可以在已有提交的前提下用`git restore --staged 文件路径`，它保留工作区修改；该路径是占位，需替换。丢弃工作区修改是另一种可能损失内容的动作，不作为新人默认操作。

## 26. 写一个能从任意目录启动的脚本

相对输入路径常是“换台机器就失败”的原因之一。下面脚本把工作目录明确定位到自己所在目录的上一层。

保存为`scripts/run_lab.sh`：

```bash
#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
lab_root="$(cd -- "$script_dir/.." && pwd)"
cd -- "$lab_root"
mkdir -p reports

python3 scripts/summarize.py data/samples.csv > reports/summary.json
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
./build/speed > reports/speed.txt
./build/average > reports/average.txt
printf 'Results are in %s/reports\n' "$lab_root"
```

运行`bash scripts/run_lab.sh`，不要求脚本有执行位；想用`./scripts/run_lab.sh`时，再通过`chmod u+x scripts/run_lab.sh`增加自己的执行位。

上面这个相对入口仍要求Shell能在当前目录找到`scripts/run_lab.sh`。若当前在其他目录，使用`bash "$HOME/cuadc_foundation_lab/scripts/run_lab.sh"`；脚本启动后才会按自身位置定位数据与构建目录。这是两个不同阶段的路径解析。

`set -u`使未定义变量的某些使用成为错误，`pipefail`改变管道状态传播；`set -e`在许多失败场景中退出，但存在条件与上下文例外，不能当作完整错误处理语言。

脚本选用PATH中的`python3`和编译工具。因此还应记录实际解释器与版本；定位了工作目录，不代表已经解决全部环境可复现问题。

启动脚本失败时保留标准错误和退出码。不要在脚本末尾无条件打印“成功”，让上游错误被掩盖。

## 27. 综合实验：制造一个可解释的失败

先确认正常CSV能运行，再在练习目录创建另一份故障数据，保留原件：

```bash
cat > data/repeated_time.csv <<'CSV'
time_s,x_m
0.0,0.00
0.1,0.02
0.1,0.05
CSV
python3 scripts/summarize.py data/repeated_time.csv
printf 'exit=%s\n' "$?"
```

第11章程序应指出时间不严格递增，返回2。输入错误发生在计算平均速度之前，这比产生一个看起来正常的数值更有用。

再分别建立缺列、包含`nan`、只有一个样本的文件。每次只改变一种条件，运行前先写预期结果。

| 输入变化 | 应拒绝的原因 | 需要保留的证据 |
|---|---|---|
| 时间重复或倒退 | 无法按约定形成严格时间序列 | 原始行、报错行号与退出码 |
| 位置是`nan` | 非有限数不具备正常物理意义 | 原字符串与错误信息 |
| 缺少列 | 不满足输入契约 | 表头与解析规则 |
| 仅一个样本 | 无法计算首尾时间差 | 样本数与提示 |

检查完错误路径，再构造“格式完全合法，但物理上可疑”的数据，例如极短时间内位置跳变100 m。当前程序可能接受它，因为本文尚未定义速度上限与跳变判据。

这不是让读者悄悄加一个万能阈值，而是学习区别输入格式验证与领域合理性判断。新增条件前应说明单位、适用场景与误拒绝风险。

### 一份最小复现应包含什么

写清系统与工具版本、当前目录、解释器、精确命令、最小输入、预期、实际输出、退出码与最近唯一改动。检查是否含私人或账号信息，再交给别人复现。

不要只说“别人的电脑能跑”“我这里报错”。同样也不应把几百兆日志一股脑发出，却不标明问题发生在哪个时间窗口。

## 28. 自测、验收与后续学习

### 自测题

**1. 当前目录与脚本所在目录相同吗？**

不一定。当前目录由启动者决定，脚本可以位于其他路径。

**2. 激活虚拟环境后，另一个终端自动使用同一环境吗？**

通常不会。激活修改当前Shell环境，应在实际启动终端确认解释器。

**3. `git add`是否已经保存了一次历史提交？**

没有，它更新暂存区。提交还需检查内容并执行本地commit。

**4. 命令经过`tee`后状态为0，前面的程序一定成功吗？**

不一定。须明确管道状态规则和是否启用pipefail，或分别保存进程结果。

**5. 设备文件存在，能否证明收到有效传感器数据？**

不能。还缺访问、占用、电气、协议与业务语义的验证。

**6. 一个库文件名相同，能直接换掉系统中的同名库吗？**

不能。接口、ABI、架构、依赖与加载路径都可能不同，先在隔离环境核对。

**7. CPU占用不高，为什么仍可能很卡？**

可能在等待I/O、锁、设备或网络，也可能受调度、内存和队列影响。

**8. Linux程序退出码为0，能否说明实验结论正确？**

不能。还要核对输入、单位、公式、适用范围与验证证据。

### 阶段验收

完成以下项目后，再进入[ROS生活篇](Re0_从零开始的ROS生活.md)：

- 能从空目录建立数据、脚本和构建目录，并解释每个文件的用途。
- 能运行Python例题，解释正常结果及至少三种输入错误。
- 能独立编译C++，辨认编译、链接和运行失败。
- 能用CMake重新构建，在GDB里观察一次函数调用。
- 能保存一次本地Git提交，审阅暂存内容，并解释哪些文件未被记录。
- 能写出一份不依赖口头补充的最小复现记录。

这些是学习目标，不是对某台设备或无人机系统的验收。后续硬件与通信实验还应依据实际设备说明开展。

### 实验记录模板

| 项目 | 填写内容 |
|---|---|
| 问题 | 想验证哪个具体行为 |
| 环境 | 发行版、内核、解释器、编译器与相关依赖 |
| 目录 | 完整工作目录与输入路径 |
| 代码身份 | Git提交及未提交差异 |
| 启动 | 命令、参数、相关环境变量，不含凭据 |
| 输入 | 最小数据、单位、编码、文件散列 |
| 结果 | 输出、错误、退出码与生成文件 |
| 解释 | 事实、假设、尚未排除的原因 |

### 官方资料入口

- [Ubuntu：命令行入门](https://ubuntu.com/tutorials/command-line-for-beginners)：终端、路径与基本操作；页面中的历史系统截图不作为本篇版本约定。
- [GNU Bash手册](https://www.gnu.org/software/bash/manual/bash.html)：引用、重定向、管道、变量与退出行为。
- [Python 3.10：venv](https://docs.python.org/3.10/library/venv.html)：环境创建与激活的语义。
- [Python 3.10：argparse](https://docs.python.org/3.10/library/argparse.html)：命令行参数与帮助信息。
- [CMake 3.22教程](https://cmake.org/cmake/help/v3.22/guide/tutorial/index.html)：构建目标与依赖组织。
- [GNU GDB手册](https://sourceware.org/gdb/current/onlinedocs/gdb/): 断点、变量、栈和调试条件。
- [Pro Git：初次配置](https://git-scm.com/book/en/v2/Getting-Started-First-Time-Git-Setup)：身份、配置层级与本地记录。
- [Git diff说明](https://git-scm.com/docs/git-diff)：工作区、暂存区与提交之间的比较。

工具文档用于确认接口；例题结果由文中完整输入定义。跟读时把自己的真实命令与输出写进记录，而不是把教程中的预期输出当作已经运行的证据。


