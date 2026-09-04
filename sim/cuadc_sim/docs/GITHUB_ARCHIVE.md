# GitHub 归档说明

## 推荐仓库名

```text
cuadc_rescue_sim
```

## 推荐提交内容

从包根目录提交：

```text
~/cuadc_ws/src/cuadc_rescue_sim
```

应包含：

```text
.gitignore
CMakeLists.txt
package.xml
README.md
config/
docs/
launch/
models/
scripts/
src/
worlds/
```

不要提交：

```text
~/cuadc_ws/build
~/cuadc_ws/install
~/cuadc_ws/log
~/.ros/log
/tmp/cuadc_rescue_sim.log
```

## 初始化 Git 仓库

```bash
cd ~/cuadc_ws/src/cuadc_rescue_sim
git init
git add .
git commit -m "Initial CUADC rescue Gazebo simulation environment"
```

## 推送到 GitHub

在 GitHub 创建空仓库后：

```bash
git remote add origin git@github.com:<your-user>/cuadc_rescue_sim.git
git branch -M main
git push -u origin main
```

如果使用 HTTPS：

```bash
git remote add origin https://github.com/<your-user>/cuadc_rescue_sim.git
git branch -M main
git push -u origin main
```

## 生成压缩包

使用脚本：

```bash
cd ~/cuadc_ws/src/cuadc_rescue_sim
scripts/archive_for_github.sh
```

输出位置：

```text
~/cuadc_rescue_sim_github_archive.tar.gz
```

## 许可证提醒

本包中：

- `models/iris_d435i`
- `models/iris_d435i_airframe`

基于本机 `~/ardupilot_gazebo` 的 Iris 模型派生。上游 `ardupilot_gazebo/LICENSE.md` 为 LGPL v3，归档到 GitHub 时应保留来源说明。

危险化学品标识图片来自比赛附件 11，建议仅用于赛事训练与仿真用途。

如果后续要公开发布给他人复用，建议单独补一个完整 `LICENSE` 文件，并确认所有派生模型与图片素材的再分发权限。

## 恢复测试清单

克隆仓库后，按以下顺序验证：

```bash
mkdir -p ~/cuadc_ws/src
cd ~/cuadc_ws/src
git clone <repo-url> cuadc_rescue_sim

cd ~/cuadc_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select cuadc_rescue_sim
source install/setup.bash

cd ~/cuadc_ws/src/cuadc_rescue_sim
python3 scripts/generate_scene.py

cd ~/cuadc_ws
colcon build --packages-select cuadc_rescue_sim
ros2 launch cuadc_rescue_sim cuadc_sim.launch.py
```

Gazebo 中应看到：

- 起降区和 H 点
- 起飞线
- 5m x 8m 投放区
- 5m x 8m 侦察区
- 3 个随机投放桶
- 5 个随机侦察桶
- 3 个危险化学品标识贴图
- `iris_d435i` 飞行器
