# 网页可视化（SSH / 无显示器）

推荐链路：

```text
D435i -> drop_bucket_realsense_node.py -> /cuadc/calib/annotated
                                     -> web_video_server
                                     -> SSH tunnel
                                     -> Windows 浏览器
```

NUC：

```bash
./bin/web_video_server.sh
```

Windows CMD：

```cmd
ssh -L 8080:127.0.0.1:8080 cuadc@NUC_IP
```

不要在 CMD 中使用 Linux 的反斜杠 `\` 做换行。

浏览器：

```text
http://127.0.0.1:8080/stream?topic=/cuadc/calib/annotated&type=mjpeg
```

独立 viewer 的网页地址：

```text
http://127.0.0.1:8080/stream?topic=/vision_servo/alignment/image&type=mjpeg
```

若提示 `Address already in use`：

```bash
pgrep -af web_video_server
ss -lntp | grep ':8080'
```

通常是已有 web server，直接复用。
