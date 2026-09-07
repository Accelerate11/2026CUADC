# 坐标系

## 正式视觉 / 状态机：FLU

```text
+X forward
+Y left
+Z up
```

当前 D435i：

```text
image left  = aircraft forward
image up    = aircraft right
optical +Z  = aircraft down
```

当前 optical -> FLU：

```text
[-0.971358, 0.237622, 0
  0.237622, 0.971358, 0
  0,        0,       -1]
```

## 独立 viewer：FRD

```text
+X forward
+Y right
+Z down
```

转换：

```text
FRD = diag(1,-1,-1) * FLU
```

当前 optical -> FRD：

```text
[-0.971358,  0.237622, 0
 -0.237622, -0.971358, 0
  0,         0,        1]
```

投放口：

```text
P1 FLU [ 0.0260,-0.0650,-0.3200]
P1 FRD [ 0.0260, 0.0650, 0.3200]

P2 FLU [ 0.0109, 0.0720,-0.3200]
P2 FRD [ 0.0109,-0.0720, 0.3200]
```
