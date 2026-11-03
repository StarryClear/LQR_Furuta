# sim/ · 定点闭环模型（辅助验证，**当前不可用**）

> ⚠ **状态（2026-11-03）：脚本会崩，不能当回归工具用。**
> `python sim/furuta_closed_loop.py` 在第 728 行抛
> `TypeError: not all arguments converted during string formatting`
> （`log` 元组 12 个字段，格式串只有 9 个占位符）。
> 另有：`log` 只在 `verbose=True` 时填充，而统计量全从 `log` 算 ——
> 所以 `--swp` 模式下 `p95|θ|` 永远是 `nan`。
>
> 详见 [../docs/04_已知问题.md](../docs/04_已知问题.md) C-2。
> 修法约半小时：补 3 个 `%` + 让 `log` 在 `verbose=False` 时也填。
> **在本文件更新前，报告里不要引用本目录的任何结论。**

---

## 是什么

`furuta_closed_loop.py` 用实测辨识参数
（m=0.14 kg、l=0.0756 m、r=0.128 m、M11=1.070e-3、Ja=1.8e-3、bp=8.89e-5、
堵转 0.216 N·m、空载 33.5 rad/s、臂侧输入增益 B=0.019 rad/s²/count）
搭非线性 Furuta 模型，然后把 `lqr_core.v` 与 `swing_ctrl.v` 的**整数运算逐步复刻**成 Python
（移位/截断/限幅/状态机都照 RTL 写），跑闭环。

只用 Python 标准库（math / random / sys），不需要 numpy：

```bash
python sim/furuta_closed_loop.py          # 默认场景（当前会崩）
python sim/furuta_closed_loop.py --swp    # 扫 invert_pd × E_STOP（能跑，但质量列为 nan）
```

## 它能回答什么

1. **泵能方向**：见 `polarity_check.txt`。从下垂静止出发，固定幅值 900 counts、
   按 `±sign(θ̇·cosθ)` 施力 6 s：`+` 方向把能量从 −1.999 抬到 −1.024（泵能），
   `−` 方向不动 → 对应 RTL 里 `invert_pd=1`。这是 `SW_INVERT` 默认值的依据。
2. **状态机/交接结构**：`S_KICK1/S_KICK2/S_PUMP/S_WAIT/S_BAL/S_FALL` 的跳转、
   捕获判据（`ANG_CAP`/`W_CAP`/2 帧）、`force_fall` 回退，都按 RTL 复刻。

## 它不能回答什么（别拿它当验收）

- **不能证明"固件在实物上能站住"**。固件那组 KTH4600/KDT60/KDP130/KPH−30 是针对实物
  摩擦/回差手调的；模型里横臂几乎只有反电动势阻尼，这组增益在模型里是**临界**的（会慢速发散）。
  要让模型站住得换成设计增益 `K=[-3.21,-0.31,-0.32,-0.18]`（脚本里 `ctrl='design'`）。
- 没有建模：齿轮回差、SA100 滑环掉线、法兰倾斜（0.8°/圈）、结构模态、驱动器电流限幅、
  以及实测到的 5.1 Hz 执行器抖振。
- 起摆耗时/成功率对摩擦、`I_FLOOR`、`E_STOP` 很敏感，模型只能给量级。

## 与实物的关系

实物的验收依据是 `docs/data/` 里的实测 CSV + `tools/recompute_all.py`，
**不是**本目录的模型。模型的定位只有两个：验方向、验状态机流程。
