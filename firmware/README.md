# firmware/ · 码流

| 文件 | 版本 | CheckSum | 说明 |
|---|---|---|---|
| `LQR_end1.1_0x2D55.fs` | v1.1 | **0x2D55** | 当前定版：`KDT=120`，θ̇ 低通 2 ms，压掉 5 Hz 极限环 |
| `history/LQR_end_1.0_0x6A6F.fs` | v1.0 | 0x6A6F | 上一版，指标更全（83% 起摆、514.8 s 平衡），回滚用 |

两个码流都由本仓库 `src/` 用 Gowin EDA V1.9.12.04 重建得到，校验和与板上运行版本一致
（复验记录见 `evidence/build_1.1.log`）。改动源码后重建，`impl/pnr/LQR_end.fs` 头部的
`CheckSum` 就是版本指纹。

## 烧写

```powershell
$env:GOWIN_HOME = 'C:\Gowin\Gowin_V1.9.12.04_x64'   # 改成你的安装路径

& "$env:GOWIN_HOME\Programmer\bin\programmer_cli.exe" `
    --device GW2A-18C --cable "Gowin USB Cable(FT2CH)" --run 2 `
    --fsFile ".\firmware\LQR_end1.1_0x2D55.fs"
# 成功标志：Status Code is: 0x00006020 + Finished.
```

或直接用 Gowin Programmer GUI：打开 `.fs` → 选 GW2A-18C → Program/Configure。

## 注意

- **`UserCode` 不是版本号**：它等于本次构建的 CheckSum（`0x00002D55`），换源码就会变。
  认版本请看上表或用 `impl/pnr/LQR_end.fs` 注释里的 CheckSum。
- **SA100 滑环死点**：重烧后若遥测 `ch0` 的 bit7=1、θ 冻结，手转横臂半圈即可（老问题，未修）。
- 烧写前确认板上电、JTAG/USB Cable 被 Gowin Programmer 识别。
