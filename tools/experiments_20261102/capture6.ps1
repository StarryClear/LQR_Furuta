param(
    [string]$Port = "COM6",
    [int]$Seconds = 10,
    [string]$Out = "v30.csv",
    [switch]$NoDtr
)

$ErrorActionPreference = "Continue"
$sp = New-Object System.IO.Ports.SerialPort $Port, 115200, 'None', 8, 'One'
$sp.ReadTimeout = 100
if (-not $NoDtr) { $sp.DtrEnable = $true; $sp.RtsEnable = $true }
try { $sp.Open() } catch {
    Set-Content -Path ($Out + ".err") -Value ("OPEN_FAIL: " + $_.Exception.Message)
    Write-Output ("OPEN_FAIL: " + $_.Exception.Message)
    exit 1
}

$sb = New-Object System.Text.StringBuilder
[void]$sb.AppendLine("t,ch0,duty,pos,theta,adc,ch5")
$buf = New-Object byte[] 4096
$bytes = New-Object System.Collections.Generic.List[byte]
$fc = New-Object byte[] 28
$scan = 3
$n = 0
$bad = 0
$sw = [System.Diagnostics.Stopwatch]::StartNew()

while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    try {
        $r = $sp.Read($buf, 0, $buf.Length)
        for ($q = 0; $q -lt $r; $q++) { $bytes.Add($buf[$q]) }
    } catch { }

    while ($scan + 3 -lt $bytes.Count) {
        if ($bytes[$scan] -eq 0x7F -and $bytes[$scan-1] -eq 0x80 -and $bytes[$scan-2] -eq 0x00 -and $bytes[$scan-3] -eq 0x00) {
            if ($scan -ge 27) {
                $st = $scan - 27
                for ($k = 0; $k -lt 28; $k++) { $fc[$k] = $bytes[$st + $k] }
                $f0 = [BitConverter]::ToSingle($fc, 0)
                $f1 = [BitConverter]::ToSingle($fc, 4)
                $f2 = [BitConverter]::ToSingle($fc, 8)
                $f3 = [BitConverter]::ToSingle($fc, 12)
                $f4 = [BitConverter]::ToSingle($fc, 16)
                $f5 = [BitConverter]::ToSingle($fc, 20)
                # 合法性校验：任一通道越界就认为这是数据里的假帧尾，跳 1 字节重新同步
                $ok = ($f0 -ge 0 -and $f0 -le 4095) -and
                      ([math]::Abs($f1) -le 2600) -and
                      ([math]::Abs($f2) -le 2000000) -and
                      ([math]::Abs($f3) -le 4000) -and
                      ($f4 -ge -10 -and $f4 -le 6000) -and
                      ([math]::Abs($f5) -le 40000)
                if ($ok) {
                    [void]$sb.AppendLine(("{0},{1},{2},{3},{4},{5},{6}" -f $n, [int]$f0, [int]$f1, [int]$f2, [math]::Round($f3, 2), [int]$f4, [int]$f5))
                    $n++
                    if ($n % 100 -eq 0) {
                        [System.IO.File]::AppendAllText($Out, $sb.ToString())
                        [void]$sb.Clear()
                    }
                } else { $bad++ }
            }
            $scan++
        } else { $scan++ }
    }

    if ($scan -gt 60000) {
        $bytes.RemoveRange(0, $scan - 27)
        $scan = 27
    }
}

[System.IO.File]::AppendAllText($Out, $sb.ToString())
$sp.Close()
"WROTE $Out frames=$n bad=$bad dur=$([math]::Round($sw.Elapsed.TotalSeconds,1))"
