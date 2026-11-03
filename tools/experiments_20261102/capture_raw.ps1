param(
    [string]$Port = "COM6",
    [int]$Seconds = 240,
    [string]$Out = "D:\DeepSeek_Harness\对话文件\work\raw.csv"
)

$sp = New-Object System.IO.Ports.SerialPort $Port, 115200, 'None', 8, 'One'
$sp.ReadTimeout = 200
$sp.DtrEnable = $true
$sp.RtsEnable = $true
try { $sp.Open() } catch { "OPEN_FAIL: " + $_.Exception.Message | Out-File -FilePath ($Out + '.err') -Encoding utf8; exit 1 }

[System.IO.File]::WriteAllText($Out, "t,theta,duty,state`r`n")
$buf = New-Object byte[] 4096
$bytes = New-Object System.Collections.Generic.List[byte]
$scan = 0
$n = 0
$sb = New-Object System.Text.StringBuilder
$sw = [System.Diagnostics.Stopwatch]::StartNew()

while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    try {
        $r = $sp.Read($buf, 0, $buf.Length)
        if ($r -gt 0) { for ($q = 0; $q -lt $r; $q++) { $bytes.Add($buf[$q]) } }
    } catch { }

    while ($scan + 3 -lt $bytes.Count) {
        if ($bytes[$scan] -eq 0x7F -and $bytes[$scan-1] -eq 0x80 -and $bytes[$scan-2] -eq 0x00 -and $bytes[$scan-3] -eq 0x00) {
            if ($scan -ge 27) {
                $arr = $bytes.ToArray()
                $ch0 = [int][BitConverter]::ToSingle($arr, $scan - 27)
                $duty = [int][BitConverter]::ToSingle($arr, $scan - 27 + 4)
                $th = [int][BitConverter]::ToSingle($arr, $scan - 27 + 12)
                $st = ($ch0 -shr 4) -band 7
                [void]$sb.AppendLine(("{0},{1},{2},{3}" -f $n, $th, $duty, $st))
                $n++
                if ($n % 50 -eq 0) {
                    [System.IO.File]::AppendAllText($Out, $sb.ToString()); [void]$sb.Clear()
                }
            }
            $scan++
        } else { $scan++ }
    }
}
[System.IO.File]::AppendAllText($Out, $sb.ToString())
$sp.Close()
"WROTE $Out frames=$n dur=$([math]::Round($sw.Elapsed.TotalSeconds,1))"
