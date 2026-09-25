$ErrorActionPreference = "Stop"

# 只校验决定当前数字孪生和遥操作行为的关键文件。
# Git本身还会校验全部版本对象；本脚本便于下载ZIP后的普通用户检查。
$expected = [ordered]@{
    "Player_通信修复\QuestPosePreview.exe" = "76A4560E40EC43334F5300F52D45E74F55579709D5EA01EB838F24C7AC9E568B"
    "Player_通信修复\UnityPlayer.dll" = "1ADA7EE20459BBCA28DC5824090D56B6C4D14E356E0CE130DF33E3B834CD6E21"
    "Player_通信修复\QuestPosePreview_Data\Managed\Assembly-CSharp.dll" = "7A955E41EC40B058334B4C7F5BA323B2B9536C9E019ACE5F434AEA4338B4EF57"
    "Python\quest_endpoint_teleop_gui.py" = "CFC9100FF3E5FD82CCBA0A693AF88549FAF8E75D70BD2DA063B37137D5A64326"
    "Python\连续采样位置遥操作.py" = "8DDE775DFB8A0B5136EF3096A2F330F62E2C5888FA02A31664E93B90BFC3B9B4"
}

function Get-Sha256([string] $Path) {
    $stream = [System.IO.File]::OpenRead((Resolve-Path -LiteralPath $Path))
    try {
        $sha = [System.Security.Cryptography.SHA256]::Create()
        try {
            return ([System.BitConverter]::ToString($sha.ComputeHash($stream))).Replace("-", "")
        }
        finally {
            $sha.Dispose()
        }
    }
    finally {
        $stream.Dispose()
    }
}

$failures = @()
foreach ($relativePath in $expected.Keys) {
    if (-not (Test-Path -LiteralPath $relativePath -PathType Leaf)) {
        $failures += "缺少：$relativePath"
        continue
    }
    $actual = Get-Sha256 $relativePath
    if ($actual -ne $expected[$relativePath]) {
        $failures += "哈希不一致：$relativePath"
    }
}

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Host $_ -ForegroundColor Red }
    exit 2
}

Write-Host "v1.0.0关键文件完整性检查通过。" -ForegroundColor Green
