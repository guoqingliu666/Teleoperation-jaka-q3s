$ErrorActionPreference = "Stop"

# 校验发布清单中的关键文件。清单与脚本分离，避免每次发布都改校验逻辑。
$manifest = "v1.5.0_SHA256SUMS.txt"
if (-not (Test-Path -LiteralPath $manifest -PathType Leaf)) {
    Write-Host "缺少：$manifest" -ForegroundColor Red
    exit 2
}

function Get-Sha256([string] $Path) {
    $stream = [System.IO.File]::OpenRead((Resolve-Path -LiteralPath $Path))
    try {
        $sha = [System.Security.Cryptography.SHA256]::Create()
        try {
            return ([System.BitConverter]::ToString($sha.ComputeHash($stream))).Replace("-", "")
        }
        finally { $sha.Dispose() }
    }
    finally { $stream.Dispose() }
}

$failures = @()
$checked = 0
foreach ($line in Get-Content -LiteralPath $manifest -Encoding UTF8) {
    if ([string]::IsNullOrWhiteSpace($line) -or $line.StartsWith("#")) { continue }
    if ($line -notmatch '^([0-9A-Fa-f]{64})  (.+)$') {
        $failures += "清单格式错误：$line"
        continue
    }
    $expected = $matches[1].ToUpperInvariant()
    $relativePath = $matches[2].Replace('/', [IO.Path]::DirectorySeparatorChar)
    if (-not (Test-Path -LiteralPath $relativePath -PathType Leaf)) {
        $failures += "缺少：$relativePath"
        continue
    }
    $checked += 1
    if ((Get-Sha256 $relativePath) -ne $expected) {
        $failures += "哈希不一致：$relativePath"
    }
}

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Host $_ -ForegroundColor Red }
    exit 2
}
if ($checked -eq 0) {
    Write-Host "清单中没有可校验文件。" -ForegroundColor Red
    exit 2
}

Write-Host "v1.5.0关键文件完整性检查通过（$checked 项）。" -ForegroundColor Green
