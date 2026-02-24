# COLMAP ONNX 模型预下载脚本
# 将模型下载到本地，运行时直接使用，无需再下载
# 用法: .\download_onnx_models.ps1           # 下载到 ~/.cache/colmap
#       .\download_onnx_models.ps1 -ToProject # 下载到项目 onnx_models/ 目录

param([switch]$ToProject)

$BaseUrl = "https://github.com/colmap/colmap/releases/download/3.13.0"
$Models = @(
    @{
        File = "aliked-n16rot.onnx"
        Sha256 = "39c423d0a6f03d39ec89d3d1d61853765c2fb6a8b8381376c703e5758778a547"
    },
    @{
        File = "aliked-n32.onnx"
        Sha256 = "a077728a02d2de1a775c66df6de8cfeb7c6b51ca57572c64c680131c988c8b3c"
    },
    @{
        File = "aliked-lightglue.onnx"
        Sha256 = "b9a5de7204648b18a8cf5dcac819f9d30de1a5961ef03756803c8b86c2dceb8d"
    },
    @{
        File = "bruteforce-matcher.onnx"
        Sha256 = "3c1282f96d83f5ffc861a873298d08bbe5219f59af59223f5ceab5c41a182a47"
    },
    @{
        File = "sift-lightglue.onnx"
        Sha256 = "e0500228472b43f92b3d36881a09b3310d3b058b56187b246cc7b9ab6429096e"
    }
)

# 输出目录
if ($ToProject) {
    $ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
    $CacheDir = Join-Path (Join-Path $ScriptDir "..") "onnx_models"
    if (-not (Test-Path $CacheDir)) {
        New-Item -ItemType Directory -Path $CacheDir -Force | Out-Null
    }
    $CacheDir = (Resolve-Path $CacheDir).Path
    Write-Host "Downloading to project: $CacheDir"
} else {
    $HomeDir = if ($env:USERPROFILE) { $env:USERPROFILE } elseif ($env:HOME) { $env:HOME } else { "." }
    $CacheDir = "$HomeDir\.cache\colmap"
    if (-not (Test-Path $CacheDir)) {
        New-Item -ItemType Directory -Path $CacheDir -Force | Out-Null
    }
    Write-Host "Downloading to cache: $CacheDir"
}

# 项目目录使用简单文件名，缓存目录使用 sha256-文件名 格式
$UseSimpleNames = $ToProject

$ErrorCount = 0
foreach ($m in $Models) {
    $CachedPath = if ($UseSimpleNames) { "$CacheDir\$($m.File)" } else { "$CacheDir\$($m.Sha256)-$($m.File)" }
    $Url = "$BaseUrl/$($m.File)"

    if (Test-Path $CachedPath) {
        Write-Host "[SKIP] $($m.File) - already cached"
        continue
    }

    Write-Host "[DOWNLOAD] $($m.File)..."
    try {
        $tempFile = [System.IO.Path]::GetTempFileName()
        Invoke-WebRequest -Uri $Url -OutFile $tempFile -UseBasicParsing
        Move-Item -Path $tempFile -Destination $CachedPath -Force
        $Hash = (Get-FileHash -Path $CachedPath -Algorithm SHA256).Hash.ToLower()
        if ($Hash -ne $m.Sha256) {
            Write-Host "[ERROR] SHA256 mismatch for $($m.File)" -ForegroundColor Red
            Remove-Item $CachedPath -Force
            $ErrorCount++
        } else {
            Write-Host "[OK] $($m.File) saved to $CachedPath"
        }
    } catch {
        Write-Host "[ERROR] Failed to download $($m.File): $_" -ForegroundColor Red
        $ErrorCount++
    }
}

if ($ErrorCount -eq 0) {
    Write-Host "`nAll models downloaded successfully. Colmap will use local cache at runtime."
} else {
    Write-Host "`n$ErrorCount download(s) failed." -ForegroundColor Red
    exit 1
}
