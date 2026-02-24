#!/bin/bash
# COLMAP ONNX 模型预下载脚本
# 将模型下载到本地，运行时直接使用，无需再下载
# 用法: ./download_onnx_models.sh           # 下载到 ~/.cache/colmap
#       ./download_onnx_models.sh -ToProject # 下载到项目 onnx_models/

BASE_URL="https://github.com/colmap/colmap/releases/download/3.13.0"

if [ "$1" = "-ToProject" ]; then
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
    CACHE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)/onnx_models"
    USE_SIMPLE_NAMES=1
else
    CACHE_DIR="${HOME}/.cache/colmap"
    USE_SIMPLE_NAMES=0
fi

mkdir -p "$CACHE_DIR"

download() {
    local file=$1
    local sha256=$2
    local cached
    if [ "$USE_SIMPLE_NAMES" = "1" ]; then
        cached="${CACHE_DIR}/${file}"
    else
        cached="${CACHE_DIR}/${sha256}-${file}"
    fi
    local url="${BASE_URL}/${file}"

    if [ -f "$cached" ]; then
        echo "[SKIP] $file - already cached"
        return 0
    fi

    echo "[DOWNLOAD] $file..."
    if curl -fL -o "$cached" "$url"; then
        local hash=$(sha256sum "$cached" | cut -d' ' -f1)
        if [ "$hash" = "$sha256" ]; then
            echo "[OK] $file saved to $cached"
        else
            echo "[ERROR] SHA256 mismatch for $file"
            rm -f "$cached"
            return 1
        fi
    else
        echo "[ERROR] Failed to download $file"
        return 1
    fi
}

ERRORS=0
download "aliked-n16rot.onnx" "39c423d0a6f03d39ec89d3d1d61853765c2fb6a8b8381376c703e5758778a547" || ((ERRORS++))
download "aliked-n32.onnx" "a077728a02d2de1a775c66df6de8cfeb7c6b51ca57572c64c680131c988c8b3c" || ((ERRORS++))
download "aliked-lightglue.onnx" "b9a5de7204648b18a8cf5dcac819f9d30de1a5961ef03756803c8b86c2dceb8d" || ((ERRORS++))
download "bruteforce-matcher.onnx" "3c1282f96d83f5ffc861a873298d08bbe5219f59af59223f5ceab5c41a182a47" || ((ERRORS++))
download "sift-lightglue.onnx" "e0500228472b43f92b3d36881a09b3310d3b058b56187b246cc7b9ab6429096e" || ((ERRORS++))

if [ $ERRORS -eq 0 ]; then
    echo ""
    echo "All models downloaded. Colmap will use local cache at runtime."
else
    echo ""
    echo "$ERRORS download(s) failed."
    exit 1
fi
