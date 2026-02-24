# COLMAP ONNX 模型预下载

本项目已改为**从本地目录加载** ONNX 模型，不再运行时下载。需先执行脚本下载模型到 `onnx_models/` 目录。

## 使用方法

**Windows (PowerShell) - 推荐：下载到项目目录**
```powershell
cd third_party\colmap
.\scripts\download_onnx_models.ps1 -ToProject
```

**下载到用户缓存目录（可选）**
```powershell
.\scripts\download_onnx_models.ps1
```

**Linux/macOS:**
```bash
cd third_party/colmap
chmod +x scripts/download_onnx_models.sh
./scripts/download_onnx_models.sh
```

## 下载位置

- **项目目录（-ToProject）：** `third_party/colmap/onnx_models/`，构建时会复制到 exe 同目录
- **用户缓存：** `~/.cache/colmap/` or `%USERPROFILE%\.cache\colmap\`

## 下载的模型列表

| 文件 | 用途 |
|------|------|
| aliked-n16rot.onnx | ALIKED 特征提取 (N16 旋转) |
| aliked-n32.onnx | ALIKED 特征提取 (N32) |
| aliked-lightglue.onnx | ALIKED + LightGlue 特征匹配 |
| bruteforce-matcher.onnx | 暴力 ONNX 匹配器 |
| sift-lightglue.onnx | SIFT + LightGlue 特征匹配 |

执行脚本后，colmap 在运行时将直接使用本地缓存，**不再发起网络下载**。
