# 下载

源码与硬件工程直接从仓库获取，二进制包从 [GitHub Releases](https://github.com/kiki348/codex-micro-diy/releases) 下载。

| 文件 | 用途 |
|---|---|
| CodexMicro-Hardware.zip | 工程、BOM、机构和面板文件；当前Gerber状态见包内说明 |
| CodexMicro-FirstFlash-v18.zip | 空白ESP32-S3 N16R8首次烧录，静态核验版 |
| CodexMicro-OTA-v18.zip | 已有HID OTA设备升级 |
| CodexMicro-Recovery-v18.zip | 恢复启动脚本与默认配置；需要配套Windows完整包 |
| WorkAxis-Windows-x64.zip | 闭源Windows软件完整运行目录 |
| SHA256SUMS.txt | 下载完整性校验 |

不要混用初次烧录包和OTA应用镜像。完整验证范围见[验证记录](validation.md)。

当前为 `v0.18.0-diy.1` 预发布。压缩包 SHA256 另见 [SHA256SUMS.txt](SHA256SUMS.txt)。
