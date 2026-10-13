# ESP32-S3 固件

适配版本：`0.5.0+work-axis.18`。目标：ESP32-S3-WROOM-1U-N16R8，16MB Flash / 8MB PSRAM。

安装 PlatformIO 后，在本目录运行 `pio run -e esp32-s3-n16r8`。平台固定为 `espressif32@6.13.0`。串口由操作者明确指定，工程不固定某台电脑的 COM16。

现有发行镜像 SHA256：`eb784b18473d7fb2ba3556f2abc520e807c77de6259d1df212e9d88dcef77a1a`。它是应用分区镜像。首次烧录需要完整包；不能把它写入地址 0。

上游：[wanglinkai326/led-control-system-firmware](https://github.com/wanglinkai326/led-control-system-firmware)，基线提交 `2ed0bd46d144e727118efefd00278c5bb7e8bbf4`。原有版权头保留，本地改动与第三方许可分别记录。见[第三方说明](../THIRD_PARTY_NOTICES.md)、[烧录指南](../docs/05-flashing.md)。
