# 首次烧录、OTA 和恢复

## 选择正确的包

| 设备状态 | 使用包 | 会做什么 |
|---|---|---|
| 空白 ESP32-S3 N16R8 | `codex-micro-first-flash-v18.zip` | 写启动程序、分区表、OTA初始化和应用 |
| 已运行本项目 HID OTA 固件 | `codex-micro-ota-v18.zip` | 只更新应用，保留现有配置 |
| HID可识别但配置/应用异常 | `codex-micro-recovery-v18.zip` | 走受支持的 HID 恢复，按说明备份配置 |
| 完全无法枚举 HID | ROM恢复排查 | 先确认供电、下载模式、驱动和Flash；不拿OTA镜像写0地址 |

`.18` 应用镜像 SHA256：`eb784b18473d7fb2ba3556f2abc520e807c77de6259d1df212e9d88dcef77a1a`。

## 空白板首次烧录

安装 Python 3.11+ 和包内固定依赖：`python -m pip install -r requirements.txt`。先让板进入 ESP32-S3 ROM 下载模式（GPIO0低电平后复位），用数据USB线连接，按设备管理器确认串口。

先运行 `python first_flash.py --port COM你的串口`，默认只校验本地文件，不接触设备。确认目标是空白板后，运行 `python first_flash.py --port COM你的串口 --blank-board --execute`。

脚本先核对 SHA256、芯片和16MB Flash；读取 NVS、OTA、两应用头和 SPIFFS 头检查空白。遇到已有数据拒绝继续，不自动全片擦除、不读写 eFuse。写入后执行对应片段校验。它不适合已有设备的保留配置升级。

| 地址 | 文件 |
|---|---|
| `0x0000` | bootloader.bin |
| `0x8000` | partitions.bin |
| `0xE000` | boot_app0.bin |
| `0x10000` | firmware.bin |

分区 app0为0x10000、app1为0x650000，每槽0x640000；SPIFFS为0xC90000。清单来源是当前 `.18` 板级构建，未混用历史修复镜像。

本包完成静态镜像/校验/分区检查，**空白板实际首次启动尚未实测**。启动后读版本、看USB枚举、保存默认配置并断电重启，才能完成首次烧录验收。

## OTA 升级

关闭 WorkAxis 与其他持有键盘 HID 的程序，USB只连接一个目标设备，运行升级包里的 `烧录.ps1`。它按设备序列号选择目标并先校验镜像。保持供电，不同时打开多个独立 RPC 工具。

## 恢复

先备份键位、灯光和智能动作。HID可识别时使用恢复包工具与发行默认配置；恢复默认会覆盖自定义配置。HID不可识别时先查 ROM 端口/驱动，不盲目刷官方异型板固件。

历史成功救援采用DIO启动程序与正确OTA元数据CRC；旧事故报告曾有错误结论，不能把个人全片备份当通用恢复包。这里不分发任何个人NVS/配对/序列号备份。
