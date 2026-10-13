# 上游固件来源清单

仓库：<https://github.com/wanglinkai326/led-control-system-firmware>

固定提交：`2ed0bd46d144e727118efefd00278c5bb7e8bbf4`

| 上游文件 | SHA-256 |
|---|---|
| `platformio.ini` | `B3F9D8B77ADF2AFC98F78B00AD625B852DD818C261AF59708BA5E59469CA0B23` |
| `include/CodexMicroBle.h` | `BDDBF83113D0EBF5CEA46B8AA31511CF33CA86398EC4E0981AFF114674342FA1` |
| `include/HidOtaUpdate.h` | `FF84114F8DF2CAA3413DA9B13EEAC3C6FF0B6789EA7667E92515077E9012C3EF` |
| `src/CodexMicroBle.cpp` | `36DFF594E670AB0111CCD0F60EBBCEA75C33E395A062C74330A47FE1FADF752F` |
| `src/HidOtaUpdate.cpp` | `1054632A930126437166D3C9F6FA9765BF1B30B9F06773A83B64C4B9440E4234` |
| `src/main.cpp` | `22439BB18EBFF2FE42813019450D7D91CE6085D4529C7B11F169A775E2E77DDC` |

`.gitignore` 是本项目为排除 `.pio/` 和 `.vscode/` 添加的本地治理文件，不属于上游提交。

## 本轮本地适配

以下文件是在固定上游基础上的本地 Work axis 适配，不应与上表的上游哈希混淆：

- `platformio.ini`：切换到 Codex Micro N16R8/16MB 自定义板和本地分区表；
- `boards/codex_micro_n16r8.json`、`partitions.csv`：板级 Flash/分区定义；
- `src/CodexMicroBle.cpp`：SPIFFS 文件层、官方 `fs.*` RPC、SHA-1 checksum、能力状态，
  以及与 `wl-device-kit` 3072-byte/4096-Base64 分片契约对齐的 RPC 文档容量；
- `README.md`：构建、烧录门禁和真实 HIL 入口。
