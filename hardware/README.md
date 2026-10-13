# 硬件文件

`codex-micro-20260810.epro2` 是冻结的嘉立创EDA工程，包含原理图和 PCB。控制板 UUID `e43935b56b465ac6`；键轴板 UUID `b40bed2e986c04fb`。

`bom/` 从实际 PCB 放置元件提取；`AUDIT.json` 记录缺失字段和重复位号。它不是完成商家料号验证的自动下单清单。缺失字段不要靠名称猜测替代料。

`mechanical/outer-tolerance.step` 为推荐有公差外围，`outer-shell.step` 为无公差历史对照。`optional/` 是自制键帽与旋钮的可选试制版本，不等同于已经验证的整套成品装配模型。

`panel/` 的印刷图案与 100mm PDF 不等于开孔加工图，孔位从工程里的面板导出。Gerber/CPL 应与本工程版本一起导出并做 DRC 和工厂预览，详见[打样](../docs/03-manufacturing.md)。
