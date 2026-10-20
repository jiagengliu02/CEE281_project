# CEE281

双击 `view.html` 选择仿真，或直接打开 `freeway.html` / `intersection.html`。页面自带数据，无需服务器。

- `freeway/`：IDM 高速公路与匝道；包含检测、估计、预测、反馈决策和执行。详见 `freeway/README.md`。
- `intersection/`：路口信号控制；详见 `intersection/README.md`。
- `docs/`：课程项目要求。
- `results/`：可复现实验数据、指标、日志、报告和 HTML。
- `run.py`：统一运行入口。

在当前目录运行：

```bash
python3 run.py freeway
python3 run.py intersection
python3 run.py check all
```

默认运行会更新当前目录对应的播放页面。指定 `--output` 的自定义实验只更新指定目录。`.html.in` 是生成模板，不能直接播放。

匝道控制支持无控制、固定控制、自适应控制和检测器故障对照。预测实际参与自适应决策；等待区采用点队列，汇入后车辆使用 IDM。当前控制规则为教学原型，实际对比结果见 `results/freeway/experiment_report.md`，不保证优于固定控制。
