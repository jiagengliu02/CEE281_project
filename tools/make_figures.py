"""Static report figures from existing freeway results (no re-simulation).

Run from the project root:
    /usr/bin/python3 -s tools/make_figures.py
Outputs PNG files to results/figures/.
"""
import csv
import json
import math
import site
import sys
from collections import defaultdict
from pathlib import Path

sys.path.append(site.getusersitepackages())
import matplotlib  # noqa: E402

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'results' / 'figures'
OUT.mkdir(parents=True, exist_ok=True)

MODES = ['open', 'closure', 'fixed', 'adaptive', 'adaptive_fault']
MODE_LABEL = {'open': '无施工区', 'closure': '封闭·不限流', 'fixed': '封闭·固定12 s',
              'adaptive': '封闭·自适应', 'adaptive_fault': '封闭·自适应+故障'}
COLORS = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4']
TEXT, MUTED, GRID = '#0b0b0b', '#52514e', '#e4e3df'

plt.rcParams.update({
    'font.family': ['PingFang SC', 'Heiti SC', 'Arial Unicode MS', 'sans-serif'],
    'axes.unicode_minus': False, 'axes.edgecolor': MUTED, 'axes.labelcolor': TEXT,
    'xtick.color': MUTED, 'ytick.color': MUTED, 'axes.spines.top': False,
    'axes.spines.right': False, 'axes.grid': True, 'grid.color': GRID,
    'grid.linewidth': 0.6, 'axes.axisbelow': True, 'figure.dpi': 150,
    'legend.frameon': False, 'font.size': 9,
})


def load_metrics(folder):
    return {m: json.load(open(folder / m / 'metrics.json')) for m in MODES}


def fig_trajectories():
    cfg = json.load(open(ROOT / 'results/freeway/config.json'))
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), sharey=True)
    for ax, mode in zip(axes, ['open', 'closure']):
        by_vehicle = defaultdict(list)
        for r in csv.DictReader(open(ROOT / f'results/freeway/{mode}/traces.csv')):
            if r['lane_id'] == '1' and r['origin'] == 'main':
                by_vehicle[r['vehicle_id']].append((float(r['time_s']), float(r['position_m'])))
        for pts in by_vehicle.values():
            pts.sort()
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=COLORS[0], lw=0.5, alpha=0.6)
        if mode != 'open':
            ax.axhspan(cfg['taper_start_m'], cfg['closure_end_m'], color=COLORS[1], alpha=0.12, lw=0)
            ax.text(5, cfg['closure_end_m'] + 10, '左车道封闭 500–850 m', color=MUTED, fontsize=8)
        ax.axvspan(240, 480, color=GRID, alpha=0.5, lw=0)
        ax.set_title(f'{MODE_LABEL[mode]}（中间车道）', loc='left', color=TEXT)
        ax.set_xlabel('时间 (s)')
        ax.set_xlim(0, 600)
    axes[0].set_ylabel('位置 (m)')
    fig.suptitle('图1 时空轨迹：阴影 240–480 s 为高需求期', x=0.01, ha='left', color=TEXT)
    fig.tight_layout()
    fig.savefig(OUT / 'fig1_trajectories.png')
    plt.close(fig)


def fig_delay_comparison():
    runs = [('默认需求', 'freeway'), ('重需求', 'freeway_heavy'),
            ('同质驾驶员', 'freeway_homogeneous'), ('无预测消融', 'freeway_no_forecast')]
    data = {label: load_metrics(ROOT / 'results' / folder) for label, folder in runs}
    fig, ax = plt.subplots(figsize=(9, 4))
    width = 0.19
    for i, (label, _) in enumerate(runs):
        vals = [data[label][m]['total_delay_vehicle_s'] / 60 for m in MODES]
        xs = [j + (i - 1.5) * width for j in range(len(MODES))]
        bars = ax.bar(xs, vals, width - 0.02, color=COLORS[i], label=label)
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v + 4, f'{v:.0f}', ha='center', fontsize=6.5, color=MUTED)
    ax.set_xticks(range(len(MODES)))
    ax.set_xticklabels([MODE_LABEL[m] for m in MODES])
    ax.set_ylabel('总延误 (veh·min)')
    ax.set_title('图2 各控制模式总延误（含残余排队与边界等待）', loc='left', color=TEXT)
    ax.set_ylim(0, 800)
    ax.legend(ncol=4, loc='upper center', bbox_to_anchor=(0.5, -0.12))
    fig.tight_layout()
    fig.savefig(OUT / 'fig2_delay_by_mode.png')
    plt.close(fig)


def fig_estimate_vs_truth():
    d = json.load(open(ROOT / 'results/freeway/data.json'))
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.8), sharex=True)
    for ax, mode in zip(axes, ['adaptive', 'adaptive_fault']):
        frames = d['runs'][mode]['frames']
        t = [f['time'] for f in frames]
        truth = [len(f['vehicles']) for f in frames]
        est = [f['estimate']['road_vehicles'] if f['estimate'] and f['estimate']['road_vehicles'] is not None
               else math.nan for f in frames]
        ax.plot(t, truth, color=COLORS[0], lw=2, label='真值：路上车辆数')
        ax.plot(t, est, color=COLORS[1], lw=2, label='估计（边界守恒）')
        if mode == 'adaptive_fault':
            for f in d['config']['tmc']['faults']:
                ax.axvspan(f['start_s'], f['end_s'], color=COLORS[3], alpha=0.15, lw=0)
            ax.text(302, 3, '报告缺失', fontsize=7, color=MUTED)
            ax.text(422, 3, '延迟45 s', fontsize=7, color=MUTED)
        ax.set_title(MODE_LABEL[mode], loc='left', color=TEXT)
        ax.set_xlabel('时间 (s)')
        ax.set_xlim(0, 600)
    axes[0].set_ylabel('车辆数 (veh)')
    axes[0].legend(loc='upper left')
    fig.suptitle('图3 路上车辆数：估计 vs 被隐藏的仿真真值', x=0.01, ha='left', color=TEXT)
    fig.tight_layout()
    fig.savefig(OUT / 'fig3_estimate_vs_truth.png')
    plt.close(fig)


def fig_forecast_mae():
    m = json.load(open(ROOT / 'results/freeway/adaptive/metrics.json'))['forecast']
    channels = ['main_arrivals', 'ramp_arrivals', 'exits']
    names = {'main_arrivals': '主路到达', 'ramp_arrivals': '匝道到达', 'exits': '离开'}
    fig, ax = plt.subplots(figsize=(6, 3.5))
    width = 0.36
    for i, (method, label) in enumerate([('moving_average', '移动平均(基线)'), ('ema', 'EMA α=0.35')]):
        vals = [m[method][c]['mae_veh'] for c in channels]
        xs = [j + (i - 0.5) * width for j in range(len(channels))]
        bars = ax.bar(xs, vals, width - 0.03, color=COLORS[i], label=label)
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v + 0.05, f'{v:.2f}', ha='center', fontsize=7, color=MUTED)
    ax.set_xticks(range(len(channels)))
    ax.set_xticklabels([names[c] for c in channels])
    ax.set_ylabel('MAE (veh / 30 s)')
    ax.set_title('图4 30 s 预测误差（评估期 ≥120 s，相同时域）', loc='left', color=TEXT)
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT / 'fig4_forecast_mae.png')
    plt.close(fig)


def fig_discharge():
    # Analytic IDM two-lane capacity with mean driver parameters.
    v0, T, a, b, s0, L = 28, 1.5, 1.2, 2.0, 2.0, 5
    cap = max(v / ((s0 + v * T) / math.sqrt(1 - (v / v0) ** 4) + L) * 3600 for v in [i / 100 for i in range(1, 2790)])
    fig, ax = plt.subplots(figsize=(8, 3.8))
    for i, mode in enumerate(['open', 'closure', 'adaptive']):
        flow = defaultdict(int)
        for r in csv.DictReader(open(ROOT / f'results/freeway/{mode}/observations.csv')):
            if r['position_m'] == '900':
                flow[float(r['interval_end_s'])] += int(r['count'])
        ts = sorted(flow)
        ax.plot(ts, [flow[t] * 120 for t in ts], color=COLORS[i], lw=2, marker='o', ms=4, label=MODE_LABEL[mode])
    ax.axhline(2 * cap, color=MUTED, ls='--', lw=1)
    ax.text(5, 2 * cap + 40, f'IDM 平衡态两车道通行能力 ≈ {2 * cap:.0f} veh/h（均值参数）', fontsize=7.5, color=MUTED)
    ax.axhline(3000 + 450, color=GRID, lw=1)
    ax.text(5, 3450 - 160, '240–480 s 总需求 3450 veh/h', fontsize=7.5, color=MUTED)
    ax.set_xlabel('30 s 区间结束时间 (s)')
    ax.set_ylabel('流量 (veh/h，三车道合计)')
    ax.set_title('图5 900 m 检测器流量：瓶颈下游放行能力', loc='left', color=TEXT)
    ax.set_xlim(0, 600)
    ax.set_ylim(0, 4000)
    ax.legend(loc='lower right')
    fig.tight_layout()
    fig.savefig(OUT / 'fig5_discharge_900m.png')
    plt.close(fig)
    return cap


if __name__ == '__main__':
    fig_trajectories()
    fig_delay_comparison()
    fig_estimate_vs_truth()
    fig_forecast_mae()
    cap = fig_discharge()
    print(f'IDM per-lane capacity {cap:.0f} veh/h; figures in {OUT}')
