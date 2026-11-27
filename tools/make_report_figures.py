"""Figures and summary numbers for the CEE C181/C281 mid-report.

Reads the outputs written by `python3 run.py freeway` and
`python3 run.py intersection` and never re-runs a simulation.

Run from the repository root:

    python3 run.py freeway
    python3 run.py intersection
    python3 tools/make_report_figures.py

Writes fig01-fig08 (PNG) and report_numbers.json to results/figures/report/.
"""
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import matplotlib
import matplotlib.ticker

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, Rectangle  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
FW = ROOT / 'results' / 'freeway'
IX = ROOT / 'results' / 'intersection'
OUT = ROOT / 'results' / 'figures' / 'report'
OUT.mkdir(parents=True, exist_ok=True)

BLUE, ORANGE, GREEN, AMBER, PURPLE = '#2a78d6', '#e0612b', '#1a9e6f', '#d99a00', '#7b52c4'
INK, MUTED, GRID, SHADE = '#1a1a1a', '#5a5955', '#e4e3df', '#efeee9'
MODE_LABEL = {'open': 'No work zone', 'closure': 'Closure, no metering',
              'fixed': 'Closure + fixed 12 s meter', 'adaptive': 'Closure + adaptive meter',
              'adaptive_fault': 'Adaptive + detector faults'}
MODE_COLOR = {'open': MUTED, 'closure': ORANGE, 'fixed': AMBER, 'adaptive': BLUE,
              'adaptive_fault': PURPLE}

plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'Liberation Serif', 'DejaVu Serif'],
    'mathtext.fontset': 'stix', 'font.size': 10, 'axes.titlesize': 10.5,
    'axes.labelsize': 10, 'legend.fontsize': 8.5, 'xtick.labelsize': 9,
    'ytick.labelsize': 9, 'axes.edgecolor': MUTED, 'axes.labelcolor': INK,
    'xtick.color': MUTED, 'ytick.color': MUTED, 'axes.spines.top': False,
    'axes.spines.right': False, 'axes.grid': True, 'grid.color': GRID,
    'grid.linewidth': 0.6, 'axes.axisbelow': True, 'legend.frameon': False,
    'figure.dpi': 220, 'savefig.bbox': 'tight', 'savefig.pad_inches': 0.04,
})

NUM = {}  # numbers quoted in the report


def save(fig, name):
    fig.savefig(OUT / name)
    plt.close(fig)
    print('wrote', OUT / name)


def read_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


CFG = json.load(open(FW / 'config.json'))
FWDATA = json.load(open(FW / 'data.json'))
IXDATA = json.load(open(IX / 'data.json'))


# --------------------------------------------------------------------- Fig. 1
def fig_layout():
    fig, (a, b) = plt.subplots(1, 2, figsize=(7.0, 2.35), gridspec_kw={'width_ratios': [2.15, 1]})
    for ax in (a, b):
        ax.grid(False)
        for s in ax.spines.values():
            s.set_visible(False)
        ax.set_xticks([])
        ax.set_yticks([])
    # freeway, flow left to right, lane 0 (left lane) on top
    lane_h = 1.0
    a.set_xlim(-40, 1040)
    a.set_ylim(-2.3, 4.6)
    a.add_patch(Rectangle((0, 0), 1000, 3 * lane_h, fc='#f4f3ef', ec=MUTED, lw=0.8))
    for k in (1, 2):
        a.plot([0, 1000], [k, k], color='#b9b7b0', lw=0.7, ls=(0, (6, 5)))
    taper, c0, c1 = CFG['taper_start_m'], CFG['closure_start_m'], CFG['closure_end_m']
    a.fill([taper, c0, c1, c1], [3, 2, 2, 3], color=ORANGE, alpha=0.30, lw=0)
    a.text((c0 + c1) / 2, 2.5, 'left lane closed', ha='center', va='center', fontsize=8.5, color='#8a3512')
    # ramp joining the right lane at 250 m
    mp = CFG['merge_position_m']
    a.fill([mp - 205, mp - 165, mp + 15, mp - 25], [-1.9, -1.9, 0, 0], color='#f4f3ef', ec=MUTED, lw=0.8)
    a.plot([mp - 192, mp - 170], [-1.45, -1.45], color=GREEN, lw=3)
    a.text(mp + 40, -1.3, 'on-ramp (200 m) with meter (green)', fontsize=8, color=MUTED, va='center')
    for x in CFG['detectors_m']:
        a.plot([x, x], [-0.15, 3.15], color=BLUE, lw=1.4)
        a.text(x, 3.35, f'loop {x} m', ha='center', fontsize=8, color=BLUE)
    a.add_patch(FancyArrowPatch((20, 1.5), (120, 1.5), arrowstyle='-|>', mutation_scale=9, color=INK, lw=0.8))
    a.text(0, -0.55, '0 m', fontsize=8, color=MUTED)
    a.text(1000, -0.55, '1,000 m', fontsize=8, color=MUTED, ha='right')
    a.text(-35, 4.6, '(a) Freeway work zone, 3 lanes, flow left to right', fontsize=9.5, color=INK, va='top')
    # intersection
    b.set_xlim(-330, 330)
    b.set_ylim(-330, 400)
    w = 20
    b.add_patch(Rectangle((-w, -320), 2 * w, 640, fc='#f4f3ef', ec=MUTED, lw=0.8))
    b.add_patch(Rectangle((-320, -w), 640, 2 * w, fc='#f4f3ef', ec=MUTED, lw=0.8))
    b.add_patch(Rectangle((-w, -w), 2 * w, 2 * w, fc='#e3e1da', ec=MUTED, lw=0.8))
    for sgn in (1, -1):
        for d in (w + 2, w + 100):
            b.plot([-w, w], [sgn * d, sgn * d], color=BLUE, lw=1.4)
            b.plot([sgn * d, sgn * d], [-w, w], color=BLUE, lw=1.4)
    b.text(w + 6, w + 100, 'advance loop\n100 m', fontsize=7.5, color=BLUE, va='center')
    b.text(w + 6, -w - 10, 'stop-line loop', fontsize=7.5, color=BLUE, va='top')
    b.text(-320, 400, '(b) Intersection, 4 approaches x 300 m', fontsize=9.5, color=INK, va='top')
    save(fig, 'fig01_layout.png')


# --------------------------------------------------------------------- Fig. 2
def speed_grid(mode, dx=20.0, dtb=10.0):
    nx, nt = int(1000 / dx), int(600 / dtb)
    s = [[0.0] * nx for _ in range(nt)]
    n = [[0] * nx for _ in range(nt)]
    for r in read_csv(FW / mode / 'traces.csv'):
        x = float(r['position_m'])
        if r['origin'] == 'ramp' and x < CFG['merge_position_m']:
            continue
        i, j = int(float(r['time_s']) // dtb), int(x // dx)
        if 0 <= i < nt and 0 <= j < nx:
            s[i][j] += float(r['speed_mps'])
            n[i][j] += 1
    return [[s[i][j] / n[i][j] if n[i][j] else float('nan') for j in range(nx)] for i in range(nt)]


def queue_tail(grid, dx=20.0, dtb=10.0, vq=15.0):
    """Most upstream position of the slow region connected to the taper, per time bin."""
    tail = []
    jt = int(CFG['taper_start_m'] // dx)
    for i, row in enumerate(grid):
        j = jt
        if not (row[j] < vq or row[j - 1] < vq):
            continue
        while j > 0 and (row[j - 1] < vq or (j > 1 and row[j - 2] < vq)):
            j -= 1
        tail.append(((i + 0.5) * dtb, j * dx))
    return tail


def fig_contour():
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.75), sharey=True)
    tails = {}
    for ax, mode in zip(axes, ['open', 'closure']):
        g = speed_grid(mode)
        im = ax.imshow([list(col) for col in zip(*g)], origin='lower', aspect='auto',
                       extent=[0, 600, 0, 1000], cmap='RdYlGn', vmin=0, vmax=30, interpolation='nearest')
        for x in CFG['detectors_m']:
            ax.axhline(x, color='white', lw=0.7, ls=(0, (4, 3)))
        ax.axvline(240, color=INK, lw=0.6)
        ax.axvline(480, color=INK, lw=0.6)
        ax.grid(False)
        ax.set_xlabel('Time (s)')
        ax.set_title(MODE_LABEL[mode], loc='left')
        if mode == 'closure':
            for y in (CFG['taper_start_m'], CFG['closure_end_m']):
                ax.axhline(y, color=INK, lw=0.8, ls=(0, (1, 1.5)))
            ax.text(8, 675, 'closed section', fontsize=8, color=INK, va='center')
            t = queue_tail(g)
            tails[mode] = t
            grow = [(tt, x) for tt, x in t if 260 <= tt <= 480]
            if len(grow) > 3:
                mt = sum(p[0] for p in grow) / len(grow)
                mx = sum(p[1] for p in grow) / len(grow)
                slope = sum((p[0] - mt) * (p[1] - mx) for p in grow) / sum((p[0] - mt) ** 2 for p in grow)
                NUM['queue_tail_speed_mps'] = round(slope, 2)
                NUM['queue_tail_min_m'] = min(p[1] for p in t)
                xs = [grow[0][0], 480]
                ax.plot(xs, [mx + slope * (x - mt) for x in xs], color=INK, lw=1.4)
                ax.text(15, 420, f'queue tail moves\nupstream at {abs(slope):.1f} m/s', fontsize=8,
                        color=INK, ha='left', va='top')
    axes[0].set_ylabel('Position (m)')
    cb = fig.colorbar(im, ax=axes, fraction=0.025, pad=0.015)
    cb.set_label('Mean speed (m/s)')
    cb.outline.set_visible(False)
    save(fig, 'fig02_speed_contour.png')


# --------------------------------------------------------------------- Fig. 3
def idm_curve(v0=28.0, T=1.5, s0=2.0, L=5.0):
    pts = []
    for i in range(1, 2800):
        v = i / 100.0
        if v >= v0:
            break
        se = (s0 + v * T) / math.sqrt(1 - (v / v0) ** 4)
        k = 1000.0 / (se + L)
        pts.append((k, k * v * 3.6))
    pts.append((1000.0 / (s0 + L), 0.0))
    pts.sort()
    return pts


def lane_points(mode):
    out = defaultdict(list)
    for r in read_csv(FW / mode / 'observations.csv'):
        c, v = int(r['count']), r['mean_speed_mps']
        if c == 0 or not v:
            continue
        q = c * 3600 / (float(r['interval_end_s']) - float(r['interval_start_s']))
        out[int(r['position_m'])].append((q / (float(v) * 3.6), q, float(v)))
    return out


def fig_fd():
    fig, ax = plt.subplots(figsize=(5.2, 2.75))
    curve = idm_curve()
    ax.plot([p[0] for p in curve], [p[1] for p in curve], color=INK, lw=1.4,
            label='IDM equilibrium, mean driver (one lane)')
    kc, qc = max(curve, key=lambda p: p[1])
    NUM['idm_capacity_vphpl'] = round(qc)
    NUM['idm_critical_density_vpkm'] = round(kc, 1)
    ax.plot([kc], [qc], 'o', color=INK, ms=4)
    ax.annotate(f'IDM capacity {qc:,.0f} veh/h/lane', (kc, qc), (kc + 8, qc + 120), fontsize=8.5, color=INK)
    style = {200: (GREEN, 'o'), 550: (ORANGE, 's'), 900: (BLUE, '^')}
    for mode, filled in (('open', False), ('closure', True)):
        pts = lane_points(mode)
        for pos, (col, mk) in style.items():
            P = pts[pos]
            ax.scatter([p[0] for p in P], [p[1] for p in P], s=13, marker=mk,
                       facecolors=col if filled else 'none', edgecolors=col, lw=0.7, alpha=0.85,
                       label=f'{pos} m, {"closure" if filled else "no work zone"}')
    cl = lane_points('closure')[550]
    cong = [p for p in cl if p[2] < 15]
    if cong:
        NUM['loop550_congested_density_vpkm'] = round(sum(p[0] for p in cong) / len(cong), 1)
        NUM['loop550_congested_flow_vphpl'] = round(sum(p[1] for p in cong) / len(cong))
    ax.set_xlabel('Density (veh/km/lane)')
    ax.set_ylabel('Flow (veh/h/lane)')
    ax.set_xlim(0, 150)
    ax.set_ylim(0, 2300)
    ax.legend(loc='upper left', bbox_to_anchor=(1.01, 1.0), ncol=1, fontsize=7.8, handletextpad=0.3)
    save(fig, 'fig03_fundamental_diagram.png')


# --------------------------------------------------------------------- Fig. 4
def cum_demand(t):
    total = 0.0
    segs = CFG['demand']
    for k, seg in enumerate(segs):
        a = seg['start_s']
        b = segs[k + 1]['start_s'] if k + 1 < len(segs) else 1e9
        if t > a:
            total += (min(t, b) - a) * (seg['main_vph'] + seg['ramp_vph']) / 3600
    return total


def fig_cumulative():
    fig, (a, b) = plt.subplots(1, 2, figsize=(7.0, 2.6), gridspec_kw={'width_ratios': [1.25, 1]})
    t = list(range(0, 601))
    a.plot(t, [cum_demand(x) for x in t], color=INK, lw=1.5, label='Arrivals A(t), all runs')
    mid = []
    for mode in ['open', 'closure', 'fixed', 'adaptive']:
        fr = FWDATA['runs'][mode]['frames']
        a.plot([f['time'] for f in fr], [f['exits'] for f in fr], color=MODE_COLOR[mode], lw=1.3,
               ls='-' if mode in ('open', 'closure') else (0, (4, 2)), label='Exits D(t), ' + {'open': 'no work zone', 'closure': 'closure', 'fixed': 'fixed meter', 'adaptive': 'adaptive meter'}[mode])
        rq = [f['ramp_queue'] for f in fr]
        sm = [sum(rq[max(0, i - 9):i + 1]) / len(rq[max(0, i - 9):i + 1]) for i in range(len(rq))]
        b.plot([f['time'] for f in fr], sm, color=MODE_COLOR[mode], lw=1.4,
               ls='-' if mode in ('open', 'closure') else (0, (4, 2)), label=MODE_LABEL[mode])
        f480 = [f for f in fr if f['time'] == 480.0][0]
        mid.append((mode, f480['exits'], max(f['ramp_queue'] for f in fr)))
    NUM['exits_at_480s'] = {m: e for m, e, _ in mid}
    NUM['max_ramp_queue'] = {m: q for m, _, q in mid}
    NUM['arrivals_at_480s'] = round(cum_demand(480))
    e480 = NUM['exits_at_480s']['closure']
    a.annotate('', (480, e480), (480, cum_demand(480)), arrowprops=dict(arrowstyle='<->', color=INK, lw=0.8))
    a.text(470, (e480 + cum_demand(480)) / 2, f'{cum_demand(480) - e480:.0f} veh\nin system', fontsize=8,
           ha='right', va='center', color=INK)
    for ax in (a, b):
        ax.axvspan(240, 480, color=SHADE, lw=0, zorder=0)
        ax.set_xlabel('Time (s)')
        ax.set_xlim(0, 600)
    a.set_ylabel('Cumulative vehicles')
    a.legend(loc='lower right', fontsize=7.4)
    a.set_title('(a) Cumulative arrivals and exits', loc='left')
    b.set_ylabel('Vehicles waiting on ramp')
    b.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True))
    b.set_title('(b) Ramp queue (10 s average)', loc='left')
    b.legend(loc='upper left', fontsize=7.6)
    save(fig, 'fig04_cumulative_curves.png')


# --------------------------------------------------------------------- Fig. 5
def fig_estimate():
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.5), sharey=True)
    for ax, mode, title in zip(axes, ['adaptive', 'adaptive_fault'],
                               ['(a) Normal detectors', '(b) Missing and delayed reports']):
        fr = FWDATA['runs'][mode]['frames']
        t = [f['time'] for f in fr]
        truth = [len(f['vehicles']) if isinstance(f['vehicles'], list) else f['vehicles'] for f in fr]
        est = [f['estimate']['road_vehicles'] if f.get('estimate') and f['estimate'].get('road_vehicles') is not None
               else float('nan') for f in fr]
        err = [abs(e - v) for e, v in zip(est, truth) if e == e]
        NUM.setdefault('estimate_mae_veh', {})[mode] = round(sum(err) / len(err), 2)
        ax.plot(t, truth, color=BLUE, lw=1.5, label='True vehicles on road (hidden)')
        ax.plot(t, est, color=ORANGE, lw=1.4, label='Estimate from boundary counts')
        if mode == 'adaptive_fault':
            for f in FWDATA['config']['tmc']['faults']:
                ax.axvspan(f['start_s'], f['end_s'], color=AMBER, alpha=0.18, lw=0)
            ax.text(305, 6, 'missing', fontsize=8, color=MUTED)
            ax.text(425, 6, 'delayed 45 s', fontsize=8, color=MUTED)
        ax.set_title(title, loc='left')
        ax.set_xlabel('Time (s)')
        ax.set_xlim(0, 600)
    axes[0].set_ylabel('Vehicles')
    axes[0].legend(loc='lower right', fontsize=8)
    save(fig, 'fig05_estimate_vs_truth.png')


# --------------------------------------------------------------------- Fig. 6
SCEN = [('balanced', 'Balanced\n450 x 4'), ('directional_shift', 'Shift\n700 / 300'),
        ('detector_fault', 'Shift +\ndetector faults'), ('heavy', 'Heavy\n800 x 4')]
CTRL = [('fixed', 'Fixed time (Webster)', '#8f8d86'), ('actuated', 'Actuated', GREEN),
        ('adaptive', 'Adaptive (queue + forecast)', BLUE), ('adaptive_no_forecast', 'Adaptive, no forecast', '#b9c9e6')]


def ix_metric(scen, ctrl):
    return IXDATA['runs'][f'{scen}/{ctrl}']['metrics']


def fig_ix_delay():
    fig, ax = plt.subplots(figsize=(7.0, 2.65))
    w = 0.2
    table = {}
    for i, (c, lab, col) in enumerate(CTRL):
        vals = [ix_metric(s, c)['total_delay_vehicle_s'] / 60 for s, _ in SCEN]
        table[c] = vals
        xs = [k + (i - 1.5) * w for k in range(len(SCEN))]
        bars = ax.bar(xs, vals, w - 0.02, color=col, label=lab)
        if c != 'fixed':
            for bb, v, s in zip(bars, vals, SCEN):
                base = ix_metric(s[0], 'fixed')['total_delay_vehicle_s'] / 60
                ax.text(bb.get_x() + bb.get_width() / 2, v + 25, f'{100 * (v / base - 1):+.0f}%',
                        ha='center', fontsize=7, color=MUTED)
    NUM['ix_delay_vehmin'] = {c: [round(v, 1) for v in vals] for c, vals in table.items()}
    ax.set_xticks(range(len(SCEN)))
    ax.set_xticklabels([s[1] for s in SCEN])
    ax.set_ylabel('Total delay (veh-min)')
    ax.grid(axis='x', visible=False)
    ax.legend(loc='upper left', fontsize=8)
    save(fig, 'fig06_intersection_delay.png')


# --------------------------------------------------------------- Fig. 7 and 8
def phase_fig(key, fname, use_stopped, shade):
    run = IXDATA['runs'][key]
    fr = run['frames']
    t = [f['time'] for f in fr]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.0, 3.05), sharex=True,
                                 gridspec_kw={'height_ratios': [1, 2.6]})
    col = {'NS': BLUE, 'EW': ORANGE}
    for p in run['phases']:
        y = 1 if p['group'] == 'NS' else 0
        c = col[p['group']] if p['stage'] == 'green' else (AMBER if p['stage'] == 'yellow' else '#9a9893')
        a1.broken_barh([(p['start_s'], p['end_s'] - p['start_s'])], (y - 0.35, 0.7), color=c, lw=0, zorder=3)
    a1.set_yticks([0, 1])
    a1.set_yticklabels(['E-W green', 'N-S green'])
    a1.grid(False)
    a1.set_ylim(-0.6, 1.6)
    idx = {'N': 0, 'E': 2}
    for ap, c in (('N', BLUE), ('E', ORANGE)):
        i = idx[ap]
        truth = [f['stopped' if use_stopped else 'zone_truth'][i] for f in fr]
        est = [f['estimate'][i] if f['estimate'][i] is not None else float('nan') for f in fr]
        lab = 'true stopped queue' if use_stopped else 'true vehicles in loop zone'
        a2.plot(t, truth, color=c, lw=1.3, label=f'{ap}: {lab}')
        a2.plot(t, est, color=c, lw=1.3, ls=(0, (3, 2)), label=f'{ap}: estimate from loops')
        if use_stopped:
            NUM.setdefault('heavy_peak', {})[ap] = {'true': max(truth), 'estimate': max(e for e in est if e == e)}
    for ax in (a1, a2):
        ax.axvspan(*shade, color=SHADE, lw=0, zorder=0)
    a2.set_ylabel('Vehicles')
    a2.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True))
    a2.set_xlabel('Time (s)')
    a2.set_xlim(0, 900)
    a2.legend(loc='upper left', ncol=2, fontsize=7.8)
    save(fig, fname)


def summaries():
    rows = read_csv(FW / 'summary.csv')
    NUM['freeway_summary'] = {r['mode']: {'delay_vehmin': round(float(r['total_delay_vehicle_s']) / 60, 1),
                                          'exited': int(r['exited_vehicles'])} for r in rows}
    # congestion criterion of Q1 at the 550 m loop
    for mode in ('open', 'closure'):
        agg = defaultdict(lambda: [0, 0.0])
        for r in read_csv(FW / mode / 'observations.csv'):
            if r['position_m'] == '550' and r['mean_speed_mps']:
                c = int(r['count'])
                agg[float(r['interval_end_s'])][0] += c
                agg[float(r['interval_end_s'])][1] += c * float(r['mean_speed_mps'])
        sp = [(t, v[1] / v[0]) for t, v in sorted(agg.items()) if v[0]]
        run = best = 0
        for _, s in sp:
            run = run + 1 if s < 15 else 0
            best = max(best, run)
        NUM.setdefault('loop550', {})[mode] = {'longest_congested_run': best,
                                               'min_interval_speed_mps': round(min(s for _, s in sp), 1)}


if __name__ == '__main__':
    fig_layout()
    fig_contour()
    fig_fd()
    fig_cumulative()
    fig_estimate()
    fig_ix_delay()
    phase_fig('directional_shift/adaptive', 'fig07_phase_timeline_shift.png', False, (300, 660))
    phase_fig('heavy/adaptive', 'fig08_phase_timeline_heavy.png', True, (0, 600))
    summaries()
    with open(OUT / 'report_numbers.json', 'w') as f:
        json.dump(NUM, f, indent=2)
    print(json.dumps(NUM, indent=2))
