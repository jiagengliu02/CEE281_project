"""Single command-line entry point for both traffic environments.

Examples:
    python3 run.py freeway
    python3 run.py intersection --scenario directional_shift --seed 11
    python3 run.py check all
"""
import argparse
from importlib import import_module
import json
import shutil
from pathlib import Path
import sys

# Keep disposable bytecode caches out of this small teaching project.
sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent
BRANCHES = ('freeway', 'intersection')


def main(argv=None):
    parser = argparse.ArgumentParser(description='Run or verify the CEE281 traffic environments.')
    commands = parser.add_subparsers(dest='command', required=True)
    for branch in BRANCHES:
        command = commands.add_parser(branch, help=f'Run the {branch} experiments and build its viewer')
        command.add_argument('--config', type=Path, default=ROOT / branch / 'configs/default.json',
                             help=f'Configuration JSON (default: {branch}/configs/default.json)')
        command.add_argument('--output', type=Path, default=ROOT / 'results' / branch,
                             help=f'Result directory (default: results/{branch})')
        if branch == 'intersection':
            command.add_argument('--scenario', action='append',
                                 help='Select a scenario; repeat for several (default: all)')
            command.add_argument('--seed', type=int, action='append',
                                 help='Select a seed; repeat for several (default: config seeds)')
    check = commands.add_parser('check', help='Run independent verification checks')
    check.add_argument('branch', choices=(*BRANCHES, 'all'), nargs='?', default='all')
    args = parser.parse_args(argv)

    if args.command == 'check':
        if not __debug__:
            parser.error('Verification uses assertions; run Python without -O.')
        branches = BRANCHES if args.branch == 'all' else (args.branch,)
        passed = True
        for branch in branches:
            print(f'Checking {branch}...', flush=True)
            outcome = import_module(f'{branch}.verify').verify()
            passed = outcome and passed
        return 0 if passed else 1

    config = json.loads(args.config.read_text(encoding='utf-8'))
    output = args.output.resolve()
    runner = import_module(f'{args.command}.run')
    if args.command == 'intersection':
        if args.scenario:
            unknown = set(args.scenario) - set(config['scenarios'])
            if unknown:
                parser.error('Unknown intersection scenarios: ' + ', '.join(sorted(unknown)))
        runner.run_experiments(config, output, args.scenario, args.seed)
    else:
        runner.run_experiments(config, output)
    viewer = output / 'viewer.html'
    if output == (ROOT / 'results' / args.command).resolve():
        published = ROOT / f'{args.command}.html'
        shutil.copy2(viewer, published)
        print('Project viewer:', published)
    print('Viewer:', viewer)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
