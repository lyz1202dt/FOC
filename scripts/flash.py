#!/usr/bin/env python3
"""Flash selected, already-built images using J-Link Commander."""

import argparse
from pathlib import Path
import shlex
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', nargs='+', required=True,
                        choices=('bootloader', 'app', 'config', 'all'),
                        help='images to flash; all includes default configuration')
    parser.add_argument('--preset', choices=('Debug', 'Release'), default='Debug')
    parser.add_argument('--build-dir', type=Path, help='override build/<preset>')
    parser.add_argument('--exe', default='JLinkExe', help='J-Link Commander executable')
    parser.add_argument('--serial-no', help='select a probe by serial number')
    parser.add_argument('--device', default='STM32G431CB')
    parser.add_argument('--interface', choices=('SWD', 'JTAG'), default='SWD')
    parser.add_argument('--speed', type=int, default=4000, help='debug clock in kHz')
    parser.add_argument('--dry-run', action='store_true', help='preview without connecting')
    args = parser.parse_args()
    if args.speed <= 0:
        parser.error('--speed must be positive')

    root = Path(__file__).resolve().parent.parent
    build = (args.build_dir or root / 'build' / args.preset).resolve()
    selected = set(args.image)
    if 'all' in selected:
        selected.update(('bootloader', 'app', 'config'))
    files = []
    for name, filename in (('bootloader', 'bootloader.elf'), ('app', 'foc.elf'),
                           ('config', 'default_config.hex')):
        if name in selected:
            files.append(build / filename)
    if 'config' in selected:
        files.append(build / 'default_config.bin')
    for path in files:
        if any(char in str(path) for char in ('"', '\n', '\r')):
            parser.error(f'unsupported image path: {path!s}')
        if not args.dry_run and not path.is_file():
            parser.error(f'missing image: {path}; build the selected images first')

    commands = ['r', 'h']
    for name, filename in (('bootloader', 'bootloader.elf'), ('app', 'foc.elf')):
        if name in selected:
            commands.append(f'loadfile "{(build / filename).as_posix()}"')
    if 'config' in selected:
        commands.extend([
            'erase 0x0801F000 0x0801FFFF',
            f'loadfile "{(build / "default_config.hex").as_posix()}"',
            f'verifybin "{(build / "default_config.bin").as_posix()}" 0x0801F000',
        ])
    commands.extend(['r', 'g', 'exit'])
    script = '\n'.join(commands) + '\n'
    command = [args.exe, '-NoGui', '1', '-ExitOnError', '1', '-AutoConnect', '1',
               '-device', args.device, '-if', args.interface, '-speed', str(args.speed)]
    if args.serial_no:
        command.extend(['-USB', args.serial_no])
    print(f'Target: {args.device}, interface: {args.interface}, images: '
          + ', '.join(name for name in ('bootloader', 'app', 'config') if name in selected),
          flush=True)
    if args.dry_run:
        print(shlex.join(command + ['-CommanderScript', '<temporary flash.jlink>']))
        print(script, end='')
        return 0
    with tempfile.TemporaryDirectory(prefix='foc-flash-') as directory:
        path = Path(directory) / 'flash.jlink'
        path.write_text(script, encoding='utf-8')
        try:
            return subprocess.run(command + ['-CommanderScript', str(path)],
                                  check=False).returncode
        except OSError as error:
            parser.exit(1, f'cannot start J-Link Commander: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
