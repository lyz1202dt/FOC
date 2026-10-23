#!/usr/bin/env python3
"""Flash selected, already-built images using OpenOCD."""

import argparse
from pathlib import Path
import shlex
import subprocess
import tempfile


def tcl_word(value):
    """Quote one Tcl argument without allowing substitution or command injection."""
    escaped = ''.join('\\' + char if char in '\\"$[]' else char for char in str(value))
    return '"' + escaped.replace('\n', '\\n').replace('\r', '\\r') + '"'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', nargs='+', required=True,
                        choices=('bootloader', 'app', 'config', 'all'),
                        help='images to flash; all includes default configuration')
    parser.add_argument('--preset', choices=('Debug', 'Release'), default='Debug')
    parser.add_argument('--build-dir', type=Path, help='override build/<preset>')
    parser.add_argument('--exe', default='openocd', help='OpenOCD executable')
    parser.add_argument('--config', type=Path, help='OpenOCD project config (default: openocd.cfg)')
    parser.add_argument('--search', action='append', default=[], help='additional OpenOCD script search directory')
    parser.add_argument('--serial-no', help='select a probe by serial number')
    parser.add_argument('--interface', help='OpenOCD adapter config, e.g. interface/cmsis-dap.cfg')
    parser.add_argument('--transport', help='OpenOCD transport, e.g. swd or hla_swd')
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
        if not args.dry_run and not path.is_file():
            parser.error(f'missing image: {path}; build the selected images first')

    commands = ['init', 'reset init']
    for name, filename in (('bootloader', 'bootloader.elf'), ('app', 'foc.elf')):
        if name in selected:
            image = tcl_word((build / filename).as_posix())
            commands.append(f'program {image} verify')
    if 'config' in selected:
        commands.extend([
            'flash erase_address 0x0801F000 0x1000',
            f'flash write_image {tcl_word((build / "default_config.hex").as_posix())}',
            f'verify_image {tcl_word((build / "default_config.bin").as_posix())} 0x0801F000 bin',
        ])
    commands.extend(['reset run', 'shutdown'])
    script = '\n'.join(commands) + '\n'
    command = [args.exe, '-s', str(root)]
    for search in args.search:
        command.extend(['-s', search])
    for variable, value in (('PROBE_INTERFACE', args.interface),
                            ('PROBE_TRANSPORT', args.transport),
                            ('PROBE_SERIAL', args.serial_no),
                            ('ADAPTER_SPEED', str(args.speed))):
        if value is not None:
            command.extend(['-c', f'set {variable} {tcl_word(value)}'])
    config = (args.config or root / 'openocd.cfg').resolve()
    if not config.is_file():
        parser.error(f'missing OpenOCD config: {config}')
    command.extend(['-f', str(config), '-c', 'gdb_port disabled; tcl_port disabled; telnet_port disabled'])
    print('Images: ' + ', '.join(name for name in ('bootloader', 'app', 'config')
                               if name in selected), flush=True)
    if args.dry_run:
        print(shlex.join(command + ['-f', '<temporary flash.cfg>']))
        print(script, end='')
        return 0
    with tempfile.TemporaryDirectory(prefix='foc-flash-') as directory:
        path = Path(directory) / 'flash.cfg'
        path.write_text(script, encoding='utf-8')
        try:
            return subprocess.run(command + ['-f', str(path)],
                                  check=False).returncode
        except OSError as error:
            parser.exit(1, f'cannot start OpenOCD: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
