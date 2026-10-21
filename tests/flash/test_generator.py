"""Exercise input validation and generate a cross-language reader fixture."""
import importlib.util
from pathlib import Path
import sys
sys.dont_write_bytecode = True

spec = importlib.util.spec_from_file_location('generator', Path(__file__).resolve().parents[2] / 'scripts/generate_flash_config.py')
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)
fixture = {'config': {'test': {
    'i32': ['int32', -(2**31)], 'u32': ['uint32', 2**32 - 1],
    'i64': ['int64', -(2**63)], 'u64': ['uint64', 2**64 - 1],
    'f32': ['float32', 1.5], 'f64': ['float64', -2.5]}}}
for invalid in (['str', 'ABC'], ['str', ''], ['str', '中文'], ['str', 'a\0b'], ['uint32', -1], ['int32', 2**31],
                ['float32', float('inf')], ['int32', True]):
    try:
        generator.serialize({'config': {'bad': invalid}})
    except (ValueError, OverflowError, generator.struct.error):
        pass
    else:
        raise AssertionError(f'accepted {invalid!r}')
try:
    generator.serialize({'config': {f'key{i}': ['int64', i] for i in range(100)}})
except ValueError:
    pass
else:
    raise AssertionError('accepted configuration exceeding page capacity')
try:
    generator.yaml.load('config:\n  a: [int32, 1]\n  a: [int32, 2]\n', Loader=generator.UniqueLoader)
except ValueError:
    pass
else:
    raise AssertionError('accepted duplicate YAML key')
output = Path(sys.argv[1])
source = output / 'fixture.yaml'
source.write_text(generator.yaml.safe_dump(fixture))
generator.write_outputs(source, output)
image = (output / 'default_config.bin').read_bytes()
# Reconstruct sparse HEX and validate checksums, addresses and erased holes.
restored = bytearray(b'\xff' * 4096)
for line in (output / 'default_config.hex').read_text().splitlines():
    record = bytes.fromhex(line[1:])
    assert sum(record) % 256 == 0
    if record[3] == 0:
        address = int.from_bytes(record[1:3], 'big') - 0xF000
        assert record[0] == 8 and record[4:-1] != b'\xff' * 8
        restored[address:address + 8] = record[4:-1]
assert restored == image
print('Flash configuration generator tests passed')
