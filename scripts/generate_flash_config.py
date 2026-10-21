#!/usr/bin/env python3
"""Serialize YAML into the Flash v2 journal (little endian, numeric values)."""
import argparse
import json
import math
from pathlib import Path
import struct
import yaml


def checksum(data, seed=2166136261):
    for byte in data:
        seed = ((seed ^ byte) * 16777619) & 0xFFFFFFFF
    return seed


class UniqueLoader(yaml.SafeLoader):
    pass


def mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node)
        if not isinstance(key, str) or key in result:
            raise ValueError(f'invalid or duplicate YAML key: {key!r}')
        result[key] = loader.construct_object(value_node)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, mapping)


def serialize(document):
    if not isinstance(document, dict) or set(document) != {'config'}:
        raise ValueError('expected a single config mapping')
    entries, report = [], []

    def visit(node, path):
        if isinstance(node, dict):
            for key in sorted(node):
                if not key or '/' in key or '\0' in key:
                    raise ValueError(f'invalid path component: {key!r}')
                visit(node[key], path + [key])
            return
        key = '/'.join(path)
        if not key or not isinstance(node, list) or len(node) != 2:
            raise ValueError(f'{key}: expected [type, value]')
        kind, value = node
        formats = {'int32': 'i', 'uint32': 'I', 'int64': 'q', 'uint64': 'Q',
                   'float32': 'f', 'float64': 'd'}
        if kind in formats:
            integer = kind.startswith(('int', 'uint'))
            if isinstance(value, bool) or not isinstance(value, int if integer else (int, float)):
                raise ValueError(f'{key}: invalid {kind} value')
            if not integer and not math.isfinite(value):
                raise ValueError(f'{key}: non-finite value')
            raw = struct.pack('<' + formats[kind], value)
        else:
            raise ValueError(f'{key}: unsupported type {kind!r}')
        encoded = key.encode('ascii')
        payload = encoded + raw
        padded = (len(payload) + 7) & ~7
        total = 16 + padded
        if total > 2000:
            raise ValueError(f'{key}: record exceeds page capacity')
        entries.append(struct.pack('<HHHH', len(encoded), len(raw), 1, total)
                       + payload + b'\xff' * (padded - len(payload)) + b'\xff' * 8)
        report.append({'key': key, 'type': kind, 'length': len(raw), 'value': value})

    visit(document['config'], [])
    size = 24 + sum(map(len, entries))
    if size + 24 > 2048:
        raise ValueError('configuration exceeds one 2048-byte page')
    prefix = struct.pack('<IHH', 0x3242544B, size, len(entries))
    digest = checksum(prefix)
    for entry in entries:
        digest = checksum(entry[:-8], digest)
    batch = prefix + struct.pack('<II', digest, 0xFFFFFFFF) + b''.join(entries)
    batch += struct.pack('<II', 0x32544D43, 0x32544D43 ^ 0xFFFFFFFF)
    header = struct.pack('<IHHI', 0x324C564B, 2, 24, 1)
    page = header + struct.pack('<I', checksum(header)) + struct.pack('<II', 0x32544341, 1) + batch
    return page.ljust(4096, b'\xff'), {'used_bytes': len(page), 'entries': report}


def hex_record(address, kind, data):
    packet = bytes([len(data)]) + struct.pack('>H', address) + bytes([kind]) + data
    return ':' + (packet + bytes([-sum(packet) & 255])).hex().upper()


def write_outputs(source, output):
    image, report = serialize(yaml.load(source.read_text(encoding='utf-8'), Loader=UniqueLoader))
    output.mkdir(parents=True, exist_ok=True)
    (output / 'default_config.bin').write_bytes(image)
    records = [hex_record(0, 4, b'\x08\x01')]
    # Omit erased doublewords: preserve ECC for future journal appends/retirement.
    for offset in range(0, len(image), 8):
        word = image[offset:offset + 8]
        if word != b'\xff' * 8:
            records.append(hex_record(0xF000 + offset, 0, word))
    records.append(hex_record(0, 1, b''))
    (output / 'default_config.hex').write_text('\n'.join(records) + '\n', encoding='ascii')
    (output / 'default_config.json').write_text(json.dumps(report, indent=2) + '\n', encoding='ascii')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    try:
        write_outputs(args.input, args.output_dir)
    except (ValueError, TypeError, OverflowError, struct.error, yaml.YAMLError) as error:
        parser.exit(1, f'configuration error: {error}\n')
