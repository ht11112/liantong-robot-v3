"""Run software-only checks against production code and a completed MCU build."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'rk/catkin_ws/src/liantong_v3'
sys.path.insert(0, str(PACKAGE / 'src'))

from liantong_v3.core import LocalizationSessionEnded, MeasurementClock, NS, parse_feedback


def check_mcu(receipt_path):
    receipt = json.loads(receipt_path.read_text(encoding='utf-8'))
    if not receipt['passed'] or receipt['hardware_accessed'] or receipt['flashed']:
        raise ValueError('Need a successful offline MCU receipt')
    if not receipt.get('tests') or not receipt.get('firmware'):
        raise ValueError('Receipt must include production C tests and ARM firmware')
    for name, expected in receipt['source_sha256'].items():
        if hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != expected:
            raise ValueError('Source changed since MCU build: ' + name)
    binary = receipt_path.parent / 'mcu-v3.bin'
    if hashlib.sha256(binary.read_bytes()).hexdigest() != receipt['firmware']['bin_sha256']:
        raise ValueError('Built firmware does not match receipt')
    return receipt['firmware']


def check_cross_language(log):
    lines = [line.split('|', 1)[1].encode('ascii') + b'\n'
             for line in log.read_text(encoding='utf-8').splitlines() if line.startswith('GOLDEN|')]
    if len(lines) != 5:
        raise ValueError('Expected five production C formatter records from test_wire')
    boot, valid, duplicate, faulted, recovered = [parse_feedback(line) for line in lines]
    if (boot.valid or boot.sequence != 0 or not valid.valid or valid.sequence != 1
            or duplicate.sequence != valid.sequence or duplicate.measurement_ms != valid.measurement_ms
            or faulted.valid or not faulted.stop_latched or faulted.fault != 2
            or not recovered.valid or not recovered.stop_latched or recovered.fault != 0
            or recovered.segment != faulted.segment or recovered.segment == valid.segment):
        raise ValueError('MCU/RK golden record contract mismatch')
    clock = MeasurementClock()
    if clock.accept(boot, 2*NS, 2*NS + 20_000_000, 1002*NS + 20_000_000) is not None:
        raise ValueError('Boot invalid sample published')
    stamp = clock.accept(valid, 3*NS, 3*NS + 20_000_000, 1003*NS + 20_000_000)
    expected = 1003*NS - ((valid.reply_ms - valid.measurement_ms) & 0xffffffff)*1_000_000
    if stamp != expected:
        raise ValueError('MCU/RK source-time mapping differs')
    if clock.accept(duplicate, 4*NS, 4*NS + 20_000_000, 1004*NS + 20_000_000) is not None:
        raise ValueError('Duplicate C sample refreshed odometry')
    try:
        clock.accept(faulted, 5*NS, 5*NS + 20_000_000, 1005*NS + 20_000_000)
    except LocalizationSessionEnded:
        pass
    else:
        raise ValueError('MCU continuity break crossed into old TF session')
    return len(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mcu-receipt', type=Path, required=True)
    parser.add_argument('--report', type=Path, default=ROOT / 'build/offline_validation.json')
    args = parser.parse_args()
    firmware = check_mcu(args.mcu_receipt.resolve())
    suite = unittest.defaultTestLoader.discover(str(PACKAGE / 'tests'), pattern='test_*.py')
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    if not result.wasSuccessful():
        raise SystemExit(1)
    golden_count = check_cross_language(args.mcu_receipt.parent / 'test_wire.log')
    python_files = list((ROOT / 'mcu').rglob('*.py')) + list((ROOT / 'rk').rglob('*.py'))
    python_files += list((ROOT / 'scripts').glob('*.py'))
    for path in python_files:
        ast.parse(path.read_text(encoding='utf-8'), filename=str(path), feature_version=(3, 8))
    xml_files = [PACKAGE / 'package.xml', *sorted((PACKAGE / 'launch').glob('*.launch'))]
    for path in xml_files:
        ET.parse(path)
    report = dict(passed=True, hardware_accessed=False, flashed=False,
                  ros_runtime_executed=False, rk_tests=result.testsRun,
                  c_formatter_to_python_records=golden_count,
                  python38_syntax_files=len(python_files), xml_files=len(xml_files),
                  firmware_bytes=firmware['bytes'], firmware_sha256=firmware['bin_sha256'])
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
