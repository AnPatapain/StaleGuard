"""Verify published file checksums, benchmark grids and the E01 record archive."""

import argparse
import json
from pathlib import Path
from .common import Data, MISSIONS
from staleguard.robustness.archive import verify_archive


def verify(data_root):
    data = Data(data_root)
    for name in data.manifest['files']:
        data.file(name)
    counts = {}
    for mission in MISSIONS:
        rows, _, _ = data.policy_records(mission)
        counts[mission] = len(rows)
    robustness = verify_archive(data_root)
    return dict(
        files_verified=len(data.used),
        benchmark_policy_outcomes=counts,
        benchmark_total=sum(counts.values()),
        robustness_outcomes=robustness['outcomes'],
        robustness_predictions=robustness['forecasts'],
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', type=Path, default=Path('data'))
    args = parser.parse_args()
    print(json.dumps(verify(args.data_root), indent=2))


if __name__ == '__main__':
    main()
