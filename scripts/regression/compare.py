# SPDX-License-Identifier: BSD-3-Clause
"""Compare a replay result with the baseline limits (#142). Needs no ROS."""


def compare(result, baseline):
    """Return one message per limit broken, for the sections that were measured."""
    failures = []
    for section, limits in baseline['limits'].items():
        if section not in result:
            continue
        for key, limit in limits.items():
            value = result[section][key]
            if 'max' in limit and value > limit['max']:
                failures.append(f'{section}.{key}={value} > {limit["max"]}')
            if 'min' in limit and value < limit['min']:
                failures.append(f'{section}.{key}={value} < {limit["min"]}')
    return failures
