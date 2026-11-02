#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Check that the DDS XML names the NIC the LiDAR is actually on (#140).

The XML marks the NIC presence_required="false", so CycloneDDS starts without it and
gives no error. An old NIC name then leaves some terminals on a different network
from others. Exit 0 if consistent, 1 with the reason if not.
"""
import argparse
import ipaddress
import json
import subprocess
import sys
import xml.etree.ElementTree as ET

# network/01-velodyne-static.yaml.template puts the LiDAR NIC at 192.168.1.100/24.
LIDAR_SUBNET = '192.168.1.0/24'


def xml_interfaces(path):
    names = []
    for elem in ET.parse(path).getroot().iter():
        if elem.tag.rsplit('}', 1)[-1] == 'NetworkInterface' and elem.get('name'):
            names.append(elem.get('name'))
    return names


def host_addresses():
    """Return {nic: [IPv4 address, ...]}."""
    out = subprocess.run(['ip', '-j', '-4', 'addr'], check=True, capture_output=True,
                         text=True).stdout
    return {link['ifname']: [a['local'] for a in link.get('addr_info', [])]
            for link in json.loads(out)}


def find_problems(xml_names, addresses, subnet):
    net = ipaddress.ip_network(subnet)
    wired = [name for name in xml_names if name != 'lo']
    on_subnet = [nic for nic, ips in addresses.items()
                 if any(ipaddress.ip_address(ip) in net for ip in ips)]
    problems = []
    if not wired:
        problems.append('XML に lo 以外の NIC がありません')
    for name in wired:
        if name not in addresses:
            problems.append(f'XML の NIC が実機にありません: {name}'
                            f'（実機: {", ".join(sorted(addresses))}）')
    if not on_subnet:
        problems.append(f'{subnet} を持つ NIC がありません。LiDAR の USB-LAN の接続を確認してください')
    for nic in on_subnet:
        if wired and nic not in wired:
            problems.append(f'{subnet} の NIC {nic} が XML にありません（XML: {", ".join(wired)}）')
    return problems


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('xml')
    ap.add_argument('--subnet', default=LIDAR_SUBNET)
    args = ap.parse_args()
    try:
        names = xml_interfaces(args.xml)
    except (OSError, ET.ParseError) as exc:
        print(f'DDS の XML を読めません: {args.xml}: {exc}')
        return 1
    problems = find_problems(names, host_addresses(), args.subnet)
    for problem in problems:
        print(problem)
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
