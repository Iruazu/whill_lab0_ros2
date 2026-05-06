# M1 — ROS 2 humble environment setup

## Goal

Have a working ROS 2 humble installation on the lab PC and an empty colcon
workspace at `~/ros2_ws`, so subsequent milestones can build packages.

## Host

| | |
|--|--|
| OS | Ubuntu 22.04.5 LTS (jammy) |
| Kernel | 6.8.0-111-generic |
| Network | behind `proxy.cc.utsunomiya-u.ac.jp:8080` (Utsunomiya University) |
| GitHub access | SSH tunneled via HTTP CONNECT to `ssh.github.com:443` |

## Procedure

The `scripts/` directory in this repo automates this. To reproduce on a clean
PC:

```bash
# 1. (optional) configure apt + SSH for the campus proxy
./scripts/configure_proxy.sh

# 2. Install ROS 2 humble Desktop + dev tools
./scripts/install_ros2_humble.sh

# 3. Create ~/ros2_ws and add sourcing to ~/.bashrc
./scripts/setup_workspace.sh
```

Each script is idempotent and contains no embedded credentials.

### What the install script does

1. Verifies the host is jammy (Ubuntu 22.04).
2. Installs the `locales` package and generates `en_US.UTF-8`.
3. Enables the `universe` apt component.
4. Downloads the latest `ros2-apt-source` deb from
   `ros-infrastructure/ros-apt-source` (currently 1.2.0) and installs it. This
   is the official mechanism since 2024 — the older `ros-archive-keyring.gpg`
   approach is deprecated.
5. `apt-get install ros-humble-desktop ros-dev-tools`.

### Proxy notes

- Direct outbound SSH (port 22 or 443) to GitHub is blocked from this network.
- `~/.ssh/config` configures `Host github.com` to use
  `ProxyCommand nc -X connect -x proxy.cc.utsunomiya-u.ac.jp:8080 %h %p` over
  `ssh.github.com:443`. This is what `git clone git@github.com:…` ultimately
  goes through.
- `/etc/apt/apt.conf.d/95proxies` makes apt itself use the same proxy.
- Shell-level `HTTP_PROXY` / `HTTPS_PROXY` env vars are already set system-wide
  on this PC; preserve them with `sudo -E` if you need them inside `sudo`.

## Verification

After the workspace is set up:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
# in a second shell:
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

The listener should print "I heard: …" messages emitted by the talker.

## Outputs

- `/opt/ros/humble/` — ROS 2 humble installation
- `~/ros2_ws/src/` — empty colcon source space, ready for M2+
- `~/.bashrc` — sources `humble` and the workspace overlay automatically

## Status

| Step | Status |
|------|--------|
| Apt proxy configured | done |
| SSH-via-proxy to GitHub working | done |
| `ros-humble-desktop` installed | in progress (PR open) |
| `ros2 talker/listener` round-trip | pending |
| `~/ros2_ws` initialized | pending |
| `.bashrc` sourcing in place | pending |

This document will be updated as remaining steps complete.
