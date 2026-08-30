#!/usr/bin/env python3
"""Emit the subset of a vcstool .repos file whose packages apt cannot provide.

CI builds this package against binary .debs wherever they exist. A dependency
that is released in rosdistro but not yet synced to the distro's apt repo is
neither installable nor optional, so the build fails at rosdep with no way for
this repository to fix it. Cloning every dependency instead would trade that
for a much slower build on the distros where the binary is fine.

So: probe each entry, clone only what is missing. The .deb name is derived from
the entry KEY (the package name) as `ros-<distro>-<key with _ -> ->`; the probe
is a dry-run `apt-get install -s`, which resolves the package and its own
dependencies without touching the system.

An entry may also set `always: true`, for a dependency whose released binary
EXISTS but is too old to build against. apt can install those, so the probe
would wrongly clear them; this says "source, regardless". The key is stripped
before the document is emitted, since vcstool would not know it.

Usage: filter_unavailable_deps.py <dependencies.repos> <ros_distro>
Writes a .repos document to stdout (an empty `repositories:` map when nothing
is missing, which `vcs import` accepts as a no-op).
"""

import subprocess
import sys

import yaml


def apt_can_install(deb: str) -> bool:
    r = subprocess.run(
        ["apt-get", "install", "-s", "-y", deb],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return r.returncode == 0


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__, file=sys.stderr)
        return 2
    repos_file, distro = sys.argv[1], sys.argv[2]

    with open(repos_file, encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}
    entries = doc.get("repositories") or {}

    missing = {}
    for name, spec in entries.items():
        spec = dict(spec or {})
        always = bool(spec.pop("always", False))
        deb = "ros-{}-{}".format(distro, name.replace("_", "-"))
        if always:
            print("[deps] {}: pinned to source, building from source".format(name), file=sys.stderr)
        elif apt_can_install(deb):
            print("[deps] {}: using binary {}".format(name, deb), file=sys.stderr)
            continue
        else:
            print(
                "[deps] {}: no installable {}, building from source".format(name, deb),
                file=sys.stderr,
            )
        missing[name] = spec

    yaml.safe_dump({"repositories": missing}, sys.stdout, default_flow_style=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
