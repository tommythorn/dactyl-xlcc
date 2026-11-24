#!/usr/bin/env bash
# Builds the generator and writes the STL and 3MF files to ./out.
set -euo pipefail
cd "$(dirname "$0")"
cargo build --release
./target/release/dactyl-truck --out out --format stl,3mf
ls -lh out
