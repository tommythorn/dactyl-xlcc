#!/usr/bin/env bash
# Builds the Truck port and generates the STL files in ./out.
set -euo pipefail
cd "$(dirname "$0")"
cargo build --release
./target/release/dactyl-truck --out out
ls -lh out
