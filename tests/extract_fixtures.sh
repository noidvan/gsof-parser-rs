#!/usr/bin/env bash
# Extract raw Trimcomm byte streams from pcap files into .bin fixtures.
# Each .bin is the concatenated TCP payload bytes — ready to feed into FrameParser.
#
# Usage: ./tests/extract_fixtures.sh [pcap_dir]
set -euo pipefail

PCAP_DIR="${1:-/home/hs293go/dev_ws/src/trimble_driver_ros/trimble_driver/test/data}"
OUT_DIR="$(dirname "$0")/fixtures"
mkdir -p "$OUT_DIR"

count=0
for pcap in "$PCAP_DIR"/*.pcap; do
    name="$(basename "$pcap" .pcap)"
    out="$OUT_DIR/${name}.bin"

    # Extract TCP payload as hex, concatenate all packets, convert to binary
    hex=$(tshark -r "$pcap" -T fields -e tcp.payload -Y tcp.payload 2>/dev/null | tr -d '\n ')
    if [ -z "$hex" ]; then
        echo "SKIP $name (no TCP payload)"
        continue
    fi

    echo "$hex" | xxd -r -p > "$out"
    size=$(wc -c < "$out")
    echo "OK   $name -> ${size} bytes"
    count=$((count + 1))
done

echo "Extracted $count fixtures to $OUT_DIR"
