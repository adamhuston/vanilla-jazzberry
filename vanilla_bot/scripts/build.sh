#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="$ROOT/bot/mx_jazzhands"
# Keep the canonical workspace/overlay; discover the Vanilla package as well.
bash "$WS/scripts/build_dev.sh" \
  --base-paths "$WS/src" "$ROOT/vanilla_bot/vanilla_bringup" "$@"
