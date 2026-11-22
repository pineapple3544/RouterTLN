#!/usr/bin/env bash
set -euo pipefail
if [ "$#" -lt 1 ]; then echo 'Usage: collect.sh EPISODE_NAME [seconds=60] [obstacles=1] [seed=2026]'; exit 2; fi
name=$1
if [[ ! "$name" =~ ^[a-zA-Z0-9_-]+$ ]]; then echo 'Use letters, digits, underscore, or hyphen for episode name'; exit 2; fi
episode="/workspace/data/$name.elds"
if [ -e "$episode" ]; then echo "Refusing to overwrite $episode"; exit 2; fi
# Only demonstration collection. No training command is invoked.
exec bash /workspace/scripts/smoke.sh "${2:-60}" "${3:-1}" teacher "$name" "$episode" "${4:-2026}"
