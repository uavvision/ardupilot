#!/usr/bin/env bash
# Launch ArduPlane SITL with the visual course-hold test parameters.
#
# 1. Resets stored parameters (eeprom.bin / flash.dat) so previous runs have
#    no influence.
# 2. Applies the canonical parameter set from Tools/scripts/visualcourse.parm
#    via the SITL -P NAME=VALUE option.
#
# Usage:
#   Tools/scripts/visualcourse_sitl.sh            # canonical untuned run
#   Tools/scripts/visualcourse_sitl.sh --autotune # also set VC_AUTOTUNE=1
#
# Then run the flight harness in another terminal:
#   python3 Tools/scripts/visualcourse_cruise_test.py --port 5770
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PARM_FILE="$ROOT/Tools/scripts/visualcourse.parm"
INSTANCE="${SITL_INSTANCE:-1}"
BASE_PORT=$((5760 + INSTANCE * 10))

cd "$ROOT"

# reset stored parameters to firmware defaults
rm -f eeprom.bin flash.dat fram.dat

# build -P NAME=VALUE flags from the .parm file (skip comments and blanks)
PARM_ARGS=()
while read -r name value rest; do
    [[ -z "$name" || "$name" == \#* ]] && continue
    [[ -n "$rest" ]] && { echo "bad .parm line: $name $value $rest" >&2; exit 1; }
    PARM_ARGS+=(-P "$name=$value")
done < "$PARM_FILE"

# optional autotune excitation
if [[ "${1:-}" == "--autotune" ]]; then
    PARM_ARGS+=(-P "VC_AUTOTUNE=1")
fi

echo "Launching ArduPlane SITL (instance $INSTANCE, port $BASE_PORT) with:" >&2
printf '  %s\n' "${PARM_ARGS[@]}" >&2

exec "$ROOT/build/sitl/bin/arduplane" -S -I"$INSTANCE" \
    --model plane --speedup 5 \
    --home=-35.363261,149.165230,584,353 \
    "${PARM_ARGS[@]}"
