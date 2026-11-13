#!/usr/bin/env bash
# The gate for this ArduPilot fork: build ArduPlane SITL. The same command is used by people, by the
# git pre-commit hook (.githooks/pre-commit) and by the Claude Code Stop hook (.claude/settings.json).
# It lives here and not in scripts/, because ArduPilot ignores /scripts/: SITL loads Lua scripts from it.
#
# Usage: Tools/agentic-dev/check.sh [--if-changed] [--hook] [--full]
#   --if-changed  Skip when the checked files are the same as at the last pass, so a docs-only
#                 commit costs nothing.
#   --full        Also build and run the library unit tests (./waf check), which takes several minutes.
#   --hook        Claude Code Stop-hook mode: --if-changed, and exit 2 on failure so the session is
#                 told to fix it instead of finishing.
#
# The build runs in the Docker image AP_IMAGE (default ardupilot:Plane-4.5.4) as its user ardupilot
# (uid 1000), never as root. The tree is mounted at /data/datasets/code/ardupilot, the path the uavnav-cpp
# dev container sees it at, so the waf configuration in build/ stays valid from both. Inside a container
# that already has the tree at that path and no docker, waf runs directly.
set -uo pipefail

# ---- repo-specific ----------------------------------------------------------------------------
WATCHED=(ArduPlane libraries wscript Tools/ardupilotwaf)
steps() {    # one line per step: run_step <name> <lines of output shown on failure> <command...>
    run_step build 60 waf_in_image plane
}
full_steps() {    # slow checks, only with --full
    run_step unit-tests 80 waf_in_image check
}
# -----------------------------------------------------------------------------------------------

hook=0
if_changed=0
full=0
for arg in "$@"; do
    case "$arg" in
        --hook) hook=1; if_changed=1 ;;
        --if-changed) if_changed=1 ;;
        --full) full=1 ;;
        *) echo "usage: $0 [--if-changed] [--hook] [--full]" >&2; exit 2 ;;
    esac
done

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$root" || exit 1
max_blocks=3    # consecutive times the Stop hook may hold a session before it lets it stop

AP_IMAGE="${AP_IMAGE:-ardupilot:Plane-4.5.4}"
AP_CONTAINER_PATH=/data/datasets/code/ardupilot
AP_HOST_PATH=/mnt/nvme0n1/datasets/code/ardupilot

# State lives in the repository's .git directory (per working tree) and is never committed.
state_dir="$(git rev-parse --git-dir)" || exit 1
stamp="$state_dir/check_passed"
blocks="$state_dir/check_blocks"

# A key for the content of the checked files as they are on disk (tracked and untracked, not ignored).
state_key() {
    git ls-files -z -c -o --exclude-standard -- "${WATCHED[@]}" 2>/dev/null \
        | sort -z -u | xargs -0 -r sha1sum 2>&1 | sha1sum | cut -d' ' -f1
}

key="$(state_key)"
if [[ $if_changed -eq 1 && -f "$stamp" && "$(cat "$stamp")" == "$key" ]]; then
    exit 0
fi

# Run ./waf with the given arguments where the build environment is.
waf_in_image() {
    if [[ "$root" == "$AP_CONTAINER_PATH" ]] && ! command -v docker >/dev/null 2>&1; then
        ./waf "$@"
        return
    fi
    local mounts=() workdir
    if [[ "$(realpath "$root")" == "$(realpath "$AP_HOST_PATH" 2>/dev/null)" ]]; then
        workdir="$AP_CONTAINER_PATH"
        mounts=(-v "$root:$workdir")
    else
        # A worktree: mount it at its own path, with the main repository's .git for the version header.
        workdir="$root"
        local common
        common="$(realpath "$(git rev-parse --git-common-dir)")"
        mounts=(-v "$root:$root" -v "$common:$common")
    fi
    local configure=":"
    if [[ ! -f build/c4che/sitl_cache.py ]]; then
        configure="./waf configure --board sitl"
    fi
    docker run --rm --network none "${mounts[@]}" -w "$workdir" "$AP_IMAGE" \
        bash -c "$configure && ./waf $*"
}

log="$(mktemp)"
trap 'rm -f "$log"' EXIT
failed=""

run_step() {
    local name="$1" lines="$2"
    shift 2
    [[ -n "$failed" ]] && return
    if [[ "$1" == waf_in_image ]] && ! command -v docker >/dev/null 2>&1 && [[ "$root" != "$AP_CONTAINER_PATH" ]]; then
        echo "check: $name FAILED: docker is not available and this is not the uavnav-cpp dev container" >&2
        failed="$name"
        return
    fi
    if "$@" >"$log" 2>&1; then
        echo "check: $name ok"
    else
        echo "check: $name FAILED ($*)" >&2
        grep -E "error:|Error|FAILED" "$log" | head -n 20 >&2
        echo "--- last lines ---" >&2
        tail -n "$lines" "$log" >&2
        failed="$name"
    fi
}

steps
if [[ $full -eq 1 && $hook -eq 0 ]]; then
    full_steps
fi

if [[ -z "$failed" ]]; then
    echo "$key" >"$stamp"
    rm -f "$blocks"
    exit 0
fi

if [[ $hook -eq 1 ]]; then
    count=$(($(cat "$blocks" 2>/dev/null || echo 0) + 1))
    echo "$count" >"$blocks"
    if [[ $count -gt $max_blocks ]]; then
        rm -f "$blocks"
        echo "check: still failing after $max_blocks attempts; letting the session stop. The tree is NOT green." >&2
        exit 1
    fi
    echo "check: fix the $failed failure above before finishing, or say plainly why it cannot be fixed." >&2
    exit 2
fi
exit 1
