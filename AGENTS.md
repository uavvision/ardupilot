# Project Guidelines

This is `uavvision/ardupilot`, a fork of ArduPilot with local patches for GPS-denied fixed-wing flight. Each firmware
line has its own branch (`Plane-4.5.4-uavnav` and its feature branches, `Plane-4.6.2-uavnav`, `ghuba/*` for 4.7). The
branches, their upstream bases and the local patches are listed in the shared knowledge base:
`knowledge/ardupilot/fork.md` in the `agentic-dev` repository (`/home/taras/Code/Projects/agentic-dev`).

## Who depends on this tree

- `uavnav-cpp` runs SITL from this checkout (`scripts/run_sitl.sh`, `ARDUPILOT_DIR=/data/datasets/code/ardupilot` in
  its dev container). The built `build/sitl/bin/arduplane` is used by its sessions.
- `fixed-wing-net-landing`, `easySLAM` and `vision` fly firmware built from these branches and ask questions about it
  through the `ardupilot-researcher` agent.
- So: never switch branches, stash or reset here while another session may be using it. Read other branches with
  `git show <ref>:<path>` and `git grep <pattern> <ref>`, or work in a worktree.

## Checks

- **Gate:** `Tools/agentic-dev/check.sh` builds ArduPlane SITL (`./waf plane`) in the Docker image
  `ardupilot:Plane-4.5.4` (about 10 s when little changed; a full rebuild takes about 4 minutes). `--full` also builds
  and runs the library unit tests (`./waf check`). The git pre-commit hook ([.githooks/pre-commit](.githooks/pre-commit),
  enable once per clone with `git config core.hooksPath .githooks`) and a Claude Code `Stop` hook
  (`.claude/settings.json`) run it when `ArduPlane/`, `libraries/`, `wscript` or `Tools/ardupilotwaf/` changed.
- The gate lives under `Tools/`, not `scripts/`: ArduPilot ignores `/scripts/`, where SITL loads Lua scripts from.
- The image mounts the tree at `/data/datasets/code/ardupilot`, the path the uavnav-cpp dev container uses, so the waf
  configuration in `build/` is valid from both. Do not reconfigure it at another path.
- **Never build as root.** A root build left 25,641 root-owned files here (fixed 2026-11-13). Inside the uavnav-cpp
  container, which runs as root, return ownership afterwards (`chown -R 1000:1000`).
- Behaviour changes need a SITL flight or an autotest (`Tools/autotest/autotest.py`, `Tools/autotest/arduplane.py`)
  that shows them, and the report says which was run, or that none was.

## Code style

Upstream ArduPilot conventions, not the shared `code_style.md` of the other projects, so that patches stay easy to
rebase and to offer upstream:

- C++ formatting as `Tools/CodeStyle/astylerc` (Linux style, 4 spaces). Follow the surrounding file.
- Commit subjects prefixed with the subsystem, as upstream does: `AP_GPS: accept two MAVLink GPS instances`. One
  subsystem per commit where possible, so a patch can be carried to the next firmware line by cherry-pick.
- New parameters get the full `// @Param` metadata block.
- Mark local behaviour changes in a comment that names the reason, so the next merge of an upstream release can find
  them.

## Workflow

- Work on the branch of the firmware line the change is for, and say in the report which branch and commit.
- When a local patch is added, moved or reverted, update `knowledge/ardupilot/fork.md` in `agentic-dev`, or say in the
  report that it needs updating.
- Do not push to `uavvision/ardupilot`, and do not touch the `ghuba/*` or other people's branches, without the
  owner's word.
- Agents are in `.claude/agents/`. They are copies; the master files are in the `agentic-dev` repo, so change them
  there. `ardupilot-researcher` answers questions about ArduPilot internals and keeps the knowledge base.
