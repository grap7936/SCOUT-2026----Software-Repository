#!/usr/bin/bash
#
# state_machine.sh — State controller for Jetson Orin Nano
#
# Implements the boot/standby/idle state tree. Each action maps to an
# executable you will define later (see EXEC_DIR below). The script handles
# state transitions, user input, and error returns.
#
# State tree:
#   Boot Up
#     -> Standby (idle, no video capture)
#          PingPID      -> back to Standby
#          TestMotor    -> back to Standby
#          TestCam      -> back to Standby
#          IdleCam      -> video capture, tracking running, not driving motor
#               YawZero       -> rotate to home, back to IdleCam
#               DrawTracking  -> draw overlays, back to IdleCam
#               DebrisTracking -> main program driving motor
#                    Error    -> return to Standby
#                    stop     -> return to IdleCam
#     -> Power Off

set -uo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

cd /home/scout/Desktop/INSTR_sh || exit

# Directory holding the executables you will define. Override with EXEC_DIR.
EXEC_DIR="${EXEC_DIR:-$(dirname "$(readlink -f "$0")")/bin}"

# Map each action to its executable. Adjust filenames here if needed.
declare -A EXECS=(
  [PingPID]="PingPID"
  [TestMotor]="TestMotor"
  [TestCam]="TestCam"
  [IdleCam]="IdleCam"
  [DebrisTracking]="main"
)

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
log()  { printf '[%s] %s\n' "$(date '+%H:%M:%S')" "$*"; }
err()  { printf '[%s] ERROR: %s\n' "$(date '+%H:%M:%S')" "$*" >&2; }

# Resolve and run an action's executable.
# Returns the executable's exit code. Non-zero is treated as an error
# by callers that care (e.g. DebrisTracking).
run_exec() {
  local action="$1"; shift
  local name="${EXECS[$action]:-}"
  if [[ -z "$name" ]]; then
    err "No executable mapped for action '$action'"
    return 127
  fi
  local path="$EXEC_DIR/$name"
  if [[ ! -x "$path" ]]; then
    err "Executable not found or not executable: $path"
    return 126
  fi
  log "Running $action ($path)"
  "$path" "$@"
  return $?
}

# ---------------------------------------------------------------------------
# Menu helper
# ---------------------------------------------------------------------------
# Reads a single choice from the user. Echoes the chosen key.
SELECTION=""
prompt() {
  local title="$1"; shift
  local -a opts=("$@")
  echo
  echo "==== $title ===="
  local i
  for ((i = 0; i < ${#opts[@]}; i++)); do
    printf '  %d) %s\n' "$((i + 1))" "${opts[$i]}"
  done
  local choice
  read -rp "Select: " choice
  # Validate numeric and in range
  if [[ "$choice" =~ ^[0-9]+$ ]] && (( choice >= 1 && choice <= ${#opts[@]} )); then
    SELECTION="${opts[$((choice - 1))]}"
  else
    SELECTION="__invalid__"
  fi
}

# ---------------------------------------------------------------------------
# States
# ---------------------------------------------------------------------------

state_pingpid() {
  # Ping Arduino to test serial connection
  log "PingPID active — pinging arduino."
  run_exec PingPID

  log "Leaving PingPID — return to Standby."
  return 0
}

state_testmotor() {
  # Send commands to Arduino to test motor positioning.
  log "TestMotor active — driving motor."
  run_exec TestMotor

  log "Leaving TestMotor — return to Standby."
  return 0
}

state_testcam() {
  # Take test photo with camera.
  log "TestCam active — taking sample iamge."
  run_exec TestCam

  log "Leaving TestCam — return to Standby."
  return 0
}

state_idlecam() {
  # Start video capture / tracking (not driving motor)
  log "IdleCam active — tracking running, motor idle."
  run_exec IdleCam

  log "Leaving IdleCam — return to Standby."
  return 0
}

state_debristracking() {
  # Start video capture / tracking (driving motor)
  log "DebrisTracking active — tracking running, motor running."
  run_exec DebrisTracking

  log "Leaving DebrisTracking — return to Standby."
  return 0
}

state_standby() {
  while true; do
    local sel
    prompt "Standby (idle, no video capture)" \
      "PingPID" \
      "TestMotor" \
      "TestCam" \
      "IdleCam" \
      "DebrisTracking" \
      "Power Off"
    case "$SELECTION" in
      PingPID)
        state_pingpid
        log "Return to Standby."
        ;;
      TestMotor)
        state_testmotor
        log "Return to Standby."
        ;;
      TestCam)
        state_testcam
        log "Return to Standby."
        ;;
      IdleCam)
        state_idlecam
        log "Return to Standby."
        ;;
      DebrisTracking)
        state_debristracking
        log "Return to Standby."
        ;;
      "Power Off")
        log "Powering off."
        return 0
        ;;
      *)
        err "Invalid selection."
        ;;
    esac
  done
}

# ---------------------------------------------------------------------------
# Boot
# ---------------------------------------------------------------------------
main() {
  log "Boot Up — Jetson Orin Nano state machine starting."
  cd 
  log "Executable directory: $EXEC_DIR"
  mkdir -p "$EXEC_DIR"
  state_standby
  log "Shutdown complete."
}

main "$@"