#!/usr/bin/env bash
# Masks/unmasks systemd's sleep targets for the duration of an Isaac Sim
# session. Suspending while the nvidia_uvm kernel module is in the tainted
# "Fixing recursive fault but reboot is needed!" state (which
# uvm_hmm_unregister_gpu's own bug puts it in on ordinary CUDA process exit,
# not just kills) turns a recoverable oops into a hard hang needing a manual
# power-cycle. Masking sleep.target and friends blocks suspend at the
# systemd-logind level regardless of lid-close, idle timeout, or the desktop
# environment's own power settings, since all of those still route through
# these targets.
#
#   tools/isaac_sim_sleep_guard.sh disable   # before starting Isaac Sim
#   tools/isaac_sim_sleep_guard.sh enable    # after you're done
#
# Needs sudo; run it yourself rather than through anything that can't prompt
# for a password.

set -euo pipefail

TARGETS=(sleep.target suspend.target hibernate.target hybrid-sleep.target)

usage() {
    echo "usage: $0 {disable|enable|status}" >&2
    exit 1
}

case "${1:-}" in
    disable)
        sudo systemctl mask "${TARGETS[@]}"
        echo "Sleep/suspend disabled. Run '$0 enable' when done with Isaac Sim."
        ;;
    enable)
        sudo systemctl unmask "${TARGETS[@]}"
        echo "Sleep/suspend re-enabled."
        ;;
    status)
        systemctl is-enabled "${TARGETS[@]}"
        ;;
    *)
        usage
        ;;
esac
