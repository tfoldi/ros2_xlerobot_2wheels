"""Append-only audit log of every MCP tool call: what was called, with what arguments, why (the
calling agent's own stated reason), and what happened - so a session can be reviewed after the
fact or replayed against a live server with replay_log.py.

One JSON object per line (JSONL), gitignored like captures/trajectories - this is a runtime
artifact, not curated data. Not a substitute for a real audit/observability system if this ever
needs one; just enough to answer "what did the agent actually do, and why" after a session ends,
which is exactly what a human reviewing a robot's autonomous actions needs.
"""
import datetime
import json
import pathlib
import threading

LOG_PATH = pathlib.Path(__file__).resolve().parent.parent.parent / "logs" / "mcp_calls.jsonl"
_write_lock = threading.Lock()


def record(tool, args, reason, ok, duration_s, result=None, error=None):
    entry = {
        "time": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="milliseconds"),
        "tool": tool,
        "args": args,
        "reason": reason or "(no reason given)",
        "ok": ok,
        "duration_s": round(duration_s, 3),
    }
    if ok:
        entry["result"] = result
    else:
        entry["error"] = error
    with _write_lock:
        LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
        with open(LOG_PATH, "a") as f:
            f.write(json.dumps(entry, default=str) + "\n")
