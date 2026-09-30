#!/usr/bin/env python3
"""Replay a captured logs/mcp_calls.jsonl against a live MCP server, in order.

  python3 tools/mcp_server/replay_log.py logs/mcp_calls.jsonl
  python3 tools/mcp_server/replay_log.py logs/mcp_calls.jsonl --dry-run
  python3 tools/mcp_server/replay_log.py logs/mcp_calls.jsonl --limit 5

Opens one connection to run.sh (same as .mcp.json) and calls tools through it, so every real
safety gate (presence, estop) applies exactly as it did originally - this replays commands, not
their prior outcomes, so a run that succeeded live but is replayed with no presence held will be
refused now, correctly. --dry-run prints what would be called without connecting to anything.
"""
import argparse
import asyncio
import json
import pathlib
import sys

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

RUN_SH = pathlib.Path(__file__).resolve().parent / "run.sh"


def load_entries(path):
    entries = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                entries.append(json.loads(line))
    return entries


async def replay(entries):
    params = StdioServerParameters(command=str(RUN_SH), args=[])
    ok = failed = 0
    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            for i, entry in enumerate(entries, 1):
                tool, args, reason = entry["tool"], entry["args"], entry.get("reason", "")
                print(f"[{i}/{len(entries)}] {tool}({args}) - {reason}")
                result = await session.call_tool(tool, {**args, "reason": f"[replay] {reason}"})
                text = result.content[0].text if result.content else None
                status = "FAILED" if result.is_error else "ok"
                failed += result.is_error
                ok += not result.is_error
                print(f"    {status}: {text[:150] if text else text}")
    print(f"\n{ok} ok, {failed} failed, {len(entries)} total")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log_file", type=pathlib.Path)
    parser.add_argument("--dry-run", action="store_true", help="print the calls, don't make them")
    parser.add_argument("--limit", type=int, default=None, help="replay only the first N entries")
    args = parser.parse_args()

    entries = load_entries(args.log_file)
    if args.limit:
        entries = entries[:args.limit]
    if not entries:
        sys.exit(f"no entries in {args.log_file}")

    if args.dry_run:
        for i, entry in enumerate(entries, 1):
            print(f"[{i}/{len(entries)}] {entry['tool']}({entry['args']}) - {entry.get('reason', '')}")
        return

    asyncio.run(replay(entries))


if __name__ == "__main__":
    main()
