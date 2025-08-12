from __future__ import annotations

import sys

from myterm.shell import MyTerm
import argparse


def main() -> int:
    parser = argparse.ArgumentParser(prog="myterm", add_help=True)
    parser.add_argument("-c", "--command", help="Run a single command and exit")
    args = parser.parse_args()

    shell = MyTerm()
    if args.command:
        return shell.execute_line(args.command)
    return shell.run()


if __name__ == "__main__":
    raise SystemExit(main())
