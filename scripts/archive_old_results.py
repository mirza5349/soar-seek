#!/usr/bin/env python3
"""Archive (default) or delete previously generated results before a new
results package is produced.

Archiving moves everything to archive/results_YYYYMMDD_HHMMSS/.
Deletion requires --delete AND interactive confirmation (type YES).
"""
import argparse
import os
import shutil
import sys
import time

WORKSPACE = "/home/px4_sitl/sim_paper"

CANDIDATES = [
    "results/csv",
    "results/figures",
    "results/tables",
    "results/pdf",
    "results/quality",
]
GLOB_PDFS = ["soar_seek_simulation_results.pdf", "results_report.pdf",
             "full_system_results.pdf"]


def collect_targets():
    targets = []
    res = os.path.join(WORKSPACE, "results")
    for rel in CANDIDATES:
        p = os.path.join(WORKSPACE, rel)
        if os.path.isdir(p) and os.listdir(p):
            targets.append(p)
    if os.path.isdir(res):
        for name in os.listdir(res):
            p = os.path.join(res, name)
            if name in GLOB_PDFS and os.path.isfile(p):
                targets.append(p)
            # old campaign run directories living directly under results/
            if os.path.isdir(p) and name not in ("csv", "figures", "tables",
                                                 "pdf", "quality", "archive"):
                targets.append(p)
    return targets


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--delete", action="store_true",
                    help="Delete permanently instead of archiving")
    ap.add_argument("--yes", action="store_true",
                    help="Skip interactive confirmation (non-interactive use)")
    args = ap.parse_args()

    targets = collect_targets()
    if not targets:
        print("No previous results found. Nothing to do.")
        return

    print("The following previous results will be "
          + ("DELETED PERMANENTLY" if args.delete else "archived") + ":")
    for t in targets:
        print(f"  {t}")

    if not args.yes:
        ans = input("Do you want me to delete/archive the previous results before "
                    "generating the new results PDF? Type YES to continue: ")
        if ans.strip() != "YES":
            print("Aborted. No files were touched.")
            sys.exit(1)

    if args.delete:
        for t in targets:
            if os.path.isdir(t):
                shutil.rmtree(t)
            else:
                os.remove(t)
        print("Previous results deleted.")
    else:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        archive_dir = os.path.join(WORKSPACE, f"archive/results_{stamp}")
        os.makedirs(archive_dir, exist_ok=True)
        for t in targets:
            shutil.move(t, os.path.join(archive_dir, os.path.basename(t)))
        print(f"Previous results archived to {archive_dir}")

    # Recreate the clean output skeleton
    for rel in ["results/csv", "results/figures", "results/pdf", "results/quality"]:
        os.makedirs(os.path.join(WORKSPACE, rel), exist_ok=True)
    print("Clean results/ skeleton recreated.")


if __name__ == "__main__":
    main()
