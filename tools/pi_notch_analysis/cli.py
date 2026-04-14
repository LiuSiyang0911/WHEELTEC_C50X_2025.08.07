from __future__ import annotations

import argparse
from pathlib import Path

from .csv_loader import load_debug_csv
from .notch_sim import SimulationConfig, simulate_closed_loop_notch
from .report import render_text_report, write_json_report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Offline PI notch recommendation from AKM debug CSV data")
    parser.add_argument("csv_path", type=Path, help="Path to the debug CSV file")
    parser.add_argument("--motor", choices=["a", "b"], default="a", help="Motor channel to analyze")
    parser.add_argument("--wheel-diameter", type=float, default=0.125, help="Wheel diameter in meters")
    parser.add_argument("--kp", type=float, default=300.0, help="PI proportional gain")
    parser.add_argument("--ki", type=float, default=300.0, help="PI integral gain")
    parser.add_argument("--kd", type=float, default=0.0, help="PI derivative gain")
    parser.add_argument(
        "--gains",
        type=float,
        nargs="*",
        default=[0.01, 0.015, 0.02, 0.025, 0.03, 0.04, 0.05, 0.08],
        help="Candidate notch LPF gains to sweep",
    )
    parser.add_argument("--settle-time", type=float, default=5.0, help="Seconds to skip after active segment starts")
    parser.add_argument("--tail-time", type=float, default=5.0, help="Seconds to skip before active segment ends")
    parser.add_argument("--json-out", type=Path, help="Optional path to write JSON report")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    data = load_debug_csv(args.csv_path, settle_time_s=args.settle_time, tail_time_s=args.tail_time)
    report = simulate_closed_loop_notch(
        data=data,
        motor=args.motor,
        config=SimulationConfig(
            wheel_circ_m=args.wheel_diameter * 3.141592653589793,
            gains=list(args.gains),
            kp=args.kp,
            ki=args.ki,
            kd=args.kd,
        ),
    )
    print(render_text_report(report))
    if args.json_out:
        write_json_report(report, args.json_out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
