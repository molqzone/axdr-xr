#!/usr/bin/env python3
"""Build, flash, collect RTT telemetry, and verify dumb FOC duty stability."""

from __future__ import annotations

import argparse
import json
import math
import re
import signal
import socket
import statistics
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RTTD = Path("/home/keruth/Projects/ratitude/target/debug/rttd")
DEFAULT_JSONL = Path("/tmp/axdr_rat_bridge2.jsonl")
DEFAULT_OPENOCD_LOG = Path("/tmp/axdr_openocd_rtt.log")
DEFAULT_RTTD_LOG = Path("/tmp/axdr_rttd.log")


def run_checked(cmd: list[str], cwd: Path) -> None:
    print(f"$ {' '.join(cmd)}")
    subprocess.run(cmd, cwd=cwd, check=True)


def resolve_data_lma_from_elf(elf_path: Path) -> str:
    output = subprocess.check_output(
        ["arm-none-eabi-readelf", "-l", str(elf_path)],
        text=True,
        encoding="utf-8",
    )
    load_pattern = re.compile(
        r"^\s*LOAD\s+0x([0-9a-fA-F]+)\s+0x([0-9a-fA-F]+)\s+0x([0-9a-fA-F]+)\s+"
        r"0x([0-9a-fA-F]+)\s+0x([0-9a-fA-F]+)"
    )
    for line in output.splitlines():
        match = load_pattern.match(line)
        if not match:
            continue
        virt = int(match.group(2), 16)
        phys = int(match.group(3), 16)
        file_size = int(match.group(4), 16)
        mem_size = int(match.group(5), 16)
        if (
            0x20000000 <= virt < 0x30000000
            and 0x08000000 <= phys < 0x09000000
            and file_size > 0
            and mem_size > 0
        ):
            return f"0x{phys:08x}"
    raise RuntimeError(f"failed to resolve .data LMA from ELF program headers: {elf_path}")


def wait_for_tcp_port(host: str, port: int, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(0.2)
            try:
                sock.connect((host, port))
                return
            except OSError:
                time.sleep(0.1)
    raise TimeoutError(f"timeout waiting for {host}:{port}")


def send_openocd_telnet_command(host: str, port: int, command: str) -> None:
    with socket.create_connection((host, port), timeout=2.0) as sock:
        sock.settimeout(0.5)
        try:
            sock.recv(4096)
        except OSError:
            pass
        sock.sendall((command.strip() + "\n").encode("utf-8"))
        time.sleep(0.1)
        sock.sendall(b"exit\n")


def terminate_process(proc: subprocess.Popen[Any] | None, name: str) -> None:
    if proc is None or proc.poll() is not None:
        return

    try:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=3.0)
        return
    except Exception:
        pass

    try:
        proc.terminate()
        proc.wait(timeout=3.0)
        return
    except Exception:
        pass

    proc.kill()
    proc.wait(timeout=2.0)
    print(f"warning: force-killed {name}", file=sys.stderr)


def parse_samples(jsonl_path: Path, packet_id: str) -> list[dict[str, float]]:
    samples: list[dict[str, float]] = []
    with jsonl_path.open("r", encoding="utf-8") as f:
        for line in f:
            raw = line.strip()
            if not raw:
                continue
            try:
                item = json.loads(raw)
            except json.JSONDecodeError:
                continue

            if str(item.get("id", "")).lower() != packet_id.lower():
                continue
            data = item.get("data")
            if not isinstance(data, dict):
                continue
            required = (
                "electrical_angle",
                "iq_target",
                "duty_u",
                "duty_v",
                "duty_w",
            )
            if any(key not in data for key in required):
                continue

            try:
                samples.append(
                    {
                        "electrical_angle": float(data["electrical_angle"]),
                        "iq_target": float(data["iq_target"]),
                        "duty_u": float(data["duty_u"]),
                        "duty_v": float(data["duty_v"]),
                        "duty_w": float(data["duty_w"]),
                    }
                )
            except (TypeError, ValueError):
                continue
    return samples


def finite_and_in_range(values: list[float], lower: float, upper: float) -> bool:
    for value in values:
        if not math.isfinite(value):
            return False
        if value < lower or value > upper:
            return False
    return True


def max_consecutive_jump(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0
    jumps = [abs(values[i + 1] - values[i]) for i in range(len(values) - 1)]
    return max(jumps)


def compute_stddev(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0
    return statistics.pstdev(values)


def verify_samples(
    samples: list[dict[str, float]],
    min_samples: int,
    duty_min: float,
    duty_max: float,
    min_stddev: float,
    max_jump: float,
) -> tuple[bool, list[str]]:
    errors: list[str] = []

    if len(samples) < min_samples:
        errors.append(f"sample count too low: {len(samples)} < {min_samples}")
        return False, errors

    duty_u = [sample["duty_u"] for sample in samples]
    duty_v = [sample["duty_v"] for sample in samples]
    duty_w = [sample["duty_w"] for sample in samples]
    angles = [sample["electrical_angle"] for sample in samples]

    for name, values in (("duty_u", duty_u), ("duty_v", duty_v), ("duty_w", duty_w)):
        if not finite_and_in_range(values, duty_min, duty_max):
            errors.append(f"{name} out of range [{duty_min}, {duty_max}] or non-finite")
        if compute_stddev(values) < min_stddev:
            errors.append(f"{name} waveform too flat (stddev < {min_stddev})")
        if max_consecutive_jump(values) > max_jump:
            errors.append(f"{name} jump too large (> {max_jump})")

    two_pi = 2.0 * math.pi
    if not finite_and_in_range(angles, 0.0, two_pi + 1e-3):
        errors.append("electrical_angle out of [0, 2*pi)")

    return len(errors) == 0, errors


def build_args() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preset", default="Debug", help="CMake preset to build")
    parser.add_argument(
        "--hex",
        default="build/Debug/AxDr.elf",
        help="Firmware image path (ELF/HEX/BIN)",
    )
    parser.add_argument("--openocd-interface", default="interface/cmsis-dap.cfg")
    parser.add_argument("--openocd-target", default="target/stm32g4x.cfg")
    parser.add_argument("--rtt-port", type=int, default=19021)
    parser.add_argument("--rtt-host", default="127.0.0.1")
    parser.add_argument("--openocd-telnet-port", type=int, default=4444)
    parser.add_argument(
        "--no-reset-after-connect",
        action="store_false",
        dest="reset_after_connect",
        help="Do not issue an extra `reset run` after rttd connects",
    )
    parser.set_defaults(reset_after_connect=True)
    parser.add_argument("--rtt-ram-base", default="0x20000000")
    parser.add_argument("--rtt-ram-size", default="0x8000")
    parser.add_argument("--rttd-bin", default=str(DEFAULT_RTTD))
    parser.add_argument("--rat-config", default="rat.toml")
    parser.add_argument("--jsonl", default=str(DEFAULT_JSONL))
    parser.add_argument("--openocd-log", default=str(DEFAULT_OPENOCD_LOG))
    parser.add_argument("--rttd-log", default=str(DEFAULT_RTTD_LOG))
    parser.add_argument("--collect-seconds", type=float, default=6.0)
    parser.add_argument("--packet-id", default="0x21")
    parser.add_argument("--min-samples", type=int, default=20)
    parser.add_argument("--duty-min", type=float, default=-0.02)
    parser.add_argument("--duty-max", type=float, default=1.02)
    parser.add_argument("--min-stddev", type=float, default=0.002)
    parser.add_argument("--max-jump", type=float, default=0.25)
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--skip-flash", action="store_true")
    return parser


def main() -> int:
    parser = build_args()
    args = parser.parse_args()

    repo = REPO_ROOT
    image_path = (repo / args.hex).resolve()
    jsonl_path = Path(args.jsonl).resolve()
    openocd_log_path = Path(args.openocd_log).resolve()
    rttd_log_path = Path(args.rttd_log).resolve()
    rttd_bin = Path(args.rttd_bin).resolve()
    rat_config = (repo / args.rat_config).resolve()

    openocd_proc: subprocess.Popen[Any] | None = None
    rttd_proc: subprocess.Popen[Any] | None = None
    openocd_log = None
    rttd_log = None

    try:
        if not args.skip_build:
            build_cache = repo / "build" / args.preset / "CMakeCache.txt"
            if not build_cache.exists():
                run_checked(["cmake", "--preset", args.preset], repo)
            run_checked(["cmake", "--build", "--preset", args.preset, "-j"], repo)

        if not image_path.exists():
            raise FileNotFoundError(f"firmware image not found: {image_path}")

        flash_image = image_path
        if image_path.suffix.lower() == ".elf":
            data_lma = resolve_data_lma_from_elf(image_path)
            flash_image = Path("/tmp/axdr_verify_flash.hex")
            run_checked(
                [
                    "arm-none-eabi-objcopy",
                    "-O",
                    "ihex",
                    "--remove-section",
                    ".got",
                    "--remove-section",
                    ".bss",
                    "--remove-section",
                    "._user_heap_stack",
                    "--remove-section",
                    ".tbss",
                    "--change-section-lma",
                    f".data={data_lma}",
                    str(image_path),
                    str(flash_image),
                ],
                repo,
            )

        if not args.skip_flash:
            flash_cmd = [
                "openocd",
                "-f",
                args.openocd_interface,
                "-f",
                args.openocd_target,
                "-c",
                f"init; reset halt; program {flash_image} verify; reset run; shutdown",
            ]
            run_checked(flash_cmd, repo)

        if jsonl_path.exists():
            jsonl_path.unlink()

        openocd_log = openocd_log_path.open("w", encoding="utf-8")
        openocd_cmd = [
            "openocd",
            "-f",
            args.openocd_interface,
            "-f",
            args.openocd_target,
            "-c",
            (
                f"init; reset run; rtt setup {args.rtt_ram_base} {args.rtt_ram_size} "
                f"\"SEGGER RTT\"; rtt start; rtt polling_interval 1; "
                f"rtt server start {args.rtt_port} 0"
            ),
        ]
        print(f"$ {' '.join(openocd_cmd)}")
        openocd_proc = subprocess.Popen(
            openocd_cmd,
            cwd=repo,
            stdout=openocd_log,
            stderr=subprocess.STDOUT,
            text=True,
        )

        wait_for_tcp_port(args.rtt_host, args.rtt_port, timeout_s=8.0)

        if not rttd_bin.exists():
            raise FileNotFoundError(f"rttd binary not found: {rttd_bin}")
        if not rat_config.exists():
            raise FileNotFoundError(f"rat config not found: {rat_config}")

        rttd_log = rttd_log_path.open("w", encoding="utf-8")
        rttd_cmd = [str(rttd_bin), "--config", str(rat_config)]
        print(f"$ {' '.join(rttd_cmd)}")
        rttd_proc = subprocess.Popen(
            rttd_cmd,
            cwd=repo,
            stdin=subprocess.PIPE,
            stdout=rttd_log,
            stderr=subprocess.STDOUT,
            text=True,
        )

        if args.reset_after_connect:
            # Re-trigger boot-time schema emission when explicitly requested.
            send_openocd_telnet_command(args.rtt_host, args.openocd_telnet_port, "reset run")

        deadline = time.monotonic() + args.collect_seconds
        while time.monotonic() < deadline:
            if rttd_proc.poll() is not None:
                raise RuntimeError("rttd exited unexpectedly, check rttd log")
            if openocd_proc.poll() is not None:
                raise RuntimeError("openocd RTT server exited unexpectedly, check openocd log")
            time.sleep(0.1)

    finally:
        terminate_process(rttd_proc, "rttd")
        terminate_process(openocd_proc, "openocd")
        if openocd_log is not None:
            openocd_log.close()
        if rttd_log is not None:
            rttd_log.close()

    if not jsonl_path.exists():
        print(f"FAIL: jsonl output not found: {jsonl_path}")
        print(f"openocd log: {openocd_log_path}")
        print(f"rttd log: {rttd_log_path}")
        return 2

    samples = parse_samples(jsonl_path, args.packet_id)
    ok, errors = verify_samples(
        samples=samples,
        min_samples=args.min_samples,
        duty_min=args.duty_min,
        duty_max=args.duty_max,
        min_stddev=args.min_stddev,
        max_jump=args.max_jump,
    )

    if samples:
        duty_u = [sample["duty_u"] for sample in samples]
        duty_v = [sample["duty_v"] for sample in samples]
        duty_w = [sample["duty_w"] for sample in samples]
        print(f"samples={len(samples)} packet_id={args.packet_id}")
        print(
            "duty_u: min={:.5f} max={:.5f} std={:.5f}".format(
                min(duty_u), max(duty_u), compute_stddev(duty_u)
            )
        )
        print(
            "duty_v: min={:.5f} max={:.5f} std={:.5f}".format(
                min(duty_v), max(duty_v), compute_stddev(duty_v)
            )
        )
        print(
            "duty_w: min={:.5f} max={:.5f} std={:.5f}".format(
                min(duty_w), max(duty_w), compute_stddev(duty_w)
            )
        )
        print(
            "jump_max: u={:.5f} v={:.5f} w={:.5f}".format(
                max_consecutive_jump(duty_u),
                max_consecutive_jump(duty_v),
                max_consecutive_jump(duty_w),
            )
        )

    if ok:
        print("PASS: dumb FOC duty waveform looks stable")
        return 0

    print("FAIL: dumb FOC duty verification failed")
    for error in errors:
        print(f"- {error}")
    print(f"openocd log: {openocd_log_path}")
    print(f"rttd log: {rttd_log_path}")
    return 2


if __name__ == "__main__":
    sys.exit(main())
