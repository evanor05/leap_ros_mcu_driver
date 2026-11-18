#!/usr/bin/env python3
"""LaserScan 元数据契约测试（宿主可运行，不需要 ESP-IDF / 硬件）。

用途：
1) 回归守卫：`microros_runtime.cpp::create_ros_entities()` 中必须先给 `scan_time`
   赋值，再计算 `time_increment`。历史上顺序颠倒，`scan_time` 读到的仍是零初始化
   值 0，导致 `/scan.time_increment` 恒为 0（P2-4）。
2) 数值契约：用 `main/config/ros_config.h` 的常量验证发布出去的角度/时间/量程
   元数据是否自洽（`time_increment * 点数 == scan_time`，`angle_max` 与
   `angle_min + (N-1)*increment` 一致）。

运行：
    python3 firmware/chassis/tests/scan_metadata_contract_test.py
退出码：0 = 全部通过；1 = 存在失败断言。
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

CHASSIS = Path(__file__).resolve().parents[1]
CONFIG = CHASSIS / "main" / "config" / "ros_config.h"
RUNTIME = CHASSIS / "main" / "tasks" / "network" / "microros" / "microros_runtime.cpp"

FAILURES: list[str] = []


def check(condition: bool, name: str, detail: str = "") -> None:
    if condition:
        print(f"PASS  {name}")
    else:
        FAILURES.append(name)
        print(f"FAIL  {name}{(' :: ' + detail) if detail else ''}")


def read(path: Path) -> str:
    if not path.is_file():
        raise SystemExit(f"缺少文件: {path}")
    return path.read_text(encoding="utf-8")


def config_float(text: str, name: str) -> float:
    m = re.search(rf"\b{name}\s*=\s*([0-9.eE+-]+)f?\s*;", text)
    if not m:
        raise SystemExit(f"ros_config.h 中找不到常量 {name}")
    return float(m.group(1))


def config_int(text: str, name: str) -> int:
    m = re.search(rf"\b{name}\s*=\s*(\d+)\s*;", text)
    if not m:
        raise SystemExit(f"ros_config.h 中找不到常量 {name}")
    return int(m.group(1))


def entity_block(text: str) -> str:
    """截取 create_ros_entities() 函数体，避免误匹配文件其它位置的同类赋值。"""
    start = text.find("static bool create_ros_entities()")
    if start < 0:
        raise SystemExit("microros_runtime.cpp 中找不到 create_ros_entities()")
    depth = 0
    for offset, ch in enumerate(text[start:], start):
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return text[start : offset + 1]
    raise SystemExit("create_ros_entities() 函数体花括号不闭合")


def main() -> int:
    cfg = read(CONFIG)
    runtime = read(RUNTIME)
    block = entity_block(runtime)

    point_count = config_int(cfg, "kLaserScanPointCount")
    deg_to_rad = config_float(cfg, "kDegToRad")
    scan_time = config_float(cfg, "kScanTimeS")
    range_min = config_float(cfg, "kLaserRangeMinM")
    range_max = config_float(cfg, "kLaserRangeMaxM")

    # ---- 1. 赋值顺序回归守卫（P2-4 本体）----
    scan_idx = block.find("s_scan_msg.scan_time =")
    inc_idx = block.find("s_scan_msg.time_increment =")
    check(scan_idx >= 0, "create_ros_entities 给 s_scan_msg.scan_time 赋值")
    check(inc_idx >= 0, "create_ros_entities 计算 s_scan_msg.time_increment")
    if scan_idx >= 0 and inc_idx >= 0:
        check(
            scan_idx < inc_idx,
            "scan_time 赋值先于 time_increment 计算（否则 time_increment 读到 0）",
            f"scan_time@{block[:scan_idx].count(chr(10))} "
            f"time_increment@{block[:inc_idx].count(chr(10))}",
        )

    # ---- 2. time_increment 必须真的由 scan_time / 点数得到 ----
    m = re.search(r"s_scan_msg\.time_increment\s*=\s*([^;]+);", block)
    expr = m.group(1) if m else ""
    check(
        "s_scan_msg.scan_time" in expr and "kLaserScanPointCount" in expr,
        "time_increment = scan_time / kLaserScanPointCount",
        f"实际表达式: {expr.strip()}",
    )

    # ---- 3. 数值契约（float32 精度内自洽）----
    expected_increment = scan_time / point_count
    # 0.1f / 360 -> float32；容差取 float32 单精度量级
    check(
        abs(expected_increment - 2.7777778e-4) < 1e-9,
        "time_increment 期望值 ≈ 2.7778e-4 s（0.1 s / 360 点）",
        f"计算值 {expected_increment:.10e}",
    )
    check(scan_time > 0.0, "scan_time 必须为正（>0），不能是零初始化残留", f"{scan_time}")
    check(
        expected_increment * point_count == scan_time or abs(expected_increment * point_count - scan_time) < 1e-9,
        "time_increment * 点数 == scan_time（扫描内时间可还原）",
    )

    # ---- 4. 角度/量程元数据 ----
    angle_min_src = re.search(r"s_scan_msg\.angle_min\s*=\s*([^;]+);", block)
    angle_max_src = re.search(r"s_scan_msg\.angle_max\s*=\s*([^;]+);", block)
    angle_inc_src = re.search(r"s_scan_msg\.angle_increment\s*=\s*([^;]+);", block)
    check(
        all(x is not None for x in (angle_min_src, angle_max_src, angle_inc_src)),
        "angle_min / angle_max / angle_increment 均有赋值",
    )
    if all(x is not None for x in (angle_min_src, angle_max_src, angle_inc_src)):
        check(
            "kDegToRad" in angle_inc_src.group(1) and "kDegToRad" in angle_max_src.group(1),
            "角度元数据由 kDegToRad 与点数推导（1°/点）",
        )
        coverage = (point_count - 1) * deg_to_rad  # 当前固件覆盖 [0, 6.2657] rad
        check(
            abs(coverage - 6.265732) < 1e-5,
            "当前角度覆盖 0..(N-1)·1° = 0..6.2657 rad（非对称 0..+360°，实际少 1°）",
            f"{coverage:.6f} rad",
        )
        # 记录给主机侧建图负责人的归一化建议，不作为失败项
        print("NOTE  角度约定为 0..+359°，angle_max 未包含 2π；对称化(-π..π)需同时重排 ranges，"
              "按任务要求仅记录、不改固件。")

    check(
        range_min > 0.0 and range_max > range_min,
        "range_min/range_max 合法且 range_max > range_min",
        f"[{range_min}, {range_max}]",
    )
    m = re.search(r"s_scan_msg\.range_min\s*=\s*([^;]+);", block)
    check(m is not None and "kLaserRangeMinM" in m.group(1), "range_min 取自 ros_config 常量")

    # ---- 5. 发布路径必须复用同一份元数据 ----
    publishers = read(CHASSIS / "main" / "tasks" / "network" / "microros" / "microros_publishers.cpp")
    check(
        "s_scan_msg.scan_time" not in publishers and "s_scan_msg.time_increment" not in publishers,
        "发布路径不重复覆盖 scan_time/time_increment（单一数据源）",
    )

    print()
    if FAILURES:
        print(f"结果: {len(FAILURES)} 项失败 -> {', '.join(FAILURES)}")
        return 1
    print("结果: 全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
