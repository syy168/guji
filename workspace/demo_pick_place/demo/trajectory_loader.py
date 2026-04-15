from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import List


def load_joint_trajectory_deg(path: Path) -> List[List[float]]:
    """加载轨迹文件并返回关节角列表（单位：度）。

    支持格式：
    - CSV: 包含 joint_1~joint_6 列
    - JSON Lines/TXT: 每行一个 JSON，格式 {"point": [编码值...]}，按 /1000 转为度
    """
    if not path.exists():
        raise FileNotFoundError(f"轨迹文件不存在: {path}")

    suffix = path.suffix.lower()
    if suffix in {".json", ".jsonl", ".txt"}:
        return _load_json_lines(path)
    return _load_csv(path)


def _load_csv(path: Path) -> List[List[float]]:
    points: List[List[float]] = []
    with path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            point = [float(row.get(f"joint_{i}", 0.0)) for i in range(1, 7)]
            points.append(point)
    return points


def _load_json_lines(path: Path) -> List[List[float]]:
    points: List[List[float]] = []
    with path.open("r", encoding="utf-8") as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                data = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"轨迹文件 JSON 解析失败: {path}, 行 {line_num}: {exc}") from exc

            raw = data.get("point")
            if not isinstance(raw, list) or len(raw) < 6:
                raise ValueError(f"轨迹格式错误: {path}, 行 {line_num} 缺少 point[0..5]")

            points.append([float(v) / 1000.0 for v in raw[:6]])
    return points


def sample_segment_midpoints(points: List[List[float]], segment_count: int) -> List[List[float]]:
    """将轨迹分段后抽取每段中点。"""
    if not points:
        raise ValueError("轨迹为空，无法分段采样")

    total = len(points)
    seg = max(1, min(segment_count, total))
    picked = []
    last_idx = -1

    for i in range(seg):
        start = int(i * total / seg)
        end = int((i + 1) * total / seg) - 1
        end = max(start, end)
        mid = (start + end) // 2

        if mid <= last_idx:
            mid = min(total - 1, last_idx + 1)
        picked.append(points[mid])
        last_idx = mid

    return picked
