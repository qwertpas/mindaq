#!/usr/bin/env python3.11
import argparse
import itertools
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


AXES = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
FORCE = 1.0
TORQUE = 0.01
XML_PATH = Path("calibration/FT8978 Net.xml")
FT8978_FW_COLS = np.array([4, 5, 2, 3, 0, 1])
FT8978_FW_SIGNS = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
FT8978_FW_SCALES = np.array(
    [0.934640523, 0.939244663, 1.036231884, 1.041894353, 1.184265010, 0.912280702]
)
FT8978_REORDERED_COLS = np.array([2, 3, 4, 5, 0, 1])
FT8978_REORDERED_SIGNS = np.array([-1.0, 1.0, 1.0, -1.0, 1.0, 1.0])

FT_SCALE = np.array(
    [
        5.242261206577384e-4,
        6.711481094720999e-4,
        1.491585028973418e-3,
        3.382662814280309e-6,
        3.926013399015233e-6,
        5.998476015074367e-6,
    ]
)


def child_text(parent: ET.Element, name: str) -> str | None:
    for child in parent:
        if child.tag.rsplit("}", 1)[-1] == name:
            return child.text
    return None


def child(parent: ET.Element, name: str) -> ET.Element | None:
    for item in parent.iter():
        if item.tag.rsplit("}", 1)[-1] == name:
            return item
    return None


def xml_info(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    root = ET.parse(path).getroot()
    info = child(root, "tblNetFTCalibrationInfo")
    if info is None:
        raise ValueError(f"missing tblNetFTCalibrationInfo in {path}")

    names = ("MatrixFX", "MatrixFy", "MatrixFz", "MatrixTx", "MatrixTy", "MatrixTz")
    rows = []
    for index, name in enumerate(names):
        text = child_text(info, name)
        if text is None:
            raise ValueError(f"missing {name} in {path}")
        row = np.fromstring(text, sep=" ", dtype=float)
        if row.size != 6:
            raise ValueError(f"{name} has {row.size} values, expected 6")
        # ATI Net/OEM matrix output is FT counts; normalize to N/count and Nm/count.
        row /= 1e6
        if index >= 3:
            row /= 1000.0
        rows.append(row)

    gains = vector_field(info, "GaugeGains")
    offsets = vector_field(info, "GaugeOffsets")
    return np.vstack(rows), gains, offsets


def xml_matrix(path: Path) -> np.ndarray:
    matrix, _, _ = xml_info(path)
    return matrix


def vector_field(info: ET.Element, name: str) -> np.ndarray:
    text = child_text(info, name)
    if text is None:
        raise ValueError(f"missing {name}")
    values = np.fromstring(text, sep=" ", dtype=float)
    if values.size != 6:
        raise ValueError(f"{name} has {values.size} values, expected 6")
    return values


def resolve_matrix() -> np.ndarray:
    s = FT_SCALE
    matrix = np.zeros((6, 6), dtype=float)
    matrix[0] = [0.0, 0.5 * s[0], 0.0, -0.5 * s[0], 0.0, 0.0]
    matrix[1] = [0.0, -0.25 * s[1], 0.0, -0.25 * s[1], 0.0, 0.5 * s[1]]
    matrix[2] = [0.25 * s[2], 0.0, 0.25 * s[2], 0.0, 0.5 * s[2], 0.0]
    matrix[3] = [-0.5 * s[3], 0.0, 0.5 * s[3], 0.0, 0.0, 0.0]
    matrix[4] = [0.25 * s[4], 0.0, 0.25 * s[4], 0.0, -0.5 * s[4], 0.0]
    matrix[5] = [0.0, 0.25 * s[5], 0.0, 0.25 * s[5], 0.0, 0.5 * s[5]]
    return matrix


def firmware_matrix(xml: np.ndarray) -> np.ndarray:
    return (xml * FT8978_FW_SIGNS[:, None])[:, FT8978_FW_COLS] * FT8978_FW_SCALES[None, :]


def reordered_matrix(xml: np.ndarray) -> np.ndarray:
    return (xml * FT8978_REORDERED_SIGNS[:, None])[:, FT8978_REORDERED_COLS]


def convention_matrix(name: str, xml: np.ndarray) -> np.ndarray:
    if name == "resolve":
        return resolve_matrix()
    if name == "firmware":
        return firmware_matrix(xml)
    if name == "bad_reordered":
        return reordered_matrix(xml)
    if name == "ft8978_xml_raw":
        return xml
    raise ValueError(f"unknown mapping {name}")


def unit_rows(matrix: np.ndarray) -> np.ndarray:
    return matrix / np.linalg.norm(matrix, axis=1, keepdims=True)


def direction_report(reference: np.ndarray, test: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    dots = unit_rows(reference) @ unit_rows(test).T
    match = np.argmax(np.abs(dots), axis=1)
    signs = np.sign(dots[np.arange(6), match])
    scores = np.abs(dots[np.arange(6), match])
    return match, signs, scores


def print_direction_report(reference_name: str, test_name: str, reference: np.ndarray, test: np.ndarray) -> None:
    match, signs, scores = direction_report(reference, test)
    print(f"\nBest row direction match: {reference_name} <- {test_name}")
    for index, axis in enumerate(AXES):
        sign = "+" if signs[index] >= 0 else "-"
        print(f"{axis:>3} <= {sign}{AXES[match[index]]:<2}  cosine={scores[index]:.6f}")


def convention_label(rows: tuple[int, ...], row_signs: tuple[int, ...], cols: tuple[int, ...], col_signs: tuple[int, ...]) -> str:
    row_text = ",".join(f"{'-' if s < 0 else '+'}{AXES[r]}" for r, s in zip(rows, row_signs))
    col_text = ",".join(f"{'-' if s < 0 else '+'}g{c}" for c, s in zip(cols, col_signs))
    return f"rows=[{row_text}] cols=[{col_text}]"


def convention_from_parts(
    matrix: np.ndarray,
    rows: tuple[int, ...],
    row_signs: tuple[int, ...],
    cols: tuple[int, ...],
    col_signs: tuple[int, ...],
) -> np.ndarray:
    out = matrix[np.array(rows)][:, np.array(cols)].copy()
    out *= np.array(row_signs, dtype=float)[:, None]
    out *= np.array(col_signs, dtype=float)[None, :]
    return out


def search_conventions(xml: np.ndarray, limit: int, force_axis_types: bool) -> list[tuple[float, float, str, np.ndarray]]:
    ref = unit_rows(resolve_matrix())
    if force_axis_types:
        row_perms = [
            force + torque
            for force in itertools.permutations((0, 1, 2))
            for torque in itertools.permutations((3, 4, 5))
        ]
    else:
        row_perms = list(itertools.permutations(range(6)))
    row_perms_array = np.array(row_perms)
    row_index = np.arange(6)
    col_perms = list(itertools.permutations(range(6)))
    col_signs_set = list(itertools.product((-1, 1), repeat=6))
    best: list[tuple[float, float, str, np.ndarray]] = []

    for cols in col_perms:
        ordered = xml[:, np.array(cols)]
        for col_signs in col_signs_set:
            col_matrix = ordered * np.array(col_signs, dtype=float)[None, :]
            unit_col = unit_rows(col_matrix)
            dots = ref @ unit_col.T
            row_dots_all = dots[row_index[None, :], row_perms_array]
            scores_all = np.abs(row_dots_all)
            means = np.mean(scores_all, axis=1)
            worsts = np.min(scores_all, axis=1)
            cutoff = best[-1][0] if len(best) == limit else -np.inf
            keep = np.flatnonzero(means > cutoff)
            if keep.size > limit:
                keep = keep[np.argpartition(means[keep], -limit)[-limit:]]
            for row_perm_index in keep:
                rows = tuple(int(value) for value in row_perms_array[row_perm_index])
                row_dots = row_dots_all[row_perm_index]
                scores = np.abs(row_dots)
                row_signs = tuple(1 if value >= 0.0 else -1 for value in row_dots)
                mean = float(means[row_perm_index])
                worst = float(worsts[row_perm_index])
                if len(best) < limit or mean > best[-1][0]:
                    matrix = convention_from_parts(xml, rows, row_signs, cols, col_signs)
                    label = convention_label(rows, row_signs, cols, col_signs)
                    best.append((mean, worst, label, matrix))
                    best.sort(key=lambda item: (item[0], item[1]), reverse=True)
                    del best[limit:]
    return best


def pure_targets(force: float, torque: float) -> np.ndarray:
    out = np.zeros((6, 6), dtype=float)
    for index in range(6):
        out[index, index] = force if index < 3 else torque
    return out


def format_row(values: np.ndarray) -> str:
    return " ".join(f"{value:>11.4g}" for value in values)


def print_matrix(name: str, matrix: np.ndarray) -> None:
    print(f"\n{name} matrix, gages uV -> [Fx Fy Fz Tx Ty Tz]")
    print("              " + " ".join(f"{axis:>11}" for axis in AXES))
    for axis, row in zip(AXES, matrix):
        print(f"{axis:>3} {format_row(row)}")


def print_compare(title: str, source: np.ndarray, other: np.ndarray, targets: np.ndarray) -> None:
    inv_source = np.linalg.inv(source)
    print(f"\n{title}")
    print("pure    source out                              other out")
    for axis, target in zip(AXES, targets):
        gages = inv_source @ target
        source_out = source @ gages
        other_out = other @ gages
        print(f"{axis:>3}   {format_row(source_out)}   {format_row(other_out)}")


def print_offset_report(matrix_name: str, matrix: np.ndarray, offsets: np.ndarray) -> None:
    offset_out = matrix @ offsets
    print(f"\n{matrix_name} output from XML GaugeOffsets if not removed")
    print("        " + " ".join(f"{axis:>11}" for axis in AXES))
    print(f"bias   {format_row(offset_out)}")
    print("Subtracting the same offset vector before matrix multiplication makes this row zero.")


def print_gages(title: str, matrix: np.ndarray, targets: np.ndarray) -> None:
    inv_matrix = np.linalg.inv(matrix)
    print(f"\n{title}")
    print("pure    g0          g1          g2          g3          g4          g5")
    for axis, target in zip(AXES, targets):
        gages = inv_matrix @ target
        print(f"{axis:>3}   {format_row(gages)}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--xml", type=Path, default=XML_PATH)
    parser.add_argument("--a", default="resolve")
    parser.add_argument("--b", default="firmware")
    parser.add_argument("--force", type=float, default=FORCE)
    parser.add_argument("--torque", type=float, default=TORQUE)
    parser.add_argument("--matrices", action="store_true")
    parser.add_argument("--offsets", action="store_true")
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--search", action="store_true")
    parser.add_argument("--search-limit", type=int, default=10)
    parser.add_argument("--all-axis-perms", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    names = (
        "resolve",
        "firmware",
        "bad_reordered",
        "ft8978_xml_raw",
    )
    if args.list:
        print("\n".join(names))
        return

    xml, gains, offsets = xml_info(args.xml)
    if args.search:
        best = search_conventions(xml, args.search_limit, not args.all_axis_perms)
        print(f"Top {len(best)} FT8978 conventions by row direction vs resolve_matrix")
        print("score = mean absolute row cosine; worst = weakest axis cosine")
        for index, (mean, worst, label, matrix) in enumerate(best, 1):
            print(f"\n{index:>2}. score={mean:.6f} worst={worst:.6f} {label}")
            print_direction_report("resolve", "candidate", resolve_matrix(), matrix)
        return

    a = convention_matrix(args.a, xml)
    b = convention_matrix(args.b, xml)
    targets = pure_targets(args.force, args.torque)

    print(f"Comparing {args.a} vs {args.b}")
    print(f"Pure target magnitudes: force={args.force:g} N, torque={args.torque:g} Nm")
    if args.matrices:
        print_matrix(args.a, a)
        print_matrix(args.b, b)
    if args.offsets:
        print(f"\nXML GaugeGains:   {format_row(gains)}")
        print(f"XML GaugeOffsets: {format_row(offsets)}")
        print_offset_report(args.b, b, offsets)
    print_direction_report(args.a, args.b, a, b)

    print_gages(f"Gages that make pure outputs in {args.a}", a, targets)
    print_gages(f"Gages that make pure outputs in {args.b}", b, targets)
    print_compare(f"Pure {args.a} inputs applied to both mappings", a, b, targets)
    print_compare(f"Pure {args.b} inputs applied to both mappings", b, a, targets)


if __name__ == "__main__":
    main()
