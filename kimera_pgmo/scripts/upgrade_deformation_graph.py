#!/usr/bin/env python3
"""Upgrade .dgrf or unversioned PGMO JSON to a version 1 checkpoint.

Optional --metadata JSON maps decimal node keys to {original: {position: [x,y,z],
quaternion: [w,x,y,z]}, stamp_ns: "..."}. Missing original poses require
--allow-incomplete; missing timestamps remain null and are reported.
"""

import argparse
import copy
import json
import math
import os
from pathlib import Path
import sys
import tempfile


FORMAT = "kimera_pgmo.deformation_graph"
INDEX_MASK = (1 << 56) - 1


def pose(numbers):
    x, y, z, qx, qy, qz, qw = map(float, numbers)
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if not math.isfinite(norm) or norm == 0:
        raise ValueError("Invalid legacy quaternion")
    return {"position": [x, y, z], "quaternion": [q / norm for q in (qw, qx, qy, qz)]}


def old_pose(value):
    return pose([value[k] for k in ("x", "y", "z", "qx", "qy", "qz", "qw")])


def node(key, estimate, temporary=False):
    return {"key": str(key), "role": "pose", "original": None,
            "estimate": estimate, "stamp_ns": None,
            "trajectory": not temporary and chr(int(key) >> 56) in "abcdefgh"}


def empty_graph():
    return {"format": FORMAT, "version": 1, "pose_mode": "POSE3",
            "add_initial_vertex_prior": False,
            "permanent": {"nodes": [], "factors": []},
            "temporary": {"nodes": [], "factors": []}}


def matrix(values, dimension):
    if len(values) != dimension * (dimension + 1) // 2:
        raise ValueError("Incorrect legacy information matrix length")
    result = [[0.0] * dimension for _ in range(dimension)]
    cursor = 0
    for row in range(dimension):
        for col in range(row, dimension):
            result[row][col] = result[col][row] = float(values[cursor])
            cursor += 1
    return result


def read_dgrf(path):
    result = empty_graph()
    vertices = {}
    inliers = {"permanent": set(), "temporary": set()}
    for line_no, line in enumerate(path.read_text().splitlines(), 1):
        parts = line.split()
        if not parts or parts[0].startswith("#"):
            continue
        tag, *data = parts
        temporary = tag.endswith("_TEMP") or tag.startswith("TEMP_")
        name = "temporary" if temporary else "permanent"
        state = result[name]
        tag = tag.removesuffix("_TEMP").removeprefix("TEMP_")
        try:
            if tag == "NODE":
                state["nodes"].append(node(data[0], pose(data[1:]), temporary))
            elif tag in ("PRIOR", "BETWEEN", "DEDGE"):
                arity = 1 if tag == "PRIOR" else 2
                n = 3 if tag == "DEDGE" else 7
                measurement = list(map(float, data[arity:arity + n]))
                factor = {"type": {"PRIOR": "prior3", "BETWEEN": "between3", "DEDGE": "deformation33"}[tag],
                          "keys": data[:arity],
                          "measurement": measurement if tag == "DEDGE" else pose(measurement),
                          "information": matrix(data[arity + n:], 3 if tag == "DEDGE" else 6),
                          "known_inlier": False, "weight": None}
                state["factors"].append(factor)
            elif tag == "VERTEX":
                if len(data) != 5 or data[0] in vertices:
                    raise ValueError("Invalid or duplicate VERTEX")
                vertices[data[0]] = (data[1], list(map(float, data[2:])))
            elif tag == "KNOWN_INLIERS":
                inliers[name].update(map(int, data))
            else:
                raise ValueError(f"Unknown tag {tag}")
        except (ValueError, IndexError) as error:
            raise ValueError(f"{path}:{line_no}: {error}") from error
    for name in ("permanent", "temporary"):
        for index in inliers[name]:
            if index < 0 or index >= len(result[name]["factors"]):
                raise ValueError("Legacy inlier index out of range")
            result[name]["factors"][index]["known_inlier"] = True
    apply_vertices(result, vertices)
    return result


def apply_vertices(result, vertices):
    seen = set()
    for entry in result["permanent"]["nodes"]:
        if entry["key"] in vertices:
            stamp, position = vertices[entry["key"]]
            entry.update(role="mesh", trajectory=False, stamp_ns=str(stamp),
                         original={"position": position, "quaternion": [1.0, 0.0, 0.0, 0.0]})
            seen.add(entry["key"])
    if seen != set(vertices):
        raise ValueError("Legacy control point has no estimated value")


def read_legacy_json(path):
    data = json.loads(path.read_text())
    if "format" in data:
        raise ValueError("Input is already versioned; this tool accepts legacy files only")
    result = empty_graph()
    result["add_initial_vertex_prior"] = data.get("add_init_vertex_prior", False)
    initial = dict(data.get("pg_initial_poses", []))
    temp_initial = {str(k): v for k, v in dict(data.get("temp_pg_initial_poses", [])).items()}
    pose_stamps = dict(data.get("pg_stamps", []))
    for name, prefix in (("permanent", ""), ("temporary", "temp_")):
        state = result[name]
        temporary = bool(prefix)
        for record in data.get(prefix + "values", []):
            entry = node(record["key"], old_pose(record["value"]), temporary)
            key = int(entry["key"])
            symbol, index = key >> 56, key & INDEX_MASK
            originals = initial.get(symbol, initial.get(str(symbol), initial.get(chr(symbol))))
            stamps = pose_stamps.get(symbol, pose_stamps.get(str(symbol), pose_stamps.get(chr(symbol))))
            if temporary and entry["key"] in temp_initial:
                entry["original"] = old_pose(temp_initial[entry["key"]])
            elif not temporary and originals is not None and index < len(originals):
                entry["original"] = old_pose(originals[index])
                entry["trajectory"] = True
            if stamps is not None and index < len(stamps):
                entry["stamp_ns"] = str(stamps[index])
            state["nodes"].append(entry)
        inliers = set(data.get(prefix + "known_inliers", []))
        weights = data.get(prefix + "inlier_weights", [])
        factors = data.get(prefix + "factors", [])
        if weights and len(weights) != len(factors):
            raise ValueError("Legacy factor weights are misaligned")
        if any(i < 0 or i >= len(factors) for i in inliers):
            raise ValueError("Legacy inlier index out of range")
        for index, record in enumerate(factors):
            kind = record["type"]
            info = record["information"]
            if info["rows"] != info["cols"] or len(info["data"]) != info["rows"] ** 2:
                raise ValueError("Invalid legacy information matrix")
            dimension = info["rows"]
            keys = [record["key"]] if kind == "prior" else [record["key1"], record["key2"]]
            state["factors"].append({
                "type": {"prior": "prior3", "between": "between3", "dedge": "deformation33"}[kind],
                "keys": list(map(str, keys)),
                "measurement": record["measurement"] if kind == "dedge" else old_pose(record["measurement"]),
                "information": [info["data"][i * dimension:(i + 1) * dimension] for i in range(dimension)],
                "known_inlier": index in inliers, "weight": weights[index] if weights else None})
    vertices = {}
    for prefix, record in data.get("vertices", {}).items():
        if len(record["pos"]) != len(record["stamps"]):
            raise ValueError("Legacy control positions and timestamps are misaligned")
        for index, (position, stamp) in enumerate(zip(record["pos"], record["stamps"])):
            vertices[str((ord(prefix) << 56) | index)] = (stamp, position)
    apply_vertices(result, vertices)
    return result


def upgrade(path, metadata=None, allow_incomplete=False):
    result = read_legacy_json(path) if path.read_text().lstrip().startswith("{") else read_dgrf(path)
    metadata = metadata or {}
    substituted, missing_stamps = [], []
    all_keys = set()
    for name in ("permanent", "temporary"):
        result[name]["nodes"].sort(key=lambda entry: int(entry["key"]))
        for entry in result[name]["nodes"]:
            key = entry["key"]
            if key in all_keys:
                raise ValueError(f"Duplicate key {key}")
            all_keys.add(key)
            supplied = metadata.get(key, {})
            for field in ("original", "stamp_ns"):
                if field in supplied:
                    entry[field] = supplied[field]
            if entry["original"] is None:
                substituted.append(key)
                entry["original"] = copy.deepcopy(entry["estimate"])
            if entry["stamp_ns"] is None:
                missing_stamps.append(key)
            else:
                entry["stamp_ns"] = str(entry["stamp_ns"])
    for name in ("permanent", "temporary"):
        for factor in result[name]["factors"]:
            if not set(factor["keys"]) <= all_keys:
                raise ValueError("Legacy factor references a missing node")
    if substituted and not allow_incomplete:
        raise ValueError(f"{len(substituted)} original poses are unavailable. Supply --metadata or explicitly use --allow-incomplete")
    result["migration"] = {"source": str(path), "substituted_original_pose_keys": substituted,
                           "missing_timestamp_keys": missing_stamps}
    # Reject non-finite legacy values before writing anything.
    json.dumps(result, allow_nan=False)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--metadata", type=Path)
    parser.add_argument("--allow-incomplete", action="store_true")
    args = parser.parse_args()
    if args.input.resolve() == args.output.resolve():
        parser.error("Choose a separate output file to preserve the legacy input")
    try:
        metadata = json.loads(args.metadata.read_text()) if args.metadata else None
        result = upgrade(args.input, metadata, args.allow_incomplete)
        with tempfile.NamedTemporaryFile(mode="w", dir=args.output.parent, delete=False) as stream:
            temporary = Path(stream.name)
            try:
                json.dump(result, stream, allow_nan=False)
                stream.write("\n")
                stream.flush()
                os.replace(temporary, args.output)
            finally:
                temporary.unlink(missing_ok=True)
        print(json.dumps(result["migration"], indent=2), file=sys.stderr)
    except (ValueError, KeyError, OSError) as error:
        parser.exit(1, f"Upgrade failed: {error}\n")


if __name__ == "__main__":
    main()
