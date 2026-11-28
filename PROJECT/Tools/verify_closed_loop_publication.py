"""Check published closed-loop evidence without MATLAB or third-party packages.

Run from any directory with Python 3.9 or later. The checks cover source and
artifact integrity, local Markdown links, selected sensitive-text patterns and
the recorded headline metrics. They do not validate a physical vehicle, decode
MAT-file metadata, or replace simulation and estimator tests.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path
import re
from urllib.parse import unquote, urlsplit
import zipfile


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "PROJECT/Documentation/closed_loop_artifact_manifest.json"
REPORT = ROOT / "PROJECT/Tools/closed_loop_publication_checks.json"
MAX_FILE_BYTES = 100_000_000
TEXT_SUFFIXES = {".m", ".md", ".json", ".csv", ".py", ".txt"}
WORKSTATION_PATH = re.compile(r"[A-Za-z]:[\\/]+Users[\\/]+|[A-Za-z]:\\[^\s\"<>]+")
SENSITIVE_TEXT = re.compile(
    r"ghp_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}"
    r"|-----BEGIN [A-Z ]*PRIVATE KEY-----"
)
MARKDOWN_LINK = re.compile(
    r'!?\[[^\]]*\]\((?:<([^>]+)>|([^\s)]+))(?:\s+"[^"]*")?\)'
)


def digest(path: Path) -> str:
    """Hash large histories and videos without loading them into memory."""
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def main() -> int:
    errors: list[str] = []
    counts = {
        "manifest_files_checked": 0,
        "active_sources_checked": 0,
        "deduplicated_references_checked": 0,
        "selected_text_files_checked": 0,
        "selected_live_scripts_checked": 0,
        "markdown_files_checked": 0,
        "local_links_checked": 0,
        "required_videos_checked": 0,
    }
    try:
        manifest = json.loads(MANIFEST.read_text(encoding="utf-8-sig"))
    except (OSError, ValueError) as error:
        errors.append(f"Cannot read artifact manifest: {type(error).__name__}")
        manifest = {"files": [], "deduplicated_files": []}

    def local_path(relative: str) -> Path | None:
        path = (ROOT / relative).resolve()
        try:
            path.relative_to(ROOT)
        except ValueError:
            errors.append(f"Manifest path escapes repository: {relative}")
            return None
        return path

    def inspect_text(text: str, relative: str) -> None:
        # Report the file only; never echo a potentially sensitive match.
        if WORKSTATION_PATH.search(text):
            errors.append(f"Workstation-specific absolute path in selected text: {relative}")
        if SENSITIVE_TEXT.search(text):
            errors.append(f"Possible credential/private key in selected text: {relative}")

    indexed_paths: set[str] = set()
    for entry in manifest.get("files", []):
        relative = entry.get("path", "")
        if not relative:
            errors.append("Manifest entry has no path")
            continue
        if relative in indexed_paths:
            errors.append(f"Duplicate manifest destination: {relative}")
        indexed_paths.add(relative)
        path = local_path(relative)
        if path is None:
            continue
        if not path.is_file():
            errors.append(f"Missing artifact: {relative}")
            continue
        counts["manifest_files_checked"] += 1
        size = path.stat().st_size
        actual_digest = digest(path)
        if size >= MAX_FILE_BYTES:
            errors.append(f"Artifact reaches conservative GitHub 100 MB limit: {relative}")
        if actual_digest != entry.get("sha256"):
            errors.append(f"Artifact SHA-256 differs from manifest: {relative}")
        if "bytes" in entry and size != entry["bytes"]:
            errors.append(f"Artifact size differs from manifest: {relative}")
        if entry.get("kind") == "active_source":
            counts["active_sources_checked"] += 1
            if actual_digest != entry.get("source_sha256"):
                errors.append(f"Active source differs from recorded original: {relative}")
        try:
            if path.suffix.lower() in TEXT_SUFFIXES:
                counts["selected_text_files_checked"] += 1
                inspect_text(path.read_text(encoding="utf-8-sig"), relative)
            elif path.suffix.lower() == ".mlx":
                counts["selected_live_scripts_checked"] += 1
                with zipfile.ZipFile(path) as archive:
                    inspect_text(archive.read("matlab/document.xml").decode("utf-8"), relative)
        except (OSError, UnicodeError, KeyError, zipfile.BadZipFile) as error:
            errors.append(f"Cannot inspect selected text/Live Script {relative}: {type(error).__name__}")

    for alias in manifest.get("deduplicated_files", []):
        relative = alias.get("published_as", "")
        path = local_path(relative) if relative else None
        counts["deduplicated_references_checked"] += 1
        if path is None or not path.is_file() or relative not in indexed_paths:
            errors.append(f"Invalid deduplicated artifact reference: {relative or '(missing)'}")

    # Check the documentation tree, including historical READMEs. Text scanning
    # above is limited to the new manifest, so unrelated dirty legacy files are
    # not represented as part of this publication's sensitive-text audit.
    for path in sorted(ROOT.rglob("*.md")):
        if ".git" in path.parts:
            continue
        relative = path.relative_to(ROOT).as_posix()
        counts["markdown_files_checked"] += 1
        try:
            text = path.read_text(encoding="utf-8-sig")
        except (OSError, UnicodeError) as error:
            errors.append(f"Cannot read Markdown {relative}: {type(error).__name__}")
            continue
        for match in MARKDOWN_LINK.finditer(text):
            reference = match.group(1) or match.group(2)
            if reference.startswith(("#", "http:", "https:", "mailto:")):
                continue
            parsed = urlsplit(reference)
            if parsed.scheme:
                continue
            target = (path.parent / unquote(parsed.path)).resolve()
            counts["local_links_checked"] += 1
            if not target.exists():
                errors.append(f"Broken local Markdown link: {relative} -> {reference}")

    closed_loop = ROOT / "PROJECT/Results/Closed_Loop"
    recorded_metrics: dict[str, float] = {}
    try:
        with (closed_loop / "summary.csv").open(encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
        controlled = next(
            row for row in rows
            if row.get("Caso", row.get("Case", "")) in {
                "Lazo estimado con ABS/TC", "Estimated loop with ABS/TC"
            }
        )
        recorded_metrics = {
            "controlled_lap_time_s": float(controlled["Tiempo_s"]),
            "controlled_maximum_path_error_m": float(controlled["ErrorMax_m"]),
        }
        if not math.isclose(recorded_metrics["controlled_lap_time_s"], 47.370, abs_tol=1e-9):
            errors.append("Controlled-lap duration differs from recorded 47.370 s")
        if not math.isclose(
            recorded_metrics["controlled_maximum_path_error_m"],
            0.349556814569182,
            abs_tol=1e-9,
        ):
            errors.append("Controlled maximum path error differs from recorded evidence")
    except (OSError, ValueError, KeyError, StopIteration) as error:
        errors.append(f"Cannot verify recorded closed-loop summary: {type(error).__name__}")

    for name in [
        "Actual_vs_estimated_ABS_TC_2D.mp4",
        "Closed_loop_ABS_TC_comparison_2D_x4.mp4",
        "First_Failed_Attempt/First_failed_attempt_2D.mp4",
    ]:
        video = closed_loop / name
        counts["required_videos_checked"] += 1
        if not video.is_file() or video.stat().st_size == 0:
            errors.append(f"Required closed-loop video missing or empty: {name}")

    # Keep historical checks above and verify the selected modulated-ABS run
    # separately. Legacy MAT names do not mean that TC is disabled.
    current_metrics = {}
    try:
        current = ROOT / "PROJECT/Results/Modulated_ABS/Circuit_14"
        with (current / "resumen.csv").open(encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
        expected = [("ABS desactivado, TC activado", 49.250), ("ABS y TC activados", 47.294)]
        for label, duration in expected:
            row = next(row for row in rows if row["Caso"] == label)
            if not math.isclose(float(row["Tiempo_s"]), duration, abs_tol=1e-9):
                errors.append(f"Current ABS comparison duration mismatch: {label}")
            if row["Finalizacion"] != "Vuelta completada":
                errors.append(f"Current ABS comparison did not complete: {label}")
        current_metrics = {
            "modulated_abs_lap_time_s": float(rows[1]["Tiempo_s"]),
            "abs_off_tc_on_lap_time_s": float(rows[0]["Tiempo_s"]),
            "modulated_abs_maximum_path_error_m": float(rows[1]["ErrorMax_m"]),
        }
        for name in ["Comparacion_ABS_off_on_TC_on_2D_x4.mp4", "ABS_TC_on_real_estimado_2D.mp4"]:
            counts["required_videos_checked"] += 1
            if not (current / name).is_file() or (current / name).stat().st_size == 0:
                errors.append(f"Current ABS video missing or empty: {name}")
    except (OSError, ValueError, KeyError, StopIteration) as error:
        errors.append(f"Cannot verify current ABS summary: {type(error).__name__}")

    esc_metrics = {}
    try:
        esc = ROOT / "PROJECT/Results/ESC_V1_13m"
        checks = json.loads((esc / "Verificacion_corredor.json").read_text(encoding="utf-8-sig"))
        baseline, controlled_esc = checks
        if baseline["finalizado"] or not controlled_esc["finalizado"]:
            errors.append("ESC manoeuvre termination differs from recorded evidence")
        if not math.isclose(controlled_esc["beta_max_grados"], 1.8808543734006689, abs_tol=1e-10):
            errors.append("ESC controlled sideslip differs from recorded evidence")
        if controlled_esc["conos_contactados"] != 3 or controlled_esc["dentroCorredor"]:
            errors.append("ESC cone-contact limitation must remain explicit")
        render = json.loads((esc / "Verificacion_render_3D_naranja_giro15_opuesto.json").read_text(encoding="utf-8-sig"))
        if render["simulacion_repetida"] or not render["camara_rotacion_constante"]:
            errors.append("Approved 3D recording metadata is inconsistent")
        if render["camara_desplazamiento"][2] != 12:
            errors.append("Approved camera height differs from 12 m")
        for name in ["Alce_ESC_V1_13m_3D_naranja_giro15_opuesto.mp4", "Comparacion_alce_02_sin_con_ESC_2D.mp4"]:
            counts["required_videos_checked"] += 1
            if not (esc / name).is_file() or (esc / name).stat().st_size == 0:
                errors.append(f"Required ESC video missing: {name}")
        esc_metrics = {
            "ESC_V1_maximum_sideslip_deg": controlled_esc["beta_max_grados"],
            "ESC_V1_cone_contacts": controlled_esc["conos_contactados"],
            "ESC_V1_perfect_state_test": True,
        }
    except (OSError, ValueError, KeyError, IndexError) as error:
        errors.append(f"Cannot verify ESC V1 evidence: {type(error).__name__}")

    report = {
        "snapshot_date": manifest.get("snapshot_date"),
        **counts,
        **recorded_metrics,
        **current_metrics,
        **esc_metrics,
        "success": not errors,
        "errors": errors,
        "scope": (
            "Artifact hashes, byte-exact active-source provenance, file sizes, "
            "deduplication references, local Markdown link targets, selected "
            "text/MLX privacy patterns and recorded headline metrics. "
            "Does not execute MATLAB, inspect binary MAT metadata, validate "
            "remote links or certify experimental vehicle-model accuracy."
        ),
    }
    REPORT.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
