"""Check the official 33-corner package, unchanged core and evidence hashes.

Standard library only. MATLAB portability/ESC-equivalence checks are recorded
separately; this check does not rerun a simulation or validate a real vehicle.
"""
from pathlib import Path
import csv
import hashlib
import json
import math
import re

ROOT = Path(__file__).resolve().parents[2]
manifest = json.loads((ROOT / 'PROJECT/Documentation/extreme_33_artifact_manifest.json').read_text(encoding='utf-8'))
errors = []

def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(chunk)
    return value.hexdigest()

for item in manifest['artifacts'] + manifest['unchanged_core']:
    path = (ROOT / item['path']).resolve()
    if not path.is_relative_to(ROOT) or not path.is_file():
        errors.append('Missing/out-of-tree file: ' + item['path'])
        continue
    if digest(path) != item['sha256']:
        errors.append('Hash mismatch: ' + item['path'])
    if path.stat().st_size >= 100_000_000:
        errors.append('File reaches conservative GitHub limit: ' + item['path'])
    if path.suffix.lower() in {'.m', '.md', '.json', '.csv', '.txt'}:
        text = path.read_text(encoding='utf-8-sig')
        if re.search(r'[A-Za-z]:[\\/]+Users[\\/]+|ghp_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|-----BEGIN [A-Z ]*PRIVATE KEY-----', text):
            errors.append('Workstation path or sensitive-text pattern: ' + item['path'])

campaign = ROOT / 'PROJECT/Results/Extreme_33_Corners'
full = json.loads((campaign / 'Complete_Loop/Resumen.json').read_text(encoding='utf-8'))
with (campaign / 'Perfect_State/Resumen.csv').open(encoding='utf-8-sig', newline='') as stream:
    rows = list(csv.DictReader(stream))
baseline = rows[3]
assert full['finalizacion'] == 'Vuelta completada'
assert math.isclose(full['tiempo_s'], 191.074)
assert math.isclose(float(baseline['Tiempo_s']), 190.006)
assert math.isclose(full['rmse_posicion_estimador_m'], .15399924598975304)
assert len(manifest['unchanged_core']) == 18
config = json.loads((campaign / 'Complete_Loop/Verificacion_lazo.json').read_text(encoding='utf-8'))
assert config['lazo_sensores_estimador_activo'] and not config['estado_perfecto_en_control']
assert math.isclose(config['Q'][0][0], 1e-6)
catalogue = json.loads((ROOT / 'gallery/catalog.json').read_text(encoding='utf-8'))
official = next(item for item in catalogue if item['id'] == 'video-25')
assert official['include'] and official['group'] == 'current'
assert 'Temporary' not in official['path'] and 'temporary' not in official['title'].lower()
assert official['path'].endswith('Complete_Loop/Full_loop_33_corners_2D_actual_estimated.mp4')
report = dict(success=not errors, errors=errors, artifacts_checked=len(manifest['artifacts']),
              unchanged_core_checked=18, official_video=official['path'],
              perfect_lap_s=190.006, estimated_lap_s=191.074,
              lap_time_change_percent=(191.074 / 190.006 - 1) * 100,
              scope='Artifact integrity, recorded metrics, unchanged core and official gallery metadata; no new simulation or experimental validation.')
(ROOT / 'PROJECT/Tools/extreme_33_publication_checks.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
print(json.dumps(report, indent=2))
raise SystemExit(bool(errors))
