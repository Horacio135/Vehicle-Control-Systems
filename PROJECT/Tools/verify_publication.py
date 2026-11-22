"""Verify documentation links and archived evidence without external dependencies."""
from pathlib import Path
from urllib.parse import unquote,urlsplit
import csv,hashlib,json,re,subprocess,zipfile,xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]
errors=[];checked_links=0
for p in ROOT.rglob('*.md'):
    if '.git' in p.parts:continue
    s=p.read_text(encoding='utf-8-sig')
    for match in re.finditer(r'!?\[[^\]]*\]\((?:<([^>]+)>|([^\s)]+))(?:\s+"[^"]*")?\)',s):
        ref=match.group(1) or match.group(2)
        if ref.startswith(('#','http:','https:','mailto:')):continue
        parsed=urlsplit(ref)
        if parsed.scheme:continue
        target=(p.parent/unquote(parsed.path)).resolve()
        checked_links+=1
        if not target.exists():errors.append(f'Broken link: {p.relative_to(ROOT)} -> {ref}')

manifest=json.loads((ROOT/'PROJECT/Documentation/artifact_manifest.json').read_text(encoding='utf-8'))
for entry in manifest['files']:
    p=ROOT/entry['path']
    if not p.exists() or hashlib.sha256(p.read_bytes()).hexdigest()!=entry['sha256']:
        errors.append('Artifact differs: '+entry['path'])

# Compare archived bytes against the Git index too: checkout newline conversion
# must not invalidate the published artifact manifest.
index={}
try:
    raw=subprocess.check_output(['git','ls-files','-s','-z'],cwd=ROOT)
    for item in raw.split(b'\0'):
        if not item:continue
        info,path=item.split(b'\t',1);index[path.decode('utf-8')]=info.split()[1].decode()
except (FileNotFoundError,subprocess.CalledProcessError):pass
index_checks=0
for entry in manifest['files']:
    if entry['path'] not in index:continue
    data=(ROOT/entry['path']).read_bytes()
    expected=hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
    index_checks+=1
    if index[entry['path']]!=expected:errors.append('Git index changed archived bytes: '+entry['path'])

ns={'w':'http://schemas.openxmlformats.org/wordprocessingml/2006/main'}
exports=0
for p in ROOT.rglob('*.mlx'):
    exp=p.parent/'Source Exports'/p.with_suffix('.m').name
    if not exp.exists():continue # Original course material is retained verbatim.
    with zipfile.ZipFile(p) as z:doc=ET.fromstring(z.read('matlab/document.xml'))
    blocks=[]
    for para in doc.findall('.//w:p',ns):
        style=para.find('w:pPr/w:pStyle',ns)
        if style is not None and style.attrib.get('{'+ns['w']+'}val')=='code':blocks.append(''.join(para.itertext()))
    if exp.read_text(encoding='utf-8')!='\n\n'.join(blocks):errors.append('Code export mismatch: '+str(exp.relative_to(ROOT)))
    exports+=1

for n in range(1,13):
    if not (ROOT/f'PROJECT/Documentation/Versions/V{n:02}/README.md').exists():errors.append(f'Missing version V{n}')

module=ROOT/'PROJECT/Modular Architecture'
if module.exists():
    provenance=json.loads((module/'plant_provenance.json').read_text(encoding='utf-8'))
    for name,key in [('pasoDinamico.m','plant_sha256'),('inicializarEstadoDinamico.m','initializer_sha256')]:
        if hashlib.sha256((module/name).read_bytes()).hexdigest()!=provenance[key]:
            errors.append('Standalone plant changed after extraction: '+name)
    source=ROOT/provenance['source_export']
    if hashlib.sha256(source.read_bytes()).hexdigest()!=provenance['source_export_sha256']:
        errors.append('Standalone extraction source has changed')
for p in ROOT.rglob('*'):
    if '.git' in p.parts or not p.is_file():continue
    if p.stat().st_size>=100_000_000:errors.append('File exceeds GitHub 100 MB: '+str(p.relative_to(ROOT)))
    if p.suffix.lower() in {'.asv','.bak'} or p.name.startswith('~$'):errors.append('Temporary file staged: '+str(p.relative_to(ROOT)))
    if p.suffix.lower() in {'.m','.md','.json','.csv','.py'}:
        text=p.read_text(encoding='utf-8-sig')
        if re.search(r'(?:ghp_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|-----BEGIN [A-Z ]*PRIVATE KEY-----)',text):errors.append('Possible credential: '+str(p.relative_to(ROOT)))

rows=list(csv.DictReader((ROOT/'PROJECT/Results/Numerical/summary_all_runs.csv').open(encoding='utf-8-sig')))
if len(rows)!=50:errors.append('Expected 50 historical result rows')
def record(dataset,case):return next(r for r in rows if r['dataset']==dataset and r['case']==case)
assert abs(float(record('V11_single_3D','hist35')['duration_s'])-47.680)<1e-9
assert abs(float(record('V12_aggressive_3D','hist35')['duration_s'])-149.036)<1e-9
assert float(record('V12_aggressive_3D','hist35')['CG_outside_pct'])>18
for r in rows:
    f=ROOT/'PROJECT/Results/Numerical'/f"{r['dataset']}_{r['case']}.csv"
    if not f.exists():errors.append('Missing telemetry: '+f.name)

report={'local_links_checked':checked_links,'archived_files_hash_checked':len(manifest['files']),
        'archived_git_blobs_checked':index_checks,
        'source_exports_checked':exports,'version_readmes_checked':12,'historical_result_rows':len(rows),
        'errors':errors,'scope':'Artifact integrity, documentation and export checks; not experimental physical validation.'}
print(json.dumps(report,ensure_ascii=False,indent=2))
(ROOT/'PROJECT/Tools/publication_checks.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n')
if errors:raise SystemExit(1)
