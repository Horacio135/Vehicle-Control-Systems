from pathlib import Path
import json
import shutil

base = Path(__file__).resolve().parent
repo = base.parent
dest = repo / "_site"
items = json.loads((base / "catalog.json").read_text(encoding="utf-8"))
assert len(items) == 19 and sum(item["include"] for item in items) == 9
for item in items:
    assert (repo / item["path"]).is_file(), item["path"]
dest.mkdir(exist_ok=True)
shutil.copy2(base / "index.html", dest / "index.html")
(dest / ".nojekyll").write_text("", encoding="utf-8")
for item in items:
    if item["include"]:
        for key in ("path", "poster"):
            source = repo / item[key]
            target = dest / item[key]
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
print("Built gallery: 9 players, 10 archive links; media and posters verified.")
