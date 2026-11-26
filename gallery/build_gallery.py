from pathlib import Path
import json
import shutil
import re

base = Path(__file__).resolve().parent
repo = base.parent
dest = repo / "_site"
items = json.loads((base / "catalog.json").read_text(encoding="utf-8"))
assert len({item["path"] for item in items}) == len(items)
assert sum(item["group"] == "current" and item["include"] for item in items) == 1
embedded = re.search(r"const catalogue = (\[.*?\])\s*;", (base / "index.html").read_text(encoding="utf-8"), re.S)
assert embedded and json.loads(embedded.group(1)) == items, "HTML/catalogue mismatch"
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
print(f"Built gallery: {sum(item['include'] for item in items)} players; media and posters verified.")
