"""Check the user docs: links and anchors resolve, no agent-doc links, no WSL.

usage: python3 -I docs_check.py <repo_root>
Prints one FAIL line per problem and exits 1 if there is any.
"""

import pathlib
import re
import sys

root = pathlib.Path(sys.argv[1]).resolve()
user_docs = [root / "README.md"] + sorted(
    p
    for p in (root / "docs").rglob("*.md")
    if p.relative_to(root / "docs").parts[0] not in ("agents", "adr")
)
agent_paths = ("docs/agents/", "docs/adr/", "CLAUDE.md")
banned = re.compile(
    r"\bwsl\w*\b|wslg|/mnt/c\b|windows shell|docker desktop", re.I
)
link = re.compile(r"\]\(([^)\s]+)\)")
fails = []


def anchors(path):
    out = set()
    for line in path.read_text().splitlines():
        m = re.match(r"#{1,6}\s+(.*)", line)
        if m:
            slug = re.sub(r"[^\w\- ]", "", m.group(1).strip().lower())
            out.add(slug.replace(" ", "-"))
    return out


for doc in user_docs:
    rel = doc.relative_to(root)
    text = doc.read_text()
    in_fence = False
    for n, line in enumerate(text.splitlines(), 1):
        if line.startswith("```"):
            in_fence = not in_fence
        if banned.search(line):
            fails.append(f"{rel}:{n} WSL/Windows wording: {line.strip()}")
        if in_fence:
            continue
        for target in link.findall(line):
            if re.match(r"[a-z]+:", target):
                continue
            path, _, frag = target.partition("#")
            dest = (doc.parent / path).resolve() if path else doc
            dest_rel = (
                dest.relative_to(root).as_posix()
                if dest.is_relative_to(root)
                else str(dest)
            )
            if any(dest_rel.startswith(a) for a in agent_paths):
                fails.append(f"{rel}:{n} links an agent doc: {target}")
            if not dest.exists():
                fails.append(f"{rel}:{n} missing target: {target}")
            elif frag and dest.suffix == ".md" and frag not in anchors(dest):
                fails.append(f"{rel}:{n} missing anchor: {target}")

for f in fails:
    print("FAIL", f)
print(f"checked {len(user_docs)} user docs, {len(fails)} problems")
sys.exit(1 if fails else 0)
