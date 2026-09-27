#!/usr/bin/env python3
"""Pack/import the private SI model bundle locally; never uploads or downloads."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / "assets/h2_sharpa"
MANIFEST = ROOT / "assets/h2_sharpa_manifest.json"


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def verify(folder, files):
    for name, expected in files.items():
        path = folder / name
        if not path.is_file() or path.is_symlink():
            raise ValueError(f"Missing regular asset: {path}")
        if path.stat().st_size != expected["bytes"] or sha256(path) != expected["sha256"]:
            raise ValueError(f"Asset checksum/size mismatch: {name}")


def pack(output, files):
    verify(DEST, files)
    output.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive create: never overwrite a manually transferred bundle.
    with output.open("xb") as handle:
        with tarfile.open(fileobj=handle, mode="w:gz") as archive:
            for name in sorted(files):
                archive.add(DEST / name, arcname=name, recursive=False)
    print(f"Private bundle: {output}\nSHA256: {sha256(output)}")


def unpack(source, files):
    if DEST.exists():
        verify(DEST, files)
        print(f"Already installed and verified: {DEST}")
        return
    DEST.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".si-assets-", dir=DEST.parent) as temp:
        staging = Path(temp) / "model"
        staging.mkdir()
        with tarfile.open(source, mode="r:gz") as archive:
            seen = set()
            for member in archive:
                # No extractall: accept only the six exact regular file names.
                if member.name not in files or member.name in seen or not member.isfile():
                    raise ValueError(f"Unexpected/duplicate/nonregular archive member: {member.name}")
                if member.size != files[member.name]["bytes"]:
                    raise ValueError(f"Wrong archive member size: {member.name}")
                target = staging / member.name
                target.parent.mkdir(parents=True, exist_ok=True)
                with archive.extractfile(member) as data, target.open("xb") as out:
                    shutil.copyfileobj(data, out)
                seen.add(member.name)
        if seen != set(files):
            raise ValueError(f"Incomplete bundle: missing {sorted(set(files) - seen)}")
        verify(staging, files)
        staging.rename(DEST)
    print(f"Imported and verified {len(files)} private assets: {DEST}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("verify")
    sub.add_parser("pack").add_argument("--out", type=Path, required=True)
    sub.add_parser("import").add_argument("bundle", type=Path)
    args = parser.parse_args()
    files = json.loads(MANIFEST.read_text())["files"]
    try:
        if args.command == "pack":
            pack(args.out.resolve(), files)
        elif args.command == "import":
            unpack(args.bundle.resolve(), files)
        else:
            verify(DEST, files)
            print(f"Verified {len(files)} private assets")
    except (ValueError, OSError, tarfile.TarError) as exc:
        parser.exit(1, f"Asset error: {exc}\n")


if __name__ == "__main__":
    main()
