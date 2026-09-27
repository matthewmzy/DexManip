"""Exercise private bundle integrity, traversal rejection and non-overwriting import."""
import hashlib
import importlib.util
import io
from pathlib import Path
import tarfile
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location("transfer_assets", Path(__file__).resolve().parents[1] / "scripts/transfer_assets.py")
assets = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(assets)


class TransferAssetsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.old_dest = assets.DEST
        assets.DEST = self.root / "installed"
        self.files = {"model.bin": {"bytes": 3, "sha256": hashlib.sha256(b"USD").hexdigest()}}
        self.bundle = self.root / "assets.tar.gz"

    def tearDown(self):
        assets.DEST = self.old_dest
        self.temp.cleanup()

    def archive(self, names, *, symlink=False):
        with tarfile.open(self.bundle, "w:gz") as handle:
            for name in names:
                info = tarfile.TarInfo(name)
                if symlink:
                    info.type = tarfile.SYMTYPE
                    info.linkname = "../outside"
                    handle.addfile(info)
                else:
                    info.size = 3
                    handle.addfile(info, io.BytesIO(b"USD"))

    def test_pack_and_import_keep_exact_bytes(self):
        assets.DEST.mkdir()
        (assets.DEST / "model.bin").write_bytes(b"USD")
        assets.pack(self.bundle, self.files)
        assets.DEST = self.root / "new_checkout"
        assets.unpack(self.bundle, self.files)
        self.assertEqual((assets.DEST / "model.bin").read_bytes(), b"USD")
        assets.unpack(self.bundle, self.files)  # Idempotent only if installed hash matches.

    def test_pack_refuses_modified_source(self):
        assets.DEST.mkdir()
        (assets.DEST / "model.bin").write_bytes(b"bad")
        with self.assertRaisesRegex(ValueError, "checksum"):
            assets.pack(self.bundle, self.files)
        self.assertFalse(self.bundle.exists())

    def test_import_rejects_traversal(self):
        self.archive(["../outside"])
        with self.assertRaisesRegex(ValueError, "Unexpected"):
            assets.unpack(self.bundle, self.files)
        self.assertFalse((self.root / "outside").exists())
        self.assertFalse(assets.DEST.exists())

    def test_import_rejects_symlink(self):
        self.archive(["model.bin"], symlink=True)
        with self.assertRaisesRegex(ValueError, "nonregular"):
            assets.unpack(self.bundle, self.files)
        self.assertFalse(assets.DEST.exists())

    def test_import_rejects_duplicate_or_missing_member(self):
        for names in (["model.bin", "model.bin"], []):
            self.archive(names)
            with self.assertRaises(ValueError):
                assets.unpack(self.bundle, self.files)
            self.assertFalse(assets.DEST.exists())

    def test_import_refuses_changed_existing_install(self):
        assets.DEST.mkdir()
        (assets.DEST / "model.bin").write_bytes(b"old")
        self.archive(["model.bin"])
        with self.assertRaisesRegex(ValueError, "checksum"):
            assets.unpack(self.bundle, self.files)
        self.assertEqual((assets.DEST / "model.bin").read_bytes(), b"old")


if __name__ == "__main__":
    unittest.main()
