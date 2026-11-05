"""Exercise the installer with fake hardware and package installers, without downloads."""

import json
import os
import shutil
import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "install_ml_deps.sh"
CUDA_INDEX = "https://download.pytorch.org/whl/cu130"


class InstallMLDepsTest(unittest.TestCase):
    """Check selected packages by running the real shell entry point."""

    def setUp(self):
        """Isolate PATH, Python modules and install receipts for each invocation."""
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.receipt = self.root / "installs.jsonl"
        self.env = {
            **os.environ,
            "PATH": str(self.bin),
            "PYTHONPATH": str(self.root),
            "PYTHONNOUSERSITE": "1",
            "INSTALL_RECEIPT": str(self.receipt),
            "INTEL_XPU": "0",
            "FAIL_INSTALL": "0",
        }
        for name in ("head", "grep"):
            (self.bin / name).symlink_to(shutil.which(name))
        self.executable(
            "python3",
            f"""\
            #!{sys.executable}
            import json
            import os
            from pathlib import Path
            import sys

            if sys.argv[1:3] == ["-m", "pip"]:
                with Path(os.environ["INSTALL_RECEIPT"]).open("a") as receipt:
                    receipt.write(json.dumps(sys.argv[3:]) + "\\n")
                sys.exit(int(os.environ["FAIL_INSTALL"]))
            os.execv({sys.executable!r}, [{sys.executable!r}, *sys.argv[1:]])
            """,
        )
        self.executable("pip", "#!/bin/bash\nprintf 'torch fake\\n'")
        self.executable(
            "nvidia-smi",
            """\
            #!/bin/bash
            case "$*" in
                *compute_cap*) printf "%s\\n" "$GPU_CAP" ;;
                *) printf "%s\\n" "$GPU_MODEL" ;;
            esac
            """,
        )
        # Let the real detection and verification snippets run without importing Torch
        # or loading a GPU driver. Package installation itself is recorded above.
        (self.root / "torch.py").write_text(
            textwrap.dedent(
                """\
                from types import SimpleNamespace
                __version__ = "fake"
                cuda = SimpleNamespace(is_available=lambda: False)
                def randn(*shape):
                    return None
                """
            )
        )

    def executable(self, name, source):
        """Write a stand-in executable under the isolated PATH."""
        path = self.bin / name
        path.write_text(textwrap.dedent(source))
        path.chmod(0o755)

    def run_installer(self, gpu, cap, **env):
        """Return the real process result and ordered package-install requests."""
        result = subprocess.run(
            ["/bin/bash", str(SCRIPT)],
            env={**self.env, "GPU_MODEL": gpu, "GPU_CAP": cap, **env},
            cwd=self.root,
            text=True,
            capture_output=True,
            timeout=10,
            check=False,
        )
        calls = [json.loads(line) for line in self.receipt.read_text().splitlines()]
        return result, calls

    def test_blackwell_uses_torchcodec_for_torch29(self):
        """Blackwell selects a TorchCodec release with cp314 wheels and the Torch 2.9 ABI."""
        result, calls = self.run_installer("NVIDIA Blackwell GPU", "12.0")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(
            calls,
            [
                [
                    "install",
                    "torch==2.9.1+cu130",
                    "torchaudio==2.9.1+cu130",
                    "torchvision==0.24.1+cu130",
                    "--index-url",
                    CUDA_INDEX,
                ],
                ["install", "torchcodec==0.9.1"],
            ],
        )

    def test_blackwell_sm100_capability_is_detected(self):
        """Blackwell is detected by architecture, not by GPU model name."""
        result, calls = self.run_installer("NVIDIA Unlisted Model", "10.0")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(calls[0][1], "torch==2.9.1+cu130")
        self.assertEqual(calls[1], ["install", "torchcodec==0.9.1"])

    def test_other_nvidia_selection_is_unchanged(self):
        """The compatibility fix does not change the existing non-Blackwell branch."""
        result, calls = self.run_installer("NVIDIA Ada GPU", "8.9")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(
            calls,
            [
                [
                    "install",
                    "--upgrade",
                    "--index-url",
                    "https://download.pytorch.org/whl/cu124",
                    "torch==2.5.1+cu124",
                    "torchvision==0.20.1+cu124",
                    "torchaudio==2.5.1+cu124",
                ]
            ],
        )

    def test_failed_torch_install_stops_before_codec_and_verification(self):
        """A package-install failure must remain a failing installer exit."""
        result, calls = self.run_installer("NVIDIA Blackwell GPU", "12.0", FAIL_INSTALL="7")
        self.assertEqual(result.returncode, 7)
        self.assertEqual(len(calls), 1)
        self.assertNotIn("Installation Complete", result.stdout)
        self.assertNotIn("tensor test", result.stdout)


if __name__ == "__main__":
    unittest.main()
