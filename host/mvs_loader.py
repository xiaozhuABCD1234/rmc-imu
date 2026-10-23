import os
import platform
import sys
from pathlib import Path

_DLL_HANDLES = []


def _candidate_roots():
    roots = []
    env = os.environ.get("MVS_SDK_PATH")
    if env:
        roots.append(Path(env))

    if platform.system() == "Windows":
        roots.extend(
            [
                Path(r"D:\BaiduNetdiskDownload\MVS"),
                Path(r"C:\Program Files (x86)\MVS"),
                Path(r"C:\Program Files\MVS"),
            ]
        )
        dll_dirs = [
            Path(r"C:\Program Files (x86)\Common Files\MVS\Runtime\Win64_x64"),
            Path(r"C:\Program Files\Common Files\MVS\Runtime\Win64_x64"),
        ]
        for dll_dir in dll_dirs:
            if dll_dir.is_dir():
                try:
                    _DLL_HANDLES.append(os.add_dll_directory(str(dll_dir)))
                except (AttributeError, OSError):
                    os.environ["PATH"] = str(dll_dir) + os.pathsep + os.environ["PATH"]
    else:
        roots.extend(
            [
                Path("/opt/MVS"),
                Path("/opt/MVS/Samples/64/Python"),
                Path("/opt/MVS/Samples/32/Python"),
                Path("/usr/local/MVS"),
            ]
        )

    roots.append(Path(__file__).resolve().parent)

    for root in roots:
        yield root
        for sub in (
            "MvImport",
            "Development/Samples/Python/MvImport",
            "Development/Samples/Python",
            "Samples/64/Python/MvImport",
            "Samples/64/Python",
            "Samples/32/Python/MvImport",
            "Samples/32/Python",
        ):
            yield root / sub


def load_mvs():
    searched = []
    for candidate in _candidate_roots():
        if not candidate.is_dir():
            continue
        searched.append(str(candidate))

        flat = candidate / "MvCameraControl_class.py"
        if flat.is_file():
            if str(candidate) not in sys.path:
                sys.path.insert(0, str(candidate))
            import MvCameraControl_class

            return MvCameraControl_class

        package = candidate / "MvImport" / "MvCameraControl_class.py"
        if package.is_file():
            if str(candidate) not in sys.path:
                sys.path.insert(0, str(candidate))
            if str(candidate / "MvImport") not in sys.path:
                sys.path.insert(0, str(candidate / "MvImport"))
            from MvImport import MvCameraControl_class

            return MvCameraControl_class

    raise RuntimeError(
        "MVS Python wrapper not found. Install the Hikrobot MVS SDK and set "
        "MVS_SDK_PATH to the SDK root or its MvImport directory.\nSearched:\n  "
        + "\n  ".join(searched)
    )
