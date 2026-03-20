"""
Convert all .c/.h files in the project from GBK/GB18030 to UTF-8 with BOM.
UTF-8-BOM (utf-8-sig) is required for Keil ARM Compiler 5 (AC5) to correctly
handle Chinese comments without 'missing closing quote' errors.

Skips: OBJ, Listing, DebugConfig, .git, .vscode (build artifacts / VCS metadata)
Scans: everything else, including FWLIB, FreeRTOS, CORE, MiddleWares
"""

from pathlib import Path

ROOT = Path(__file__).parent
SKIP_DIRS = {'OBJ', 'Listing', 'DebugConfig', '.git', '.vscode'}
EXTENSIONS = {'.c', '.h'}

converted = skipped = errors = 0

for path in sorted(ROOT.rglob('*')):
    if not path.is_file():
        continue
    if path.suffix not in EXTENSIONS:
        continue
    if any(part in SKIP_DIRS for part in path.parts):
        continue

    try:
        content = path.read_bytes()

        # Already UTF-8 (with or without BOM) — skip
        try:
            content.decode('utf-8')
            print(f"SKIP       {path.relative_to(ROOT)}")
            skipped += 1
            continue
        except UnicodeDecodeError:
            pass

        # Force-decode with GB18030 (superset of GBK/GB2312), ignore unrecognised bytes
        text = content.decode('gb18030', errors='ignore')

        # Write back as UTF-8 with BOM (Keil AC5 compatible)
        path.write_bytes(text.encode('utf-8-sig'))
        print(f"CONVERTED  {path.relative_to(ROOT)}")
        converted += 1

    except Exception as e:
        print(f"ERROR      {path.relative_to(ROOT)} — {e}")
        errors += 1

print(f"\nDone: {converted} converted, {skipped} skipped, {errors} errors")
