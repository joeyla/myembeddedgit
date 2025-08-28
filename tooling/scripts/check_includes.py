
#!/usr/bin/env python3
import sys, re, pathlib

FORBIDDEN = re.compile(r'#\s*include\s*<stm32|#\s*include\s*\"stm32', re.I)
ROOT = pathlib.Path(__file__).resolve().parents[2]
VIOLATIONS = []

def scan(dirpath):
    for p in pathlib.Path(dirpath).rglob('*.[ch]'):
        txt = p.read_text(errors='ignore')
        if FORBIDDEN.search(txt):
            VIOLATIONS.append(str(p.relative_to(ROOT)))

def main():
    for d in ['core', 'services']:
        scan(ROOT / d)
    if VIOLATIONS:
        print("Forbidden MCU includes in portable code:", *VIOLATIONS, sep="\n- ")
        sys.exit(1)
    print("OK: no MCU includes in core/ or services/")

if __name__ == "__main__":
    main()
