Import("env")
import subprocess
import sys
from pathlib import Path

def transpile_cnext():
    """Transpile the actuator C-Next module before compilation.

    cnext follows #includes, so transpiling actuator.cnx also generates any
    .cnx it includes (e.g. can_tx_queue.cnx). Generated .cpp land next to the
    .cnx; headers go to include/ via --header-out.
    """
    entry = Path("src/display/actuator.cnx")
    if not entry.exists():
        return

    print("Transpiling actuator.cnx (C-Next)...")
    try:
        result = subprocess.run(
            ["cnext", str(entry), "--header-out", "include"],
            check=True,
            capture_output=True,
            text=True,
        )
        out = result.stdout.strip()
        if out:
            print(out)
    except subprocess.CalledProcessError as e:
        print("  C-Next transpilation FAILED")
        print(e.stderr)
        sys.exit(1)

transpile_cnext()
