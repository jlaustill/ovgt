Import("env")
import subprocess
import sys
from pathlib import Path

def transpile_cnext():
    """Transpile the C-Next modules before compilation.

    cnext follows #includes, so transpiling an entry also generates any
    .cnx it includes (e.g. can_tx_queue.cnx). Generated .c/.cpp land next
    to the .cnx; headers go to include/ via --header-out.
    """
    entries = [
        Path("src/display/actuator.cnx"),
        Path("src/domain/json.cnx"),
        Path("src/domain/systemHealthLogic.cnx"),
        Path("src/domain/systemHealth.cnx"),
    ]
    for entry in entries:
        if not entry.exists():
            continue

        print(f"Transpiling {entry.name} (C-Next)...")
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
            print(f"  C-Next transpilation FAILED for {entry.name}")
            print(e.stderr)
            sys.exit(1)

transpile_cnext()
