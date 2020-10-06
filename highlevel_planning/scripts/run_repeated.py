import os
import shutil
import subprocess
from tqdm import tqdm

if __name__ == "__main__":
    BASEDIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    num_repetitions = 20

    domain_dir = os.path.join(BASEDIR, "knowledge", "chimera", "main")
    initial_domain_file = os.path.join(domain_dir, "_domain_initial.pkl")
    domain_file = os.path.join(domain_dir, "_domain.pkl")
    stdout_file = os.path.join(BASEDIR, "data", "reports", "repeated_stdout.txt")

    script = os.path.join(BASEDIR, "scripts", "run_auto.py")

    if not os.path.isfile(initial_domain_file):
        raise RuntimeError("Initial domain file needs to be supplied")

    for i in tqdm(range(num_repetitions)):
        if os.path.isfile(domain_file):
            os.remove(domain_file)
        shutil.copyfile(initial_domain_file, domain_file, follow_symlinks=False)

        res = subprocess.run(
            ["python", script, "-n", "-m", "direct", "--no-seed"],
            capture_output=True,
            text=True,
        )
        with open(stdout_file, "a") as f:
            f.write(
                "\n============================================================================\n"
            )
            f.write(
                f"============================== Repetition {i+1} ==============================\n"
            )
            f.write(
                "------------------------------- stdout -------------------------------\n"
            )
            f.write(res.stdout)
            f.write(
                "------------------------------- stderr -------------------------------\n"
            )
            f.write(res.stderr)
