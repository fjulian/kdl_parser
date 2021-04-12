import os
import shutil
import subprocess
import time

from tqdm import tqdm

DATA_DIR = os.path.join(os.path.expanduser("~"), "Data", "highlevel_planning")
SRCROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def hlp_run():
    domain_dir = os.path.join(DATA_DIR, "knowledge", "ScenePlanning1", "main")
    initial_domain_file = os.path.join(domain_dir, "_domain_initial.pkl")
    domain_file = os.path.join(domain_dir, "_domain.pkl")

    if os.path.isfile(domain_file):
        os.remove(domain_file)
    shutil.copyfile(initial_domain_file, domain_file, follow_symlinks=False)

    script = os.path.join(SRCROOT, "scripts", "run_auto.py")
    res = subprocess.run(
        ["python", script, "-n", "-m", "direct", "--no-seed"],
        capture_output=True,
        text=True,
    )
    return res


def mcts_run():
    script = os.path.join(SRCROOT, "scripts", "run_mcts.py")
    res = subprocess.Popen(
        ["python", script, "-n", "-m", "direct", "--no-seed"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return res


if __name__ == "__main__":
    num_repetitions = 20

    stdout_file = os.path.join(DATA_DIR, "reports", "repeated_stdout.txt")

    num_done, num_started = 0, 0
    currently_running = 0
    max_simultaneously = 8
    open_processes = list()
    tic = time.time()
    while num_done < num_repetitions:
        if currently_running < max_simultaneously and num_started < num_repetitions:
            p = mcts_run()
            open_processes.append(p)
            num_started += 1
            currently_running += 1
            time.sleep(5)
        else:
            idx = 0
            while True:
                poll_res = open_processes[idx].poll()
                if poll_res is not None:
                    break
                idx += 1
                idx = idx % currently_running
                time.sleep(4)
            p = open_processes.pop(idx)
            out, err = p.communicate()
            with open(stdout_file, "a") as f:
                f.write(
                    "\n============================================================================\n"
                )
                f.write(
                    f"============================== Repetition {num_done + 1} ==============================\n"
                )
                f.write(
                    "------------------------------- stdout -------------------------------\n"
                )
                f.write(out)
                f.write(
                    "------------------------------- stderr -------------------------------\n"
                )
                f.write(err)
            num_done += 1
            currently_running -= 1
        print("----------------")
        print(f"Currently running: {currently_running}")
        print(f"Started: {num_started}/{num_repetitions}")
        print(f"Done: {num_done}/{num_repetitions}")
    print("==================================")
    print(f"All done. Total time elapsed: {time.time()-tic}")
