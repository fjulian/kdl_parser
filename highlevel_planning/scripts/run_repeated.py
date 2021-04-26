import os
import shutil
import subprocess
import time
import argparse


DATA_DIR = os.path.join(os.path.expanduser("~"), "Data", "highlevel_planning")
SRCROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

NUM_REPETITIONS = 20
MAX_SIMULTANEOUSLY = 6


def hlp_run(individual_index):
    domain_dir = os.path.join(DATA_DIR, "knowledge", "ScenePlanning2", "main")
    initial_domain_file = os.path.join(domain_dir, "_domain_initial.pkl")
    domain_file_name = f"_domain{individual_index}.pkl"
    domain_file = os.path.join(domain_dir, domain_file_name)

    if os.path.isfile(domain_file):
        os.remove(domain_file)
    shutil.copyfile(initial_domain_file, domain_file, follow_symlinks=False)

    script = os.path.join(SRCROOT, "scripts", "run_auto.py")
    res = subprocess.Popen(
        [
            "python",
            script,
            "-n",
            "-m",
            "direct",
            "--no-seed",
            "--domain-file",
            domain_file_name,
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return res


def mcts_run(individual_index):
    script = os.path.join(SRCROOT, "scripts", "run_mcts.py")
    res = subprocess.Popen(
        ["python", script, "-n", "-m", "direct", "--no-seed"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return res


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "method",
        action="store",
        help="choose which method to evaluate. Choice between 'mcts' and 'ours'",
    )
    args = parser.parse_args()

    stdout_file = os.path.join(DATA_DIR, "reports", "repeated_stdout.txt")
    num_done, num_started = 0, 0
    currently_running = 0
    open_processes = list()
    tic = time.time()
    while num_done < NUM_REPETITIONS:
        if currently_running < MAX_SIMULTANEOUSLY and num_started < NUM_REPETITIONS:
            if args.method == "mcts":
                p = mcts_run(num_started)
            elif args.method == "ours":
                p = hlp_run(num_started)
            else:
                raise ValueError("Undefined method")
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
        print(f"Started: {num_started}/{NUM_REPETITIONS}")
        print(f"Done: {num_done}/{NUM_REPETITIONS}")
    print("==================================")
    print(f"All done. Total time elapsed: {time.time()-tic}")
