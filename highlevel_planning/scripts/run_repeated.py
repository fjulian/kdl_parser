import os
import shutil
import subprocess
import time
from datetime import datetime
from collections import OrderedDict


DATA_DIR = os.path.join(os.path.expanduser("~"), "Data", "highlevel_planning")
SRCROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

NUM_REPETITIONS = 20
MAX_SIMULTANEOUSLY = 5


def hlp_run(individual_index, config_file):
    domain_dir = os.path.join(DATA_DIR, "knowledge", "ScenePlanning2", "main")
    initial_domain_file = os.path.join(domain_dir, "_domain_initial.pkl")
    domain_file_name = f"_domain{individual_index}.pkl"
    domain_file = os.path.join(domain_dir, domain_file_name)

    if os.path.isfile(domain_file):
        os.remove(domain_file)
    shutil.copyfile(initial_domain_file, domain_file, follow_symlinks=False)

    script = os.path.join(SRCROOT, "scripts", "run_auto.py")
    command = [
        "python",
        script,
        "-n",
        "-m",
        "direct",
        "--no-seed",
        "--domain-file",
        domain_file_name,
    ]
    if config_file is not None:
        command.append("--config-file-path")
        command.append(config_file)
    res = subprocess.Popen(
        command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    return res


def mcts_run(individual_index, config_file):
    script = os.path.join(SRCROOT, "scripts", "run_mcts.py")
    command = ["python", script, "-n", "-m", "direct", "--no-seed"]
    if config_file is not None:
        command.append("--config-file-path")
        command.append(config_file)
    res = subprocess.Popen(
        command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    return res


class ExperimentSpec:
    def __init__(self, label: str, method_type: str, config_file: str, start_idx=0):
        assert method_type in ["ours", "mcts"]
        self.method = method_type
        self.config_file = config_file
        self.label = label
        self.start_idx = start_idx


def single_experiment(spec: ExperimentSpec):
    stdout_file = os.path.join(DATA_DIR, "reports", "repeated_stdout.txt")
    num_done, num_started = 0, 0
    file_idx = spec.start_idx
    currently_running = 0
    open_processes = list()
    tic = time.time()
    while num_done < NUM_REPETITIONS:
        if currently_running < MAX_SIMULTANEOUSLY and num_started < NUM_REPETITIONS:
            if spec.method == "mcts":
                p = mcts_run(file_idx, spec.config_file)
            elif spec.method == "ours":
                p = hlp_run(file_idx, spec.config_file)
            else:
                raise ValueError("Undefined method")
            open_processes.append(p)
            num_started += 1
            file_idx += 1
            currently_running += 1
            time.sleep(3)
        else:
            idx = 0
            while True:
                poll_res = open_processes[idx].poll()
                if poll_res is not None:
                    break
                idx += 1
                idx = idx % currently_running
                time.sleep(3)
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
            print(
                f"Currently running: {currently_running}, "
                f"Started: {num_started}/{NUM_REPETITIONS}, "
                f"Done: {num_done}/{NUM_REPETITIONS}"
            )
    print(f"All done. Total time elapsed: {time.time() - tic}")


def move_all_files_in_dir(src_dir, target_dir):
    files = os.listdir(src_dir)
    os.makedirs(target_dir, exist_ok=True)
    for file in files:
        shutil.move(os.path.join(src_dir, file), os.path.join(target_dir, file))
    assert len(os.listdir(src_dir)) == 0


if __name__ == "__main__":
    in_dir = os.path.join(DATA_DIR, "repeated_inputs")
    os.makedirs(in_dir, exist_ok=True)
    out_dir = os.path.join(DATA_DIR, "repeated_outputs")
    os.makedirs(out_dir, exist_ok=True)
    knowledge_dir = os.path.join(DATA_DIR, "knowledge", "ScenePlanning2", "main")
    os.makedirs(knowledge_dir, exist_ok=True)
    report_dir = os.path.join(DATA_DIR, "reports")
    mcts_dir = os.path.join(DATA_DIR, "mcts")

    experiments = [
        ExperimentSpec("[o1]", "ours", os.path.join(in_dir, "config_o1.yaml")),
        ExperimentSpec("[o2]", "ours", os.path.join(in_dir, "config_o2.yaml")),
        ExperimentSpec("[o3]", "ours", os.path.join(in_dir, "config_o3.yaml")),
        ExperimentSpec("[o4]", "ours", os.path.join(in_dir, "config_o4.yaml")),
        ExperimentSpec("[m1]", "mcts", os.path.join(in_dir, "config_m1.yaml")),
    ]
    summary = OrderedDict()
    for spec in experiments:
        # Assert the individual output dirs are empty
        assert len(os.listdir(report_dir)) == 0
        if spec.method == "ours":
            assert len(os.listdir(knowledge_dir)) == 0
        else:
            assert len(os.listdir(mcts_dir)) == 0

        # Put initial domain into place
        if spec.method == "ours":
            shutil.copy(
                os.path.join(in_dir, "_domain_initial.pkl"),
                os.path.join(knowledge_dir, "_domain_initial.pkl"),
            )

        # Run the repetitions
        single_experiment(spec)

        # Store the results
        time_now = datetime.now()
        time_string = time_now.strftime("%y%m%d_%H%M%S")
        out_dir_this_run = os.path.join(out_dir, time_string)
        os.makedirs(out_dir_this_run, exist_ok=False)
        shutil.copy(
            spec.config_file,
            os.path.join(out_dir_this_run, spec.config_file.split("/")[-1]),
        )
        move_all_files_in_dir(report_dir, out_dir_this_run)
        if spec.method == "ours":
            move_all_files_in_dir(knowledge_dir, out_dir_this_run)
        else:
            move_all_files_in_dir(mcts_dir, out_dir_this_run)

        print(
            f"Moved all result files for method {spec.label} to directory {out_dir_this_run}."
        )
        summary[spec.label] = out_dir_this_run

        print("==================================")

    print("Names of the output directories:")
    for label in summary:
        print(f"{label}: {summary[label].split('/')[-1]}")
    print("Bye :)")
