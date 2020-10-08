import pickle
import os
import numpy as np
import matplotlib.pyplot as plt


def summarize_experiment(basedir, name_string):
    raw_data_dir = os.path.join(basedir, name_string, "Data")
    files = os.listdir(raw_data_dir)

    files.sort()
    rm_idx = list()
    for idx, f in enumerate(files):
        if not f.endswith(".pkl"):
            rm_idx.append(idx)
        elif not f.startswith("20"):
            rm_idx.append(idx)
    rm_idx.sort(reverse=True)
    for idx in rm_idx:
        del files[idx]

    with open(os.path.join(raw_data_dir, files[0]), "rb") as f:
        data = pickle.load(f)

    num_runs = len(files)
    success_label = ["found_plan"]
    success_count = 0
    summary_data = dict.fromkeys(
        [
            "num_seq_samples",
            "num_preplan_exec_success",
            "num_plan_exec_success",
            "time_sampling",
            "time_sequence_completion",
            "time_execution",
            "time_domain_extension",
        ],
        [],
    )
    for i in range(num_runs):
        with open(os.path.join(raw_data_dir, files[i]), "rb") as f:
            data = pickle.load(f)
        metrics = data["metrics"]
        success = False
        this_success_label = None
        for key in metrics:
            found_success_label = [key.find(l) > -1 for l in success_label]
            if np.all(np.array(found_success_label)) and metrics[key]:
                success = True
                this_success_label = key
                success_count += 1
                break
        if success:
            label_prefix = this_success_label[: -(len("_found_plan"))]
            summary_data["num_seq_samples"].append(
                metrics[f"{label_prefix}_#_valid_sequences"]
            )
            summary_data["num_preplan_exec_success"].append(
                metrics[f"{label_prefix}_#_preplan_success"]
            )
            summary_data["num_plan_exec_success"].append(
                metrics[f"{label_prefix}_#_plan_success"]
            )
            summary_data["time_sampling"].append(metrics[f"{label_prefix}_t_sampling"])
            summary_data["time_sequence_completion"].append(
                metrics[f"{label_prefix}_t_sequence_completion"]
            )
            summary_data["time_execution"].append(
                metrics[f"{label_prefix}_t_execution"]
            )
            summary_data["time_domain_extension"].append(
                metrics[f"{label_prefix}_t_domain_extension"]
            )
    return summary_data, success_count


if __name__ == "__main__":
    basedir = os.path.join(
        os.getenv("HOME"),
        "Polybox",
        "PhD",
        "Publications",
        "2021 ICRA",
        "Experiments",
        "Repeated",
    )
    experiment_string = ["201006-122200", "201006-143900", "201006-193900"]
    titles = ["(a)", "(b)", "(c)"]
    num_seq_samples_data = list()
    for i in range(len(experiment_string)):
        summary, success_cnt = summarize_experiment(basedir, experiment_string[i])
        print("==================================================")
        print(f"Experiment ID: {experiment_string[i]}")
        print(f"Success count: {success_cnt}")
        print(f"Mean num seq samples: {np.mean(summary['num_seq_samples'])}")
        num_seq_samples_data.append(summary["num_seq_samples"])

    # Font sizes
    parameters = {
        "axes.labelsize": 18,
        # "axes.titlesize": 25,
        "xtick.labelsize": 15,
        "ytick.labelsize": 15,
    }
    plt.rcParams.update(parameters)

    fig1, ax1 = plt.subplots(figsize=(7, 4))
    # ax1.set_title("Number of sampled sequences")
    ax1.boxplot(
        num_seq_samples_data, vert=False, labels=titles, notch=False, widths=0.65
    )
    # ax1.set_aspect(20)
    plt.xlabel("Number of sampled sequences [-]")
    plt.ylabel("Experiment ID")
    plt.grid()

    plt.show()
