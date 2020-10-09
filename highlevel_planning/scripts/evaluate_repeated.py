import pickle
import os
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime


def sum_metrics(metrics, label_prefixes, label_string):
    return np.sum(
        [metrics[f"{label_prefix}_{label_string}"] for label_prefix in label_prefixes]
    )


def summarize_experiment(base_dir, name_string, success_label):
    raw_data_dir = os.path.join(base_dir, name_string)
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

    num_runs = len(files)
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
        ]
    )
    for key in summary_data:
        summary_data[key] = list()
    for i in range(num_runs):
        with open(os.path.join(raw_data_dir, files[i]), "rb") as f:
            data = pickle.load(f)
        metrics = data["metrics"]
        success = False
        this_success_labels = list()
        for key in metrics:
            found_success_label = [key.find(l) > -1 for l in success_label]
            if np.all(np.array(found_success_label)):
                this_success_labels.append(key)
                if metrics[key]:
                    success = True
                    success_count += 1
                    break
        if success:
            label_prefixes = [
                label[: -(len("_found_plan"))] for label in this_success_labels
            ]

            summary_data["num_seq_samples"].append(
                sum_metrics(metrics, label_prefixes, "#_valid_sequences")
            )
            summary_data["num_preplan_exec_success"].append(
                sum_metrics(metrics, label_prefixes, "#_preplan_success")
            )
            summary_data["num_plan_exec_success"].append(
                sum_metrics(metrics, label_prefixes, "#_plan_success")
            )
            summary_data["time_sampling"].append(
                sum_metrics(metrics, label_prefixes, "t_sampling")
            )
            summary_data["time_sequence_completion"].append(
                sum_metrics(metrics, label_prefixes, "t_sequence_completion")
            )
            summary_data["time_execution"].append(
                sum_metrics(metrics, label_prefixes, "t_execution")
            )
            summary_data["time_domain_extension"].append(
                sum_metrics(metrics, label_prefixes, "t_domain_extension")
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
    experiment_string = [
        "201006-122200",
        "201006-143900",
        "201006-193900",
        # "201008-125800",
        # "201008-133700",
        "201009-104000",
    ]
    experiment_success_label = ["found_plan"]
    titles = ["(a)", "(b)", "(c)", "(d)"]
    num_seq_samples_data = list()
    for i in range(len(experiment_string)):
        summary, success_cnt = summarize_experiment(
            basedir, experiment_string[i], experiment_success_label
        )
        print("==================================================")
        print(f"Experiment ID: {experiment_string[i]}")
        print(f"Success count: {success_cnt}")
        for key in summary:
            print(f"{key} mean (std): {np.mean(summary[key])} ({np.std(summary[key])})")
        num_seq_samples_data.append(summary["num_seq_samples"])

    # Font sizes
    parameters = {
        "axes.labelsize": 18,
        # "axes.titlesize": 25,
        "xtick.labelsize": 15,
        "ytick.labelsize": 15,
    }
    plt.rcParams.update(parameters)

    fig1, ax1 = plt.subplots(figsize=(7, 3))
    # ax1.set_title("Number of sampled sequences")
    ax1.boxplot(
        list(reversed(num_seq_samples_data)),
        vert=False,
        labels=list(reversed(titles)),
        notch=False,
        widths=0.65,
    )
    # ax1.set_aspect(20)
    plt.xlabel("Number of sampled sequences [-]")
    plt.ylabel("Experiment ID")
    plt.grid()

    plt.show()

    time_now = datetime.now()
    time_string = time_now.strftime("%y%m%d-%H%M%S")

    fig1.savefig(
        os.path.join(basedir, "Output", f"{time_string}_boxplot.pdf"),
        bbox_inches="tight",
    )
