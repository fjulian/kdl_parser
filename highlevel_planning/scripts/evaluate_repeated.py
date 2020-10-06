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
    num_seq_samples = list()
    num_preplan_exec_success = list()
    num_plan_exec_success = list()
    time_sampling = list()
    time_sequence_completion = list()
    time_execution = list()
    time_domain_extension = list()
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
            num_seq_samples.append(metrics[f"{label_prefix}_#_valid_sequences"])
            num_preplan_exec_success.append(
                metrics[f"{label_prefix}_#_preplan_success"]
            )
            num_plan_exec_success.append(metrics[f"{label_prefix}_#_plan_success"])
            time_sampling.append(metrics[f"{label_prefix}_t_sampling"])
            time_sequence_completion.append(
                metrics[f"{label_prefix}_t_sequence_completion"]
            )
            time_execution.append(metrics[f"{label_prefix}_t_execution"])
            time_domain_extension.append(metrics[f"{label_prefix}_t_domain_extension"])
    mean_num_seq_samples = np.mean(num_seq_samples)
    mean_num_preplan_exec_success = np.mean(num_preplan_exec_success)
    mean_num_plan_exec_success = np.mean(num_plan_exec_success)
    mean_time_sampling = np.mean(time_sampling)
    mean_time_sequence_completion = np.mean(time_sequence_completion)
    mean_time_execution = np.mean(time_execution)
    mean_time_domain_extension = np.mean(time_domain_extension)
    return num_seq_samples


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
    experiment_string = ["201006-122200", "201006-143900"]
    titles = ["(a)", "(b)"]
    all_data = list()
    for i in range(len(experiment_string)):
        this_data = summarize_experiment(basedir, experiment_string[i])
        # this_data = np.array(this_data).reshape((-1,1))
        all_data.append(this_data)

    fig1, ax1 = plt.subplots()
    # ax1.set_title("Number of sampled sequences")
    ax1.boxplot(all_data, vert=False, labels=titles, notch=True)
    plt.xlabel("Number of sampled sequences [-]")
    plt.grid()
    plt.show()
