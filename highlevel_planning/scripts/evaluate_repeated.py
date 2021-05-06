import pickle
import os
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import pandas as pd
import seaborn as sns

print_human_readable = False


def sum_metrics(metrics, label_prefixes, label_string):
    return np.sum(
        [metrics[f"{label_prefix}_{label_string}"] for label_prefix in label_prefixes]
    )


def number_formatter(number):
    if number >= 0.1:
        return f"{number:.2f}"
    else:
        return f"{number:.1e}"


def filter_data_files(files):
    files.sort()
    rm_idx = list()
    for idx, f in enumerate(files):
        if not f.endswith(".pkl"):
            rm_idx.append(idx)
        elif not f.startswith("2"):
            rm_idx.append(idx)
    rm_idx.sort(reverse=True)
    for idx in rm_idx:
        del files[idx]


def summarize_experiment(base_dir, name_string, success_label):
    raw_data_dir = os.path.join(base_dir, name_string)
    files = os.listdir(raw_data_dir)
    filter_data_files(files)

    num_runs = len(files)
    success_count = 0
    summary_data = dict.fromkeys(
        [
            "file_name",
            "num_seq_samples",
            "num_preplan_exec_success",
            "num_plan_exec_success",
            "time_sampling",
            "time_sequence_completion",
            "time_execution",
            "time_domain_extension",
            "time_total",
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

            summary_data["file_name"].append(files[i])
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
            summary_data["time_total"].append(
                sum_metrics(metrics, label_prefixes, "total_time")
            )
    return summary_data, success_count


def summarize_mcts_experiment(base_dir, name_string, success_label):
    raw_data_dir = os.path.join(base_dir, name_string)
    files = os.listdir(raw_data_dir)
    filter_data_files(files)

    num_runs = len(files)
    success_count = 0
    summary_data = dict.fromkeys(
        [
            "file_name",
            "max_depth",
            "num_restarts",
            "num_expand",
            "time_check_expanding",
            "time_select_child",
            "time_expand",
            "time_sim",
            "time_sample_feasible",
            "time_total",
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
            summary_data["file_name"].append(files[i])
            summary_data["max_depth"].append(metrics["max_depth"])
            summary_data["num_restarts"].append(metrics["num_restarts"])
            summary_data["num_expand"].append(metrics["num_expand"])
            summary_data["time_check_expanding"].append(metrics["time_check_expanding"])
            summary_data["time_select_child"].append(metrics["time_select_child"])
            summary_data["time_expand"].append(metrics["time_expand"])
            summary_data["time_sim"].append(metrics["time_sim"])
            summary_data["time_sample_feasible"].append(metrics["time_sample_feasible"])
            summary_data["time_total"].append(metrics["time_until_result"])
    return summary_data, success_count


def main_evaluate_hlp():
    basedir = os.path.join(
        os.getenv("HOME"),
        "Polybox",
        "PhD",
        "Publications",
        "2021 ICRA HLP",
        "Second try",
        "Experiments",
    )
    experiment_string = ["210423_110900"]
    experiment_success_label = ["found_plan"]
    titles = ["(a)", "(b)", "(c)", "(d)", "(e)"]
    num_seq_samples_data = list()
    for i in range(len(experiment_string)):
        summary, success_cnt = summarize_experiment(
            basedir, experiment_string[i], experiment_success_label
        )
        if print_human_readable:
            print("==================================================")
            print(f"Experiment ID: {experiment_string[i]}")
            print(f"Success count: {success_cnt}")
            for key in summary:
                print(
                    f"{key} [median] mean (std): [{np.median(summary[key])}] {np.mean(summary[key])} ({np.std(summary[key])})"
                )

            print("------------------------------------------")
        funcs = [np.median, np.mean, np.std]
        starts = [
            f"\\multirow{{3}}{{*}}{{{titles[i]}}} & \\multirow{{3}}{{*}}{{{success_cnt}}} & $m$ & ",
            f"& & $\\mu$ & ",
            f"& & $\\sigma$ & ",
        ]
        for j in range(len(funcs)):
            print(starts[j], end="")
            print(
                f"\\num{{{number_formatter(funcs[j](summary['num_seq_samples']))}}} & \\num{{{number_formatter(funcs[j](summary['num_plan_exec_success']))}}}"
                f" & \\num{{{number_formatter(funcs[j](summary['time_sampling']))}}} & \\num{{{number_formatter(funcs[j](summary['time_sequence_completion']))}}} & "
                f"\\num{{{number_formatter(funcs[j](summary['time_execution']))}}} & \\num{{{number_formatter(funcs[j](summary['time_domain_extension']))}}} \\\\"
            )
        print("\\hline")
        # & & $\sigma$ & \num{21.26} & \num{8.02} & \num{1.1e-2} & \num{1.04} & \num{4.79} & \num{8.1e-6} \\
        # \hline")
        num_seq_samples_data.append(summary["num_seq_samples"])

    # Font sizes
    parameters = {
        "axes.labelsize": 18,
        # "axes.titlesize": 25,
        "xtick.labelsize": 15,
        "ytick.labelsize": 15,
    }
    plt.rcParams.update(parameters)

    fig1, ax1 = plt.subplots(figsize=(7, 3.5))
    # ax1.set_title("Number of sampled sequences")
    ax1.boxplot(
        list(reversed(num_seq_samples_data)),
        vert=False,
        labels=list(reversed(titles)),
        notch=False,
        widths=0.7,
    )
    # ax1.set_aspect(20)
    plt.xlabel("Number of sampled sequences [-]")
    plt.ylabel("Experiment ID")
    plt.grid()

    plt.show()

    time_now = datetime.now()
    time_string = time_now.strftime("%y%m%d-%H%M%S")

    # fig1.savefig(
    #     os.path.join(basedir, "Output", f"{time_string}_boxplot.pdf"),
    #     bbox_inches="tight",
    # )


def compare_hlp_mcts():
    basedir = os.path.join(
        os.getenv("HOME"),
        "Polybox",
        "PhD",
        "Publications",
        "2021 ICRA HLP",
        "Second try",
        "Experiments",
    )
    method_strings = ["ours, w/ alt", "ours, w/o alt", "ours, demo", "mcts"]
    experiment_strings = {
        # cube on cupboard
        "(a)": {
            method_strings[0]: "210426_093200",
            method_strings[1]: "210426_093600",
            method_strings[3]: "210426_093900",
        },
        # box on shelf
        "(b)": {
            method_strings[0]: "210426_103100",
            method_strings[1]: "210426_110600",
            method_strings[3]: "210426_113200",
        },
        # cube in container (w/lid)
        "(c)": {
            method_strings[0]: "210423_181300",
            method_strings[1]: "210426_075600",
            method_strings[2]: "210427_142300",
            method_strings[3]: "210426_082000",
        },
        # cube from container to container (w/lids)
        "(d)": {
            method_strings[0]: "210427_130200",
            method_strings[1]: "210427_141400",
            method_strings[2]: "210427_101600",
            method_strings[3]: "210426_190200",
        },
    }
    plot_data = pd.DataFrame(columns=["experiment", "method", "total_time"])
    table_combined_str = ""
    table_ours_str = ""
    table_mcts_str = ""
    funcs = [np.median, np.mean, np.std]
    for experiment_id, methods in experiment_strings.items():
        table_data = dict()
        for method in method_strings:
            table_data[method] = dict.fromkeys(["success_cnt", "summary"], None)
            if method not in methods:
                continue
            experiment_string = methods[method]

            if "ours" in method:
                experiment_success_label = ["found_plan"]
                summary, success_cnt = summarize_experiment(
                    basedir, experiment_string, experiment_success_label
                )
            else:
                assert "mcts" in method
                experiment_success_label = ["success"]
                summary, success_cnt = summarize_mcts_experiment(
                    basedir, experiment_string, experiment_success_label
                )

            # Plot data
            new_rows = [
                [experiment_id, method, summary["time_total"][sum_idx]]
                for sum_idx in range(success_cnt)
            ]
            new_rows = pd.DataFrame(
                new_rows, columns=["experiment", "method", "total_time"]
            )
            plot_data = plot_data.append(new_rows)

            # Table data
            table_data[method]["success_cnt"] = success_cnt
            table_data[method]["summary"] = summary

        # Combined table
        table_combined_str += f"\\multirow{{4}}{{*}}{{{experiment_id}}} & \\multicolumn{{2}}{{c|}}{{Success count [-]}} "
        for method in method_strings:
            if table_data[method]["success_cnt"] is None:
                table_combined_str += "& "
            else:
                table_combined_str += f"& \\num{{{table_data[method]['success_cnt']}}} "
        table_combined_str += "\\\\\n"
        starts_combined = [
            f"& \\multirow{{3}}{{*}}{{Total time [s]}} & $m$ ",
            f"& & $\\mu$ ",
            f"& & $\\sigma$ ",
        ]
        starts_individual = [
            f"\\multirow{{3}}{{*}}{{{experiment_id}}} & $m$ ",
            f"& $\\mu$ ",
            f"& $\\sigma$ ",
        ]
        for j in range(len(funcs)):
            table_combined_str += starts_combined[j]
            for method in method_strings:
                if (
                    table_data[method]["success_cnt"] is None
                    or table_data[method]["success_cnt"] == 0
                ):
                    table_combined_str += "& "
                else:
                    table_combined_str += f" & \\num{{{number_formatter(funcs[j](table_data[method]['summary']['time_total']))}}} "
            table_combined_str += "\\\\\n"

            # table_ours_str += starts_individual[j]
            # table_ours_str += f"& \\num{{{number_formatter(funcs[j](summary['num_seq_samples']))}}}"
        table_combined_str += "\\hline\n"
    print(table_combined_str)

    # Plot timing data
    fig1, ax1 = plt.subplots(figsize=(7, 3.5))
    sns.boxplot(
        x="experiment",
        y="total_time",
        hue="method",
        data=plot_data,
        dodge=True,
        palette="Paired",
        ax=ax1,
    )
    plt.show()

    # Plot success data
    fig2, ax2 = plt.subplots(figsize=(7, 4))
    sns.countplot(x="experiment", hue="method", data=plot_data, ax=ax2)
    plt.show()

    # Save figures
    # time_now = datetime.now()
    # time_string = time_now.strftime("%y%m%d-%H%M%S")
    # fig1.savefig(
    #     os.path.join(basedir, "Output", f"{time_string}_boxplot.pdf"),
    #     bbox_inches="tight",
    # )
    # fig2.savefig(
    #     os.path.join(basedir, "Output", f"{time_string}_barplot.pdf"),
    #     bbox_inches="tight",
    # )


if __name__ == "__main__":
    compare_hlp_mcts()
