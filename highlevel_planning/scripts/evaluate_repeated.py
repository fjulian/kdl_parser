import pickle
import os
from copy import deepcopy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from datetime import datetime
import pandas as pd
import seaborn as sns
from collections import defaultdict

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


def summarize_experiment(base_dir, file_name, method_string, experiment_type):
    raw_data_dir = os.path.join(base_dir, file_name)
    files = os.listdir(raw_data_dir)
    filter_data_files(files)

    num_runs = len(files)
    new_df = pd.DataFrame()
    for i in range(num_runs):
        with open(os.path.join(raw_data_dir, files[i]), "rb") as f:
            data = pickle.load(f)
        metrics = data["metrics"]
        metrics = metrics["explore_0"]["counters"]

        successes = list()
        total_times = list()
        times = defaultdict(list)
        counts = defaultdict(list)
        for method_name, method_metrics in metrics.items():
            if "config" in method_name:
                continue

            total_times.append(method_metrics["total_time"])
            if "found_plan" in method_metrics:
                successes.append(method_metrics["found_plan"])
                for metric_label in method_metrics:
                    if metric_label.startswith("t_"):
                        time_sum = 0.0
                        for _, individual_time in method_metrics[metric_label].items():
                            time_sum += individual_time
                        times[metric_label].append(time_sum)
                    elif metric_label.startswith("#_"):
                        count_sum = 0
                        for _, individual_count in method_metrics[metric_label].items():
                            count_sum += individual_count
                        counts[metric_label].append(count_sum)

        new_row_data = dict()

        # Common fields
        new_row_data["method"] = [method_string]
        new_row_data["experiment"] = [experiment_type]
        new_row_data["success"] = [np.any(successes)]
        new_row_data["total_time"] = [np.sum(total_times)]

        # Method specific fields
        for label in times:
            new_row_data[label] = [np.sum(times[label])]
        for label in counts:
            new_row_data[label] = [np.sum(counts[label])]

        # Timing comparison
        tmp_sum = 0.0
        tmp_summand = (
            new_row_data["t_sampling"][0] + new_row_data["t_sequence_completion"][0]
        )
        tmp_sum += tmp_summand
        new_row_data["t_comp_sampling"] = [tmp_summand]
        tmp_summand = new_row_data["t_execution"][0]
        tmp_sum += tmp_summand
        new_row_data["t_comp_simulating"] = [tmp_summand]
        tmp_summand = (
            new_row_data["t_sequence_refinement"][0]
            + new_row_data["t_domain_extension"][0]
        )
        tmp_sum += tmp_summand
        new_row_data["t_comp_domain_extension"] = [tmp_summand]
        new_row_data["t_comp_other"] = [new_row_data["total_time"][0] - tmp_sum]
        assert new_row_data["t_comp_other"][0] > 0

        new_row = pd.DataFrame(new_row_data)
        new_df = new_df.append(new_row)
    return new_df


def summarize_mcts_experiment(base_dir, name_string, method_string, experiment_type):
    raw_data_dir = os.path.join(base_dir, name_string)
    files = os.listdir(raw_data_dir)
    filter_data_files(files)

    num_runs = len(files)
    new_df = pd.DataFrame()
    for i in range(num_runs):
        with open(os.path.join(raw_data_dir, files[i]), "rb") as f:
            data = pickle.load(f)
        metrics = data["metrics"]

        new_row_data = dict()

        # Common fields
        new_row_data["method"] = [method_string]
        new_row_data["experiment"] = [experiment_type]
        new_row_data["success"] = [metrics["success"]]
        new_row_data["total_time"] = [metrics["time_until_result"]]

        tmp_sum = 0.0
        tmp_summand = (
            metrics["time_check_expanding"]
            + metrics["time_select_child"]
            + metrics["time_sample_feasible"]
        )
        tmp_sum += tmp_summand
        new_row_data["t_comp_sampling"] = [tmp_summand]
        tmp_summand = metrics["time_sim"]
        tmp_sum += tmp_summand
        new_row_data["t_comp_simulating"] = [tmp_summand]
        new_row_data["t_comp_domain_extension"] = [0.0]
        new_row_data["t_comp_other"] = [new_row_data["total_time"][0] - tmp_sum]
        assert new_row_data["t_comp_other"][0] > 0

        # Method specific fields
        counters = [
            "max_depth",
            "num_restarts",
            "num_expand",
            "time_check_expanding",
            "time_select_child",
            "time_expand",
            "time_sim",
            "time_sample_feasible",
            "num_expand_tries",
        ]
        for counter in counters:
            new_row_data[counter] = [metrics[counter]]
        new_row = pd.DataFrame(new_row_data)
        new_df = new_df.append(new_row)
    return new_df


def compare_hlp_mcts():
    basedir = os.path.join(
        os.getenv("HOME"),
        "Polybox",
        "PhD",
        "Publications",
        "2021 ICRA HLP",
        "Second try",
        "quantitative_experiments",
    )
    method_strings = [
        "ours, alternating",
        "ours, no alternating",
        "ours, full length",
        "ours, from demonstration",
        "MCTS",
    ]
    experiment_strings = {
        # cube on cupboard
        "(a)": {
            method_strings[0]: "210520_164800",
            method_strings[1]: "210520_165100",
            method_strings[2]: "210520_170400",
            method_strings[3]: "210521_151500",
            method_strings[4]: "210521_150300",
        },
        # box on shelf
        "(b)": {
            method_strings[0]: "210521_192344",
            method_strings[1]: "210521_194355",
            method_strings[2]: "210521_201857",
            method_strings[3]: "210524_170708",
            method_strings[4]: "210524_175907",
        },
        # cube in container (w/lid)
        "(c1)": {
            method_strings[0]: "210524_193859",
            method_strings[1]: "210524_195423",
            method_strings[2]: "210524_200445",
            method_strings[3]: "210524_230812",
            method_strings[4]: "210524_213307",
        },
        # duck in container (knowledge from (c1), same container)
        "(c2)": {
            method_strings[0]: "210525_075055",
            method_strings[1]: "210525_075841",
            method_strings[2]: "210525_081117",
            method_strings[3]: "210525_081557",
            method_strings[4]: "210525_084416",
        },
        "(c3)": {
            method_strings[0]: "210525_121109",
            method_strings[1]: "210525_121706",
            method_strings[2]: "210525_122325",
            method_strings[3]: "210525_122840",
            method_strings[4]: "210525_125842",
        },
        "(c4)": {
            method_strings[0]: "210525_140616",
            method_strings[1]: "210525_141823",
            method_strings[2]: "210525_142756",
            method_strings[3]: "210525_143623",
            method_strings[4]: "210525_150115",
        },
        # cube from container to container (w/lids)
        "(d1)": {
            method_strings[0]: "210525_000737",
            method_strings[1]: "210525_005854",
            method_strings[2]: "210525_013438",
            method_strings[3]: "210525_015642",
            method_strings[4]: "210525_025227",
        },
        # duck from container to container (w/lids)
        "(d2)": {
            method_strings[0]: "210609_163459",
            method_strings[1]: "210609_164141",
            method_strings[2]: "210609_165015",
            method_strings[3]: "210609_175538",
            method_strings[4]: "210609_185741",
        },
    }
    plot_data = pd.DataFrame(columns=["experiment", "method", "total_time"])
    table_combined_str = ""
    table_ours_str = ""
    table_mcts_str = ""
    funcs = [np.median, np.mean, np.std]
    counts = dict()
    for experiment_label, methods in experiment_strings.items():
        table_data = dict()
        for method in method_strings:
            table_data[method] = {
                key: None for key in ["success_cnt", "timeout_cnt", "total_times"]
            }
            if method not in methods:
                continue
            experiment_id = methods[method]

            if "ours" in method:
                new_data = summarize_experiment(
                    basedir, experiment_id, method, experiment_label
                )
            else:
                assert "mcts" in method.lower()
                new_data = summarize_mcts_experiment(
                    basedir, experiment_id, method, experiment_label
                )

            if len(new_data) == 0:
                continue

            # Plot data
            plot_data = plot_data.append(new_data)

            # Table data
            table_data[method]["success_cnt"] = new_data.success.value_counts()[True]
            table_data[method]["timeout_cnt"] = (
                len(new_data) - table_data[method]["success_cnt"]
            )
            table_data[method]["total_times"] = new_data.total_time.to_numpy()

        counts[experiment_label] = table_data

        # Combined table
        table_combined_str += f"\\multirow{{4}}{{*}}{{{experiment_label}}} & \\multicolumn{{2}}{{c|}}{{Success count [-]}} "
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
            f"\\multirow{{3}}{{*}}{{{experiment_label}}} & $m$ ",
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
                    table_combined_str += f" & \\num{{{number_formatter(funcs[j](table_data[method]['total_times']))}}} "
            table_combined_str += "\\\\\n"

            # table_ours_str += starts_individual[j]
            # table_ours_str += f"& \\num{{{number_formatter(funcs[j](summary['num_seq_samples']))}}}"
        table_combined_str += "\\hline\n"
    print(table_combined_str)

    # Plot timing data
    plot_timing = True
    color_palette = ["#007F5F", "#55A630", "#AACC00", "#D4D700", "#FF006E"]
    color_palette_lighter = ["#21FFC8", "#98D87A", "#E1FF4E", "#FCFF53", "#FF6BAB"]
    if plot_timing:
        fig1, ax1 = plt.subplots(figsize=(12, 3.5))
        sns.boxplot(
            x="experiment",
            y="total_time",
            hue="method",
            hue_order=method_strings,
            data=plot_data[plot_data["success"] == True],
            dodge=True,
            palette=color_palette,
            ax=ax1,
        )
        label_dist = 0.33
        label_offsets = np.arange(
            -label_dist,
            label_dist + 2 * label_dist / (len(method_strings) - 1),
            2 * label_dist / (len(method_strings) - 1),
        )
        for i in range(len(experiment_strings)):
            for j in range(len(method_strings)):
                pos_x = i + label_offsets[j]
                pos_y = 960
                tmp = list(experiment_strings.keys())
                tmt_cnt = counts[tmp[i]][method_strings[j]]["timeout_cnt"]
                label_text = f"{tmt_cnt}" if tmt_cnt is not None else ""
                plt.text(
                    pos_x,
                    pos_y,
                    label_text,
                    horizontalalignment="center",
                    color=color_palette[j],
                )
        lgd1 = plt.legend(
            bbox_to_anchor=(0.0, 1.02, 1.0, 0.102),
            loc="lower left",
            ncol=3,
            mode="expand",
            borderaxespad=0.0,
        )
        vline_pos = np.arange(0.5, len(experiment_strings) - 0.5, 1.0)
        plt.vlines(vline_pos, ymin=0, ymax=900, colors="gray", linestyles="dashed")
        plt.ylim([-20, 1030])
        plt.xlim([-0.5, len(experiment_strings) - 1 + 0.5])
        plt.xlabel("Experiment ID")
        plt.ylabel("Time until result [s]")
        plt.show()

    # Plot success data
    plot_success = False
    if plot_success:
        fig2, ax2 = plt.subplots(figsize=(8, 4))
        sns.countplot(
            x="experiment",
            hue="method",
            hue_order=method_strings,
            data=plot_data[plot_data["success"] == True],
            palette=color_palette,
            ax=ax2,
        )
        lgd2 = plt.legend(
            bbox_to_anchor=(0.0, 1.02, 1.0, 0.102),
            loc="lower left",
            ncol=3,
            mode="expand",
            borderaxespad=0.0,
        )
        plt.show()

    # Plot how time is spent
    grouped_data = (
        plot_data[plot_data["success"] == True].groupby(["experiment", "method"]).mean()
    )
    grouped_data.reset_index(inplace=True)

    grouped_data["t_comp_all_wo_sim"] = (
        grouped_data["t_comp_sampling"]
        + grouped_data["t_comp_domain_extension"]
        + grouped_data["t_comp_other"]
    )
    grouped_data["t_comp_percent_simulating"] = (
        grouped_data["t_comp_simulating"]
        / (grouped_data["t_comp_all_wo_sim"] + grouped_data["t_comp_simulating"])
        * 100
    )
    grouped_data["t_comp_percent_all_wo_sim"] = (
        grouped_data["t_comp_all_wo_sim"]
        / (grouped_data["t_comp_all_wo_sim"] + grouped_data["t_comp_simulating"])
        * 100
    )
    grouped_data["t_comp_percent_checksum"] = (
        grouped_data["t_comp_percent_simulating"]
        + grouped_data["t_comp_percent_all_wo_sim"]
    )

    fig3, ax3 = plt.subplots(figsize=(5, 1.5))
    sns.barplot(
        x="experiment",
        hue="method",
        y="t_comp_percent_checksum",
        palette=color_palette_lighter,
        color="b",
        hue_order=method_strings,
        data=grouped_data,
        ax=ax3,
        hatch="..",
        alpha=0.99,
    )
    sns.barplot(
        x="experiment",
        hue="method",
        y="t_comp_percent_simulating",
        palette=color_palette,
        color="b",
        hue_order=method_strings,
        data=grouped_data,
        ax=ax3,
        hatch="--",
        alpha=0.99,
    )
    legend_elements = list()
    # for i in range(len(method_strings)):
    #     legend_elements.append(
    #         Patch(facecolor=color_palette[i], label=method_strings[i])
    #     )
    legend_elements.append(
        Patch(facecolor="white", edgecolor="gray", hatch="--", label="Simulation")
    )
    legend_elements.append(
        Patch(facecolor="white", edgecolor="gray", hatch="..", label="Other")
    )
    lgd3 = plt.legend(
        handles=legend_elements,
        bbox_to_anchor=(0.0, 1.02, 1.0, 0.102),
        loc="lower left",
        ncol=2,
        mode="expand",
        borderaxespad=0.0,
    )
    plt.xlabel("Experiment ID")
    plt.ylabel("Relative time [%]")
    plt.show()

    # Save figures
    time_now = datetime.now()
    time_string = time_now.strftime("%y%m%d-%H%M%S")
    # fig1.savefig(
    #     os.path.join(basedir, "Output", f"{time_string}_boxplot.pdf"),
    #     bbox_extra_artists=(lgd1,),
    #     bbox_inches="tight",
    # )
    # fig2.savefig(
    #     os.path.join(basedir, "Output", f"{time_string}_barplot.pdf"),
    #     bbox_extra_artists=(lgd2,),
    #     bbox_inches="tight",
    # )
    fig3.savefig(
        os.path.join(basedir, "Output", f"{time_string}_timings.pdf"),
        bbox_extra_artists=(lgd3,),
        bbox_inches="tight",
    )


if __name__ == "__main__":
    compare_hlp_mcts()
