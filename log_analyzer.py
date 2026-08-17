import re
import os
import numpy as np
import matplotlib.pyplot as plt

# ---------------------------------------------------
# 设置 matplotlib 正常显示中文和负号
# ---------------------------------------------------

# --- 修改这部分 ---
plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei'] # 专门为 Ubuntu 设置的字体
plt.rcParams['axes.unicode_minus'] = False
# -----------------

def parse_nav2_log(file_path):
    node_counts = []
    durations = []
    # 兼容多种日志格式的正则
    node_pattern = re.compile(r"(?:采样节点数|采样点数|原始点数|采样数):\s*(\d+)")
    time_pattern = re.compile(r"耗时:\s*([\d\.]+)\s*ms")

    if not os.path.exists(file_path):
        return None, None

    with open(file_path, 'r', encoding='utf-8') as f:
        for line in f:
            node_match = node_pattern.search(line)
            time_match = time_pattern.search(line)
            if node_match and time_match:
                node_counts.append(int(node_match.group(1)))
                durations.append(float(time_match.group(1)))
    return node_counts, durations

def analyze_all_logs(log_dir="."):
    results = {}
    log_files = [f for f in os.listdir(log_dir) if f.endswith('.log')]
    if not log_files:
        print("未在当前目录下找到 .log 文件！")
        return None

    for file_name in log_files:
        label = file_name.replace(".log", "").upper()
        nodes, times = parse_nav2_log(os.path.join(log_dir, file_name))
        if nodes and len(nodes) > 0:
            results[label] = {
                "nodes": nodes,
                "times": times,
                "avg_node": np.mean(nodes),
                "avg_time": np.mean(times)
            }
            print(f"成功解析: {file_name} (样本数: {len(nodes)})")
    return results

def get_sorted_labels(results):
    """
    定义显示的逻辑顺序
    """
    custom_order = [
        "ORIGINAL_RRT", 
        "IMPROVED_DYNAMIC_BIASED_RRT", 
        "IMPROVED_APF_RRT",
        "IMPROVED_ADAPTIVE_STEP_RRT",
        # "IMPROVED_PRUNING_RRT",
        # "IMPROVED_BSPLINE_SMOOTH_RRT",
        # "B_SPLINE_SMOOTH_RRT",
    ]
    # 过滤掉不存在的 log，并保留不在列表中的额外 log
    sorted_labels = [l for l in custom_order if l in results]
    other_labels = [l for l in results.keys() if l not in custom_order]
    return sorted_labels + other_labels

def print_summary_table(results):
    sorted_labels = get_sorted_labels(results)
    
    header = f"{'Version':<20} | {'Samples':<8} | {'Avg Nodes':<10} | {'Avg Time(ms)':<12} | {'Max Time':<10}"
    print("\n" + "="*75)
    print("                RRT 算法前端改进多版本实验对比分析报告")
    print("-" * 75)
    print(header)
    print("-" * 75)
    
    for label in sorted_labels:
        res = results[label]
        avg_n = f"{res['avg_node']:.2f}"
        avg_t = f"{res['avg_time']:.2f}"
        max_t = f"{np.max(res['times']):.2f}"
        print(f"{label:<20} | {len(res['nodes']):<8} | {avg_n:<10} | {avg_t:<12} | {max_t:<10}")
    
    print("="*75 + "\n")

def plot_multi_comparison(results):
    # 获取排序后的标签
    labels = get_sorted_labels(results)
    avg_nodes = [results[l]["avg_node"] for l in labels]
    avg_times = [results[l]["avg_time"] for l in labels]

    x = np.arange(len(labels))
    width = 0.4
    fig, ax1 = plt.subplots(figsize=(12, 7))

    # 绘制左轴：平均节点数（柱状图）- 为了图表统一，我也顺手将图例(label)改成了中文
    bars = ax1.bar(x, avg_nodes, width, alpha=0.7, color='skyblue', label='平均节点数')
    ax1.set_xlabel('算法版本', fontweight='bold')  # 修改为中文
    ax1.set_ylabel('平均节点数', color='steelblue', fontweight='bold')  # 修改为中文
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, rotation=15) # 旋转标签防止重叠
    ax1.tick_params(axis='y', labelcolor='steelblue')

    # 绘制右轴：平均耗时（折线图）- 同理，图例(label)改成中文
    ax2 = ax1.twinx()
    ax2.plot(x, avg_times, color='crimson', marker='o', markersize=8, linewidth=2, label='平均耗时 (ms)')
    ax2.set_ylabel('平均耗时 (ms)', color='crimson', fontweight='bold')  # 修改为中文
    ax2.tick_params(axis='y', labelcolor='crimson')

    # 添加数值标注
    for i, v in enumerate(avg_nodes):
        ax1.text(i, v + 5, f'{int(v)}', ha='center', color='steelblue', fontweight='bold')

    # 图例处理
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left')

    # 修改为中文标题
    plt.title('RRT算法性能对比 (前端改进)', fontsize=14, fontweight='bold')
    plt.grid(axis='y', linestyle='--', alpha=0.3)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    all_results = analyze_all_logs(".")
    if all_results:
        print_summary_table(all_results)
        plot_multi_comparison(all_results)