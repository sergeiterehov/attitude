"""
Визуализация результатов тестирования фильтра.

Читает results.csv (опционально несколько файлов для сравнения), строит графики по сценариям:
- True vs filter attitude over time
- Error over time
- Метрики: max, RMS, mean error

Использование:
  python3 visualize.py results_custom.csv [results_mahony.csv ...] [-o output_dir]
"""

import csv
import sys
from pathlib import Path

try:
    import matplotlib.pyplot as plt
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend
except ImportError:
    print("Error: matplotlib not installed. Install with: pip3 install matplotlib")
    sys.exit(1)


# Цвета для разных фильтров
FILTER_COLORS = [
    ('blue', 'Filter 1'),
    ('red', 'Filter 2'),
    ('green', 'Filter 3'),
    ('orange', 'Filter 4'),
]


def read_results(filename):
    """Читает results.csv, возвращает данные по сценариям."""
    scenarios = {}

    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            scenario = row['scenario']
            if scenario not in scenarios:
                scenarios[scenario] = {
                    't_ms': [],
                    'true_roll': [],
                    'true_pitch': [],
                    'filter_roll': [],
                    'filter_pitch': [],
                    'error_roll': [],
                    'error_pitch': [],
                }

            s = scenarios[scenario]
            s['t_ms'].append(int(row['t_ms']))
            s['true_roll'].append(float(row['true_roll']))
            s['true_pitch'].append(float(row['true_pitch']))
            s['filter_roll'].append(float(row['filter_roll']))
            s['filter_pitch'].append(float(row['filter_pitch']))
            s['error_roll'].append(float(row['error_roll']))
            s['error_pitch'].append(float(row['error_pitch']))

    return scenarios


def compute_metrics(errors):
    """Вычисляет метрики: max, RMS, mean."""
    if not errors:
        return 0, 0, 0
    max_err = max(abs(e) for e in errors)
    rms_err = (sum(e * e for e in errors) / len(errors)) ** 0.5
    mean_err = sum(abs(e) for e in errors) / len(errors)
    return max_err, rms_err, mean_err


def plot_scenario(scenario_name, true_data, filter_datasets, output_dir):
    """
    Строит графики для одного сценария с поддержкой нескольких фильтров.
    
    true_data: данные с true_roll/true_pitch (из первого файла)
    filter_datasets: список (filter_name, data) для каждого фильтра
    """
    t_sec = [t / 1000.0 for t in true_data['t_ms']]

    fig, axes = plt.subplots(4, 1, figsize=(12, 13), sharex=True)

    # Roll
    ax = axes[0]
    ax.plot(t_sec, true_data['true_roll'], 'k-', label='True', linewidth=2)
    for i, (filter_name, data) in enumerate(filter_datasets):
        color = FILTER_COLORS[i % len(FILTER_COLORS)][0]
        ax.plot(t_sec, data['filter_roll'], color=color, label=filter_name, alpha=0.7, linewidth=1.5)
    ax.set_ylabel('Roll (deg)')
    ax.set_title(f'{scenario_name} — Roll')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Pitch
    ax = axes[1]
    ax.plot(t_sec, true_data['true_pitch'], 'k-', label='True', linewidth=2)
    for i, (filter_name, data) in enumerate(filter_datasets):
        color = FILTER_COLORS[i % len(FILTER_COLORS)][0]
        ax.plot(t_sec, data['filter_pitch'], color=color, label=filter_name, alpha=0.7, linewidth=1.5)
    ax.set_ylabel('Pitch (deg)')
    ax.set_title(f'{scenario_name} — Pitch')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Error
    ax = axes[2]
    for i, (filter_name, data) in enumerate(filter_datasets):
        color = FILTER_COLORS[i % len(FILTER_COLORS)][0]
        ax.plot(t_sec, data['error_roll'], color=color, linestyle='-', label=f'{filter_name} roll', alpha=0.7)
        ax.plot(t_sec, data['error_pitch'], color=color, linestyle='--', label=f'{filter_name} pitch', alpha=0.7)
    ax.set_ylabel('Error (deg)')
    ax.set_title(f'{scenario_name} — Error')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Cumulative absolute error
    ax = axes[3]
    dt = 0.01  # 100 Hz
    for i, (filter_name, data) in enumerate(filter_datasets):
        color = FILTER_COLORS[i % len(FILTER_COLORS)][0]
        # Интеграл |error_roll| + |error_pitch| по времени
        cum_err_roll = []
        cum_err_pitch = []
        s_roll = 0.0
        s_pitch = 0.0
        for er, ep in zip(data['error_roll'], data['error_pitch']):
            s_roll += abs(er) * dt
            s_pitch += abs(ep) * dt
            cum_err_roll.append(s_roll)
            cum_err_pitch.append(s_pitch)
        ax.plot(t_sec, cum_err_roll, color=color, linestyle='-', label=f'{filter_name} roll', alpha=0.7)
        ax.plot(t_sec, cum_err_pitch, color=color, linestyle='--', label=f'{filter_name} pitch', alpha=0.7)
    ax.set_ylabel('Cumulative |error| (deg·s)')
    ax.set_xlabel('Time (s)')
    ax.set_title(f'{scenario_name} — Cumulative Absolute Error')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_dir / f'{scenario_name}.png', dpi=150)
    plt.close()


def print_summary(all_datasets):
    """Печатает сводку метрик для всех фильтров."""
    print("\n" + "=" * 120)
    print("SUMMARY")
    print("=" * 120)
    
    # Заголовок
    header = f"{'Scenario':<25}"
    for filter_name, _ in all_datasets:
        header += f" | {filter_name:>28}"
    print(header)
    print("-" * 120)
    
    # Получаем список сценариев из первого датасета
    _, first_data = all_datasets[0]
    scenarios = sorted(first_data.keys())
    
    for scenario_name in scenarios:
        row = f"{scenario_name:<25}"
        for filter_name, data in all_datasets:
            if scenario_name in data:
                roll_max, roll_rms, _ = compute_metrics(data[scenario_name]['error_roll'])
                pitch_max, pitch_rms, _ = compute_metrics(data[scenario_name]['error_pitch'])
                cum_roll = sum(abs(e) * 0.01 for e in data[scenario_name]['error_roll'])
                cum_pitch = sum(abs(e) * 0.01 for e in data[scenario_name]['error_pitch'])
                row += f" | R:{roll_max:5.1f}/{roll_rms:4.1f}° Σ{cum_roll:6.0f} P:{pitch_max:5.1f}/{pitch_rms:4.1f}° Σ{cum_pitch:6.0f}"
            else:
                row += f" | {'N/A':>28}"
        print(row)
    
    print("=" * 120)
    print("Формат: R:max/RMS° Σcumulative  P:max/RMS° Σcumulative")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 visualize.py <results1.csv> [results2.csv ...] [-o output_dir]")
        print("  Supports multiple result files for comparison")
        sys.exit(1)

    # Parse arguments
    results_files = []
    output_dir = Path("test/filter_sim/plots")
    
    i = 1
    while i < len(sys.argv):
        if sys.argv[i] == '-o' and i + 1 < len(sys.argv):
            output_dir = Path(sys.argv[i + 1])
            i += 2
        else:
            results_files.append(sys.argv[i])
            i += 1
    
    if not results_files:
        print("Error: no result files specified")
        sys.exit(1)
    
    output_dir.mkdir(parents=True, exist_ok=True)

    # Read all result files
    all_datasets = []
    true_data = None
    
    for results_file in results_files:
        print(f"Reading {results_file}...")
        data = read_results(results_file)
        filter_name = Path(results_file).stem  # e.g., "results_mahony" -> "mahony"
        all_datasets.append((filter_name, data))
        
        # Используем true данные из первого файла
        if true_data is None:
            true_data = data
        
        print(f"  {filter_name}: {len(data)} scenarios")

    print("Generating plots...")
    for scenario_name in sorted(true_data.keys()):
        filter_datasets = [(name, data[scenario_name]) for name, data in all_datasets if scenario_name in data]
        plot_scenario(scenario_name, true_data[scenario_name], filter_datasets, output_dir)
        print(f"  {scenario_name}: {len(true_data[scenario_name]['t_ms'])} samples")

    print(f"\nPlots saved to {output_dir}/")

    print_summary(all_datasets)


if __name__ == "__main__":
    main()
