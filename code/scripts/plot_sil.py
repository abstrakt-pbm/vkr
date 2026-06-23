import os

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import pandas as pd

# ===== НАСТРОЙКИ =====
CSV_FILE = "square.csv"   # имя входного CSV-файла
OUTPUT_DIR = "plots"      # каталог для сохранения графиков

os.makedirs(OUTPUT_DIR, exist_ok=True)

# ===== НАСТРОЙКИ ОТОБРАЖЕНИЯ =====
plt.rcParams["font.family"] = "DejaVu Sans"
plt.rcParams["axes.unicode_minus"] = False

# ===== ЗАГРУЗКА ДАННЫХ =====
df = pd.read_csv(CSV_FILE)
t = df["t"]


def save_plot(
    x,
    y_list,
    labels,
    xlabel,
    ylabel,
    output_path,
    equal_axis=False,
):
    plt.figure()
    for y, label in zip(y_list, labels):
        if label is None:
            plt.plot(x, y)
        else:
            plt.plot(x, y, label=label)

    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True)

    if any(label is not None for label in labels):
        plt.legend()

    if equal_axis:
        plt.axis("equal")

    plt.tight_layout()
    plt.savefig(output_path, dpi=200)
    plt.close()


# ===== 1. ЛИНЕЙНАЯ СКОРОСТЬ =====
save_plot(
    x=t,
    y_list=[df["v_cmd"], df["v_meas"]],
    labels=["Заданная скорость", "Фактическая скорость"],
    xlabel="Время, с",
    ylabel="Линейная скорость, м/с",
    output_path=f"{OUTPUT_DIR}/linear_velocity.png",
)

# ===== 2. УГЛОВАЯ СКОРОСТЬ =====
save_plot(
    x=t,
    y_list=[df["w_cmd"], df["w_meas"]],
    labels=["Заданная угловая скорость", "Фактическая угловая скорость"],
    xlabel="Время, с",
    ylabel="Угловая скорость, рад/с",
    output_path=f"{OUTPUT_DIR}/angular_velocity.png",
)

# ===== 3. ОШИБКА ПО ЛИНЕЙНОЙ СКОРОСТИ =====
save_plot(
    x=t,
    y_list=[df["e_v"]],
    labels=[None],
    xlabel="Время, с",
    ylabel="Ошибка линейной скорости, м/с",
    output_path=f"{OUTPUT_DIR}/error_linear.png",
)

# ===== 4. ОШИБКА ПО УГЛОВОЙ СКОРОСТИ =====
save_plot(
    x=t,
    y_list=[df["e_w"]],
    labels=[None],
    xlabel="Время, с",
    ylabel="Ошибка угловой скорости, рад/с",
    output_path=f"{OUTPUT_DIR}/error_angular.png",
)

# ===== 5. УПРАВЛЯЮЩИЕ ВОЗДЕЙСТВИЯ =====
save_plot(
    x=t,
    y_list=[df["left_voltage"], df["right_voltage"]],
    labels=["Левый привод", "Правый привод"],
    xlabel="Время, с",
    ylabel="Напряжение, В",
    output_path=f"{OUTPUT_DIR}/control.png",
)

# ===== 6. ТРАЕКТОРИЯ =====
save_plot(
    x=df["x_gt"],
    y_list=[df["y_gt"]],
    labels=[None],
    xlabel="x, м",
    ylabel="y, м",
    output_path=f"{OUTPUT_DIR}/trajectory.png",
    equal_axis=True,
)

print("Графики сохранены в каталог:", OUTPUT_DIR)
