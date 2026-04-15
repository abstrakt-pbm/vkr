import matplotlib
matplotlib.use("Agg")

import pandas as pd
import matplotlib.pyplot as plt
import os

# ===== НАСТРОЙКИ =====
CSV_FILE = "square.csv"        # <-- поменяй на свой файл
OUTPUT_DIR = "plots"

os.makedirs(OUTPUT_DIR, exist_ok=True)

# ===== ЗАГРУЗКА =====
df = pd.read_csv(CSV_FILE)

t = df["t"]

# ===== 1. ЛИНЕЙНАЯ СКОРОСТЬ =====
plt.figure()
plt.plot(t, df["v_cmd"], label="v_cmd")
plt.plot(t, df["v_meas"], label="v_meas")
plt.xlabel("t, s")
plt.ylabel("v, m/s")
plt.title("Linear velocity")
plt.legend()
plt.grid(True)
plt.savefig(f"{OUTPUT_DIR}/linear_velocity.png", dpi=200)
plt.close()

# ===== 2. УГЛОВАЯ СКОРОСТЬ =====
plt.figure()
plt.plot(t, df["w_cmd"], label="w_cmd")
plt.plot(t, df["w_meas"], label="w_meas")
plt.xlabel("t, s")
plt.ylabel("omega, rad/s")
plt.title("Angular velocity")
plt.legend()
plt.grid(True)
plt.savefig(f"{OUTPUT_DIR}/angular_velocity.png", dpi=200)
plt.close()

# ===== 3. ОШИБКА ПО ЛИНЕЙНОЙ =====
plt.figure()
plt.plot(t, df["e_v"])
plt.xlabel("t, s")
plt.ylabel("e_v, m/s")
plt.title("Linear velocity error")
plt.grid(True)
plt.savefig(f"{OUTPUT_DIR}/error_linear.png", dpi=200)
plt.close()

# ===== 4. ОШИБКА ПО УГЛОВОЙ =====
plt.figure()
plt.plot(t, df["e_w"])
plt.xlabel("t, s")
plt.ylabel("e_w, rad/s")
plt.title("Angular velocity error")
plt.grid(True)
plt.savefig(f"{OUTPUT_DIR}/error_angular.png", dpi=200)
plt.close()

# ===== 5. УПРАВЛЯЮЩИЕ ВОЗДЕЙСТВИЯ =====
plt.figure()
plt.plot(t, df["left_voltage"], label="left")
plt.plot(t, df["right_voltage"], label="right")
plt.xlabel("t, секунды")
plt.ylabel("Вольтаж")
plt.title("Вольтаж на моторах")
plt.legend()
plt.grid(True)
plt.savefig(f"{OUTPUT_DIR}/control.png", dpi=200)
plt.close()

# ===== 6. ТРАЕКТОРИЯ =====
plt.figure()
plt.plot(df["x_gt"], df["y_gt"])
plt.xlabel("x, m")
plt.ylabel("y, m")
plt.title("Траектория")
plt.grid(True)
plt.axis("equal")
plt.savefig(f"{OUTPUT_DIR}/trajectory.png", dpi=200)
plt.close()

print("Plots saved to:", OUTPUT_DIR)

