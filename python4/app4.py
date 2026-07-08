import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

file_path = "data.txt"

data = []

# ===== ЧТЕНИЕ ФАЙЛА =====
with open(file_path, "r", encoding="utf-8") as f:
    for line in f:
        line = line.strip()

        if not line.startswith("R"):
            continue

        line = line.replace("<\\r>", "").replace("<\\n>", "")
        line = line[1:]

        values = line.split(",")

        if len(values) != 14:
            continue

        try:
            values = list(map(float, values))
        except ValueError:
            continue

        data.append({
            "x0": values[0], "y0": values[1],
            "x1": values[4], "y1": values[5],
            "x2": values[8], "y2": values[9],
            "ts_ms": values[12],
        })

df = pd.DataFrame(data)

# ===== ВЫБОР ЦЕЛИ =====
target_id = 0   # можно менять 0 / 1 / 2

x_col = f"x{target_id}"
y_col = f"y{target_id}"

# ===== ВЫЧИСЛЯЕМ ПОЛНОЕ РАССТОЯНИЕ =====
df["R"] = np.sqrt(df[x_col]**2 + df[y_col]**2)

# ===== ПОСТРОЕНИЕ =====
fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

# 1️⃣ X
axs[0].plot(df["ts_ms"], df[x_col])
axs[0].set_title(f"Цель {target_id}: X (x0) — поперечная ось")
axs[0].set_ylabel("X")
axs[0].grid(True)

# 2️⃣ Y
axs[1].plot(df["ts_ms"], df[y_col])
axs[1].set_title(f"Цель {target_id}: Y — продольная ось")
axs[1].set_ylabel("Y")
axs[1].grid(True)

# 3️⃣ Полная дистанция
axs[2].plot(df["ts_ms"], df["R"])
axs[2].set_title(f"Цель {target_id}: Полное расстояние sqrt(x² + y²)")
axs[2].set_xlabel("Время (ms)")
axs[2].set_ylabel("Расстояние")
axs[2].grid(True)

plt.tight_layout()
plt.show()