import os
import re
import pandas as pd

CONSTANT = 0.009765 * 9.81
CONTACT_FORCES_DIR = os.path.dirname(os.path.abspath(__file__))
TEX_FILE = os.path.join(CONTACT_FORCES_DIR, "contact_forces_table.tex")

csv_files = [f for f in os.listdir(CONTACT_FORCES_DIR) if f.endswith(".csv")]

episodes = []
for fname in csv_files:
    match = re.search(r"episode_(\d+)_z_force\.csv", fname)
    if not match:
        continue
    ep_num = int(match.group(1))
    df = pd.read_csv(os.path.join(CONTACT_FORCES_DIR, fname))
    initial = df["z_force"].iloc[0]
    if initial < 0:
        print(f"Episode {ep_num}: initial z_force = {initial:.4f}, applying offset +{abs(initial):.4f}")
        print(f"  before: mean={df['z_force'].mean():.4f}, min={df['z_force'].min():.4f}, max={df['z_force'].max():.4f}")
        df["z_force"] = df["z_force"] + abs(initial)
        print(f"  after:  mean={df['z_force'].mean():.4f}, min={df['z_force'].min():.4f}, max={df['z_force'].max():.4f}")
    df["z_force_N"] = df["z_force"] * CONSTANT
    df["timestamp"] = pd.to_datetime(df["timestamp"], utc=True)
    duration = (df["timestamp"].iloc[-1] - df["timestamp"].iloc[0]).total_seconds()
    episodes.append({
        "ep": ep_num,
        "N": len(df),
        "duration": duration,
        "mean": df["z_force_N"].mean(),
        "std": df["z_force_N"].std(),
        "min": df["z_force_N"].min(),
        "max": df["z_force_N"].max(),
    })

episodes.sort(key=lambda x: x["ep"])

rows = []
for e in episodes:
    rows.append(
        f"  {e['ep']:<3} & {e['N']:>4} & {e['duration']:>5.1f} & "
        f"{e['mean']:>7.2f} & {e['std']:>6.2f} & "
        f"{e['min']:>8.2f} & {e['max']:>7.2f} \\\\"
    )

table_body = "\n".join(rows)

tex = f"""\
% Required packages in preamble: \\usepackage{{booktabs,longtable}}
\\begin{{longtable}}{{rrrrrrr}}
  \\caption{{Statistike Z-sile u fazi kontakta po epizodi.}}
  \\label{{tab:contact_forces}} \\\\
  \\toprule
  Epizoda & $N$ & Trajanje (s) & Srednja vr. (N) & Std. odst. (N) & Min. (N) & Maks. (N) \\\\
  \\midrule
  \\endfirsthead
  \\toprule
  Epizoda & $N$ & Trajanje (s) & Srednja vr. (N) & Std. odst. (N) & Min. (N) & Maks. (N) \\\\
  \\midrule
  \\endhead
  \\midrule
  \\endfoot
  \\bottomrule
  \\endlastfoot
{table_body}
\\end{{longtable}}
"""

with open(TEX_FILE, "w", encoding="utf-8") as f:
    f.write(tex)

print(f"Written {len(episodes)} episodes to {TEX_FILE}")
