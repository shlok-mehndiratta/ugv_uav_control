import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import os
import glob

results_dir = '/home/shlok-mehndiratta/Desktop/LCS-334-434-684/Project/Shlok_23309, Divyam_23112, Prashant_23237/ros2_ws/results'
files = glob.glob(os.path.join(results_dir, 'C1_Baseline*.csv'))
c1_csv = max(files, key=os.path.getctime) if files else None

track_img_path = '/home/shlok-mehndiratta/Desktop/LCS-334-434-684/Project/Shlok_23309, Divyam_23112, Prashant_23237/ros2_ws/src/ugv_uav_control/models/path_ground_plane_v2/materials/textures/track4.png'

df = pd.read_csv(c1_csv)
orig_img = mpimg.imread(track_img_path)

fig, axes = plt.subplots(2, 4, figsize=(20, 10))
axes = axes.flatten()

configs = [
    (0, False, False), (1, False, False), (2, False, False), (3, False, False),
    (0, True, False), (1, True, False), (0, False, True), (1, False, True)
]

for i, (k, flip_lr, flip_ud) in enumerate(configs):
    img = np.rot90(orig_img, k=k)
    if flip_lr: img = np.fliplr(img)
    if flip_ud: img = np.flipud(img)
    
    ax = axes[i]
    ax.imshow(img, extent=[-6, 6, -6, 6], alpha=0.7, origin='lower')
    ax.plot(df['x'], df['y'], color='blue', linewidth=2)
    ax.set_title(f'rot90={k}, flr={flip_lr}, fud={flip_ud}')
    ax.set_xlim([-6, 6]); ax.set_ylim([-6, 6])

plt.tight_layout()
plt.savefig(os.path.join(results_dir, 'alignment_debug_2.png'))
