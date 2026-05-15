import csv
import numpy as np
import open3d as o3d


def load_scan_csv(path):
    angles = []
    dists = []
    quals = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            d = float(row['distance_mm'])
            if d <= 0:
                continue
            angles.append(float(row['angle_deg']))
            dists.append(d)
            quals.append(int(row['quality']))
    return np.array(angles), np.array(dists), np.array(quals)


def polar_to_cartesian(angles_deg, dists_mm, tilt_angle_deg=0):
    tilt_rad = np.radians(tilt_angle_deg)
    theta = np.radians(angles_deg)
    r = dists_mm / 1000.0
    x = r * np.cos(theta) * np.cos(tilt_rad)
    y = r * np.sin(theta) * np.cos(tilt_rad)
    z = np.full_like(x, r * np.sin(tilt_rad))
    return np.column_stack([x, y, z])


def main():
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else 'data/lidar_scan.csv'

    print(f"Loading {path}...")
    angles, dists, quals = load_scan_csv(path)
    print(f"Loaded {len(angles)} valid points")

    tilt = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    xyz = polar_to_cartesian(angles, dists, tilt)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    colors = np.zeros((len(xyz), 3))
    norm_qual = quals / 255.0
    colors[:, 0] = norm_qual
    colors[:, 2] = 1.0 - norm_qual
    pcd.colors = o3d.utility.Vector3dVector(colors)

    o3d.visualization.draw_geometries(
        [pcd],
        window_name='LiDAR Scan',
        width=1024, height=768,
    )


if __name__ == '__main__':
    main()
