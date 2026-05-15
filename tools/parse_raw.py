import struct
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def parse_lidar(raw_path, out_prefix='lidar'):
    with open(raw_path, 'rb') as f:
        data = f.read()

    pkts = []
    i = 0
    while i < len(data):
        if data[i] == 0x55 and i + 36 <= len(data):
            pkts.append(data[i:i+36])
            i += 36
        else:
            i += 1

    print(f'Total packets: {len(pkts)}')

    counters = []
    for p in pkts:
        c = struct.unpack('<H', p[6:8])[0]
        counters.append(c)

    rotations = [0]
    for i in range(1, len(counters)):
        if counters[i-1] - counters[i] > 1000:
            rotations.append(i)

    print(f'Rotations detected: {len(rotations)}')
    print(f'Avg packets/rotation: {len(pkts)/max(1,len(rotations)):.1f}')

    rotations_data = []
    for rot_idx in range(len(rotations)):
        start = rotations[rot_idx]
        end = rotations[rot_idx + 1] if rot_idx + 1 < len(rotations) else len(pkts)
        pkt_in_rot = end - start
        total_samples = pkt_in_rot * 8

        if total_samples < 10:
            continue

        rot_angles = []
        rot_dists = []
        rot_quals = []

        for pkt_idx in range(start, end):
            p = pkts[pkt_idx]
            for s in range(8):
                off = 8 + s * 3
                if off + 3 > 36:
                    break
                q = p[off]
                d_raw = struct.unpack('>H', p[off+1:off+3])[0]
                d_mm = d_raw * 0.25 if d_raw < 0x8000 else 0
                sample_global = (pkt_idx - start) * 8 + s
                angle_deg = (sample_global / total_samples) * 360.0
                rot_angles.append(angle_deg)
                rot_dists.append(d_mm)
                rot_quals.append(q)

        rotations_data.append((rot_angles, rot_dists, rot_quals))

    all_angles = []
    all_dists = []
    all_quals = []

    for ang, dst, ql in rotations_data:
        all_angles.extend(ang)
        all_dists.extend(dst)
        all_quals.extend(ql)

    print(f'Total samples: {len(all_angles)}')
    valid = sum(1 for d in all_dists if d > 0)
    print(f'Valid returns: {valid} ({100*valid/len(all_angles):.0f}%)')
    if valid:
        valid_dists = [d for d in all_dists if d > 0]
        print(f'Distance range: {min(valid_dists):.0f} - {max(valid_dists):.0f} mm')

    csv_path = f'{out_prefix}_scan.csv'
    with open(csv_path, 'w') as f:
        f.write('angle_deg,distance_mm,quality,rotation\n')
        for ri, (ang, dst, ql) in enumerate(rotations_data):
            for a, d, q in zip(ang, dst, ql):
                f.write(f'{a:.2f},{d:.1f},{q},{ri}\n')
    print(f'Saved: {csv_path}')

    fig, axes = plt.subplots(1, 2, figsize=(16, 7),
                             subplot_kw={'projection': 'polar'})
    for ri, (ang, dst, ql) in enumerate(rotations_data):
        angles_rad = np.radians(ang)
        valid_pts = [(a, d, q) for a, d, q in zip(angles_rad, dst, ql)
                     if 10 < d < 8000]
        if not valid_pts:
            continue
        va, vd, vq = zip(*valid_pts)
        axes[0].scatter(va, vd, c=vq, cmap='viridis',
                        s=3, alpha=0.6, label=f'Rot {ri}')
        axes[0].set_title('Distance by quality (color)', fontsize=11)
        axes[0].set_ylim(0, 6000)
        axes[1].scatter(va, [d/1000 for d in vd],
                        c=[d/1000 for d in vd],
                        cmap='plasma', s=3, alpha=0.6)
        axes[1].set_title('Distance in meters', fontsize=11)
        axes[1].set_ylim(0, 6)

    plt.tight_layout()
    png_path = f'{out_prefix}_polar.png'
    plt.savefig(png_path, dpi=150)
    plt.close()
    print(f'Saved: {png_path}')

    fig, ax = plt.subplots(figsize=(14, 5))
    for ri, (ang, dst, ql) in enumerate(rotations_data):
        valid_pts = [(a, d, q) for a, d, q in zip(ang, dst, ql)
                     if 10 < d < 8000]
        if not valid_pts:
            continue
        va, vd, vq = zip(*valid_pts)
        ax.scatter(va, vd, c=vq, cmap='viridis', s=3, alpha=0.4)
    ax.set_xlim(0, 360)
    ax.set_xlabel('Angle (deg)')
    ax.set_ylabel('Distance (mm)')
    ax.set_title('LiDAR Scan - Cartesian')
    ax.grid(True, alpha=0.3)
    cart_path = f'{out_prefix}_cartesian.png'
    plt.savefig(cart_path, dpi=150)
    plt.close()
    print(f'Saved: {cart_path}')


if __name__ == '__main__':
    path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/lidar_raw.bin'
    prefix = sys.argv[2] if len(sys.argv) > 2 else 'lidar'
    parse_lidar(path, prefix)
