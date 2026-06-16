"""
path_planner.py
===============
Path planning menggunakan A* atau path manual.
Output: list of (row, col) dari start ke finish.
"""

import heapq
import numpy as np
from grid_map import ARENA_MAP, ROWS, COLS, CELL_SIZE, is_free, print_map

MANUAL_PATH = np.array([
    [0, 0, 0, 0, 0, 0, 1, 0, 0],   # Row 0
    [0, 0, 0, 0, 0, 0, 1, 0, 0],   # Row 1
    [0, 0, 0, 0, 0, 0, 1, 1, 0],   # Row 2
    [0, 0, 0, 0, 0, 0, 0, 1, 0],   # Row 3
    [0, 0, 0, 0, 0, 0, 0, 1, 0],   # Row 4
    [0, 0, 0, 0, 0, 0, 1, 1, 0],   # Row 5
    [0, 0, 0, 0, 0, 0, 1, 0, 0],   # Row 6
    [0, 0, 0, 0, 0, 0, 1, 0, 0],   # Row 7
    [0, 0, 0, 0, 0, 0, 1, 1, 0],   # Row 8
    [0, 0, 0, 0, 0, 0, 0, 1, 0],   # Row 9
    [0, 0, 0, 0, 0, 0, 1, 1, 0],   # Row 10
    [0, 0, 0, 0, 0, 0, 1, 0, 0],   # Row 11
    [0, 0, 0, 0, 0, 0, 0, 0, 0],   # Row 12
    [0, 0, 0, 0, 0, 0, 0, 0, 0],   # Row 13
])


# ── A* ─────────────────────────────────────────────────────

def heuristic(a: tuple, b: tuple) -> float:
    """Manhattan distance heuristic."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(start: tuple, goal: tuple, margin: int = 0) -> list:
    """
    A* pathfinding di grid.

    Parameters:
        start  : (row, col) posisi awal
        goal   : (row, col) posisi tujuan
        margin : jumlah sel buffer di sekitar obstacle (0 = tidak ada buffer)

    Returns:
        list of (row, col), kosong jika tidak ada jalur
    """
    # Buat grid dengan buffer obstacle
    grid = _build_buffered_grid(margin)

    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from   = {}
    g_score     = {start: 0}
    f_score     = {start: heuristic(start, goal)}

    # 4-directional movement (tidak diagonal)
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return _reconstruct_path(came_from, current)

        for dr, dc in neighbors:
            nr, nc = current[0] + dr, current[1] + dc
            neighbor = (nr, nc)

            if not (0 <= nr < ROWS and 0 <= nc < COLS):
                continue
            if grid[nr, nc] != 0:
                continue

            tentative_g = g_score[current] + 1

            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor]   = tentative_g
                f_score[neighbor]   = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []   # tidak ada jalur


def _build_buffered_grid(margin: int) -> np.ndarray:
    """Tambah buffer di sekitar obstacle agar robot tidak terlalu mepet."""
    grid = ARENA_MAP.copy()
    if margin == 0:
        return grid

    obstacles = np.argwhere(ARENA_MAP == 1)
    for (r, c) in obstacles:
        for dr in range(-margin, margin + 1):
            for dc in range(-margin, margin + 1):
                nr, nc = r + dr, c + dc
                if 0 <= nr < ROWS and 0 <= nc < COLS:
                    grid[nr, nc] = 1
    return grid


def _reconstruct_path(came_from: dict, current: tuple) -> list:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


# ── Manual path ─────────────────────────────────────────────

def parse_manual_path(path_matrix: np.ndarray, start: tuple) -> list:
    """
    Konversi matriks path manual ke list waypoint dengan cara "menelusuri" angka 1.
    Memulai dari titik start robot, lalu mencari angka 1 yang menyambung di sebelahnya.
    """
    if path_matrix is None:
        return []

    # Pastikan posisi start robot juga ditandai dengan angka 1 di matriks
    if path_matrix[start[0], start[1]] != 1:
        print(f"[PATH WARNING] Titik start {start} tidak ada angka 1 di matriks manual!")
        return []

    path = [start]
    visited = set()
    visited.add(start)
    
    current = start
    # Arah pencarian tetangga: Atas, Bawah, Kiri, Kanan
    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while True:
        next_step = None
        for dr, dc in neighbors:
            nr, nc = current[0] + dr, current[1] + dc
            
            # Pastikan tidak keluar dari batas array/peta
            if 0 <= nr < ROWS and 0 <= nc < COLS:
                # Jika sel tetangga adalah 1 dan belum pernah dilewati
                if path_matrix[nr, nc] == 1 and (nr, nc) not in visited:
                    next_step = (nr, nc)
                    break # Langsung kunci langkah ini

        # Jika tidak ada lagi angka 1 di sekelilingnya, berarti jalur sudah mentok/selesai
        if next_step is None:
            break

        path.append(next_step)
        visited.add(next_step)
        current = next_step

    return path


def get_path(start: tuple, goal: tuple,
             manual_matrix: np.ndarray = None,
             obstacle_margin: int = 0) -> list:
    """
    Dapatkan path dari start ke goal.
    Jika manual_matrix diberikan → telusuri path manual.
    Jika tidak → gunakan A*.
    """
    if manual_matrix is not None:
        # PENTING: Sekarang kita memasukkan parameter 'start' ke dalam parser
        waypoints = parse_manual_path(manual_matrix, start)
        if waypoints:
            print(f"[PATH] Menggunakan path manual (Traced): {len(waypoints)} waypoint")
            return waypoints
        else:
            print("[PATH] GAGAL membaca matriks manual! Pastikan start robot ditandai angka 1.")

    print(f"[PATH] Menjalankan A* dari {start} ke {goal} (margin={obstacle_margin})")
    path = astar(start, goal, obstacle_margin)

    if path:
        print(f"[PATH] A* berhasil: {len(path)} langkah")
    else:
        print(f"[PATH] A* GAGAL — tidak ada jalur!")

    return path


# ── Utilitas path ─────────────────────────────────────────

def path_to_directions(path: list) -> list:
    """
    Konversi list (row, col) ke list arah gerak.

    Returns:
        list of str: 'N', 'S', 'E', 'W'
        (N=atas/berkurang row, S=bawah, E=kanan, W=kiri)
    """
    directions = []
    for i in range(1, len(path)):
        dr = path[i][0] - path[i-1][0]
        dc = path[i][1] - path[i-1][1]
        if dr == -1:
            directions.append('N')
        elif dr == 1:
            directions.append('S')
        elif dc == 1:
            directions.append('E')
        elif dc == -1:
            directions.append('W')
    return directions

def path_to_relative_commands(path: list, initial_heading: str = 'S') -> list:
    """
    Konversi list (row, col) ke list perintah relatif robot.
    Output: 'F' (Maju 1 sel), 'RL' (Putar Kiri 90°), 'RR' (Putar Kanan 90°).
    """
    if len(path) < 2:
        return []

    # 1. Dapatkan arah mata angin absolut terlebih dahulu (N, S, E, W)
    abs_dirs = []
    for i in range(1, len(path)):
        dr = path[i][0] - path[i-1][0]
        dc = path[i][1] - path[i-1][1]
        if dr == -1: abs_dirs.append('N')
        elif dr == 1: abs_dirs.append('S')
        elif dc == 1: abs_dirs.append('E')
        elif dc == -1: abs_dirs.append('W')

    # 2. Terjemahkan ke perintah kemudi (Egocentric)
    commands = []
    current_heading = initial_heading
    dirs_ccw = ['N', 'W', 'S', 'E'] # Urutan Berlawanan Arah Jarum Jam (Kiri)

    for target_dir in abs_dirs:
        idx_curr = dirs_ccw.index(current_heading)
        idx_next = dirs_ccw.index(target_dir)

        # Selisih indeks untuk menentukan belokan
        diff = (idx_next - idx_curr) % 4

        if diff == 1:
            commands.append('RL') # Kiri 90°
        elif diff == 2:
            commands.append('RR') # Putar Balik 180° (Kanan 2x)
            commands.append('RR') 
        elif diff == 3:
            commands.append('RR') # Kanan 90°

        # Setelah menghadap arah yang benar, perintahkan maju 1 kotak
        commands.append('F')
        current_heading = target_dir

    return commands


def direction_to_yaw(direction: str) -> float:
    """
    Konversi arah ke sudut yaw (radian).
    Asumsi: robot menghadap East (kanan) saat yaw = 0.
    """
    mapping = {
        'E': 0.0,
        'N': 1.5708,    # +90° = kiri dalam konvensi ROS
        'W': 3.1416,    # 180°
        'S': -1.5708,   # -90° = kanan
    }
    return mapping.get(direction, 0.0)


if __name__ == '__main__':
    # Test A*
    from grid_map import START_ROBOT2, FINISH_ROBOT2, print_map
    path = get_path(START_ROBOT2, FINISH_ROBOT2, manual_matrix=MANUAL_PATH, obstacle_margin=0)
    
    print(f"\nPath: {path}")
    commands = path_to_relative_commands(path, initial_heading='S')
    print(f"Perintah Kemudi: {commands}")
    print_map(path=path, robot_pos=START_ROBOT2)
