"""
grid_map.py
===========
Definisi arena dalam bentuk grid 0.4m x 0.4m per sel.
0 = kosong, 1 = obstacle, 2 = start, 3 = finish

Koordinat grid: (row, col)
  row meningkat ke bawah (arah Selatan)
  col meningkat ke kanan (arah Timur)

Arena fisik: 3.6m x 4.8m → 9 col x 12 row (@ 0.4m/sel)
"""

import numpy as np

# ── Ukuran sel ─────────────────────────────────────────────
CELL_SIZE = 0.39   # meter per sel

# ── Peta arena (0=kosong, 1=obstacle) ──────────────────────
# Col:  0  1  2  3  4  5  6  7  8
ARENA_MAP = np.array([
    [0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0],   # row 0
    [0, 0, 1, 0, 0, 0, 1, 0, 0],   # row 1  ← obstacle atas
    [0, 0, 0, 0, 0, 0, 0, 0, 0],   # row 2
    [0, 0, 0, 0, 0, 0, 0, 0, 0],   # row 3
    [0, 1, 0, 0, 1, 0, 0, 1, 0],   # row 4  ← obstacle tengah atas
    [0, 0, 0, 0, 0, 0, 0, 0, 0],   # row 5
    [0, 0, 0, 0, 0, 0, 0, 0, 0],   # row 6
    [0, 0, 1, 0, 0, 0, 1, 0, 0],   # row 7  ← obstacle tengah bawah
    [0, 0, 0, 0, 0, 0, 0, 0, 0],   # row 8
    [0, 0, 0, 0, 0, 0, 0, 0, 0],   # row 9
    [0, 0, 0, 0, 0, 0, 0, 0, 0],   # row 10
    [0, 0, 1, 0, 0, 0, 1, 0, 0],   # row 11 ← obstacle bawah
], dtype=np.int8)

ROWS, COLS = ARENA_MAP.shape

# ── Posisi start dan finish ─────────────────────────────────
# Robot 1: baris atas, Robot 2: baris bawah
START_ROBOT1 = (0, 2)    # pojok kiri atas
START_ROBOT2 = (0, 6)    # tengah kiri (estimasi Robot 2)
START_ROBOT3 = (0, 2)    # pojok kiri atas
START_ROBOT4 = (0, 6)    # tengah kiri (estimasi Robot 2)

# Finish: pojok kanan
FINISH_ROBOT1 = (10, 2)   # pojok kanan atas
FINISH_ROBOT2 = (10, 6)   # tengah kanan
FINISH_ROBOT3 = (10, 6)   # pojok kanan atas
FINISH_ROBOT4 = (10, 2)   # tengah kanan

# ── Path manual (opsional) ─────────────────────────────────
# Isi dengan matriks 12x9 dimana 1 = waypoint yang harus dilalui
# Dibaca dari atas ke bawah, kiri ke kanan
# None berarti gunakan A* otomatis
MANUAL_PATH_ROBOT1 = None
MANUAL_PATH_ROBOT2 = None

# Contoh path manual Robot 1 (row 0, lurus ke kanan):
# MANUAL_PATH_ROBOT1 = np.array([
#     [1, 1, 0, 1, 1, 1, 0, 1, 1],
#     [0, 0, 0, 0, 0, 0, 0, 0, 0],
#     ...
# ])


def cell_to_world(row: int, col: int) -> tuple:
    """Konversi index grid ke koordinat dunia (meter)."""
    x = col * CELL_SIZE + CELL_SIZE / 2
    y = row * CELL_SIZE + CELL_SIZE / 2
    return (x, y)


def world_to_cell(x: float, y: float) -> tuple:
    """Konversi koordinat dunia ke index grid (row, col)."""
    col = int(x / CELL_SIZE)
    row = int(y / CELL_SIZE)
    col = max(0, min(COLS - 1, col))
    row = max(0, min(ROWS - 1, row))
    return (row, col)


def is_free(row: int, col: int) -> bool:
    """Cek apakah sel bebas (tidak ada obstacle)."""
    if 0 <= row < ROWS and 0 <= col < COLS:
        return ARENA_MAP[row, col] == 0
    return False


def print_map(path=None, robot_pos=None):
    """Print peta ke terminal dengan visualisasi ASCII."""
    symbols = {0: '.', 1: '█', 2: 'S', 3: 'F', 4: '*', 5: 'R'}
    display = np.full((ROWS, COLS), '.', dtype='<U2')

    for r in range(ROWS):
        for c in range(COLS):
            if ARENA_MAP[r, c] == 1:
                display[r, c] = '█'

    if path:
        for (r, c) in path:
            if display[r, c] == '.':
                display[r, c] = '*'

    if robot_pos:
        r, c = robot_pos
        display[r, c] = 'R'

    print(f"\n  {'Col':>4}", end='')
    for c in range(COLS):
        print(f" {c:2}", end='')
    print()

    for r in range(ROWS):
        print(f"Row {r:2}:", end='')
        for c in range(COLS):
            print(f" {display[r, c]:2}", end='')
        print()
    print()
