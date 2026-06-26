"""미로 생성/탐색에 공통으로 쓰는 그리드 로직. generate_maze.py와 explore_maze.py가 같이 사용."""
import random

CELL_SIZE = 1.0

OPPOSITE = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
DELTAS = {'N': (0, -1), 'S': (0, 1), 'E': (1, 0), 'W': (-1, 0)}


def carve_maze(width, height, seed):
    """랜덤 Prim's 알고리즘으로 width x height 미로의 스패닝 트리를 생성."""
    random.seed(seed)
    grid = [[{'N': True, 'S': True, 'E': True, 'W': True} for _ in range(width)] for _ in range(height)]
    visited = [[False] * width for _ in range(height)]

    def neighbors(x, y):
        result = []
        for d, (dx, dy) in DELTAS.items():
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                result.append((d, nx, ny))
        return result

    visited[0][0] = True
    frontier = [(0, 0, d, nx, ny) for d, nx, ny in neighbors(0, 0)]

    while frontier:
        idx = random.randrange(len(frontier))
        x, y, d, nx, ny = frontier.pop(idx)
        if visited[ny][nx]:
            continue
        grid[y][x][d] = False
        grid[ny][nx][OPPOSITE[d]] = False
        visited[ny][nx] = True
        for d2, nx2, ny2 in neighbors(nx, ny):
            if not visited[ny2][nx2]:
                frontier.append((nx, ny, d2, nx2, ny2))
    return grid


def dfs_walk(grid, start=(0, 0)):
    """모든 칸을 한 번씩 방문하고 돌아오는 완전 탐색 경로(셀 좌표 리스트) 생성."""
    visited = set()
    path = [start]

    def visit(cell):
        x, y = cell
        visited.add(cell)
        for d, (dx, dy) in DELTAS.items():
            if not grid[y][x][d]:
                nxt = (x + dx, y + dy)
                if nxt not in visited:
                    path.append(nxt)
                    visit(nxt)
                    path.append(cell)

    visit(start)
    return path


def cell_center(cell):
    cx, cy = cell
    return (cx * CELL_SIZE + CELL_SIZE / 2, cy * CELL_SIZE + CELL_SIZE / 2)
