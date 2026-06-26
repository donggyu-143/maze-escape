"""미로를 랜덤 Prim's 알고리즘으로 생성해서 Gazebo SDF .world 파일로 출력.

recursive backtracking은 한 방향으로 길게 뻗는 직선 복도가 많이 생겨서,
대신 랜덤 Prim's을 사용 — 분기/꺾임이 훨씬 자주 생겨서 더 미로답게 나옴.

연속된 벽 칸은 개별 박스 대신 하나의 긴 박스로 합쳐서 모델 개수를 줄임
(Gazebo Classic은 <model> 개수가 늘어날수록 렌더링 부하가 커짐 — 소프트웨어
렌더링 환경에서는 이게 체감상 가장 큰 영향을 줌).
"""
import argparse

from maze_escape.maze_grid import CELL_SIZE, carve_maze

WALL_THICKNESS = 0.05
WALL_HEIGHT = 0.5


def build_wall_grids(grid, width, height):
    # h_wall[r][c]: row 경계 r(0..height)와 열 c 사이의 가로벽
    h_wall = [[True] * width for _ in range(height + 1)]
    # v_wall[r][c]: 행 r과 열 경계 c(0..width) 사이의 세로벽
    v_wall = [[True] * (width + 1) for _ in range(height)]

    for y in range(height):
        for x in range(width):
            if not grid[y][x]['N'] and y > 0:
                h_wall[y][x] = False
            if not grid[y][x]['W'] and x > 0:
                v_wall[y][x] = False
    return h_wall, v_wall


def merge_runs(flags):
    """[True, True, False, True] -> [(start=0, length=2), (start=3, length=1)]"""
    runs = []
    start = None
    for i, flag in enumerate(list(flags) + [False]):
        if flag and start is None:
            start = i
        elif not flag and start is not None:
            runs.append((start, i - start))
            start = None
    return runs


def wall_box_sdf(name, x, y, size_x, size_y):
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {WALL_HEIGHT / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{size_x:.3f} {size_y:.3f} {WALL_HEIGHT:.3f}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{size_x:.3f} {size_y:.3f} {WALL_HEIGHT:.3f}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
'''


def generate_world(width, height, seed, exit_side='E'):
    grid = carve_maze(width, height, seed)
    h_wall, v_wall = build_wall_grids(grid, width, height)

    # 출구: 목표 셀(우하단)의 바깥쪽 경계벽 하나를 뚫어줌
    if exit_side == 'E':
        v_wall[height - 1][width] = False
    elif exit_side == 'S':
        h_wall[height][width - 1] = False

    walls_xml = []
    idx = 0

    # 가로벽: 각 행(r)에서 연속된 칸을 하나의 긴 박스로 합침
    for r in range(height + 1):
        for start, length in merge_runs(h_wall[r]):
            cx = start * CELL_SIZE + length * CELL_SIZE / 2
            cy = r * CELL_SIZE
            walls_xml.append(wall_box_sdf(f'wall_h_{idx}', cx, cy, length * CELL_SIZE, WALL_THICKNESS))
            idx += 1

    # 세로벽: 각 열(c)에서 연속된 칸을 하나의 긴 박스로 합침
    for c in range(width + 1):
        column = [v_wall[r][c] for r in range(height)]
        for start, length in merge_runs(column):
            cx = c * CELL_SIZE
            cy = start * CELL_SIZE + length * CELL_SIZE / 2
            walls_xml.append(wall_box_sdf(f'wall_v_{idx}', cx, cy, WALL_THICKNESS, length * CELL_SIZE))
            idx += 1

    world = f'''<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <scene>
      <shadows>false</shadows>
    </scene>

    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose frame=''>{width * CELL_SIZE / 2:.2f} {height * CELL_SIZE / 2:.2f} {max(width, height) * 1.5:.2f} 0 1.5708 0</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>

    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <precon_iters>0</precon_iters>
          <sor>1.400000</sor>
          <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000.000000</contact_max_correcting_vel>
          <contact_surface_layer>0.01000</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

{''.join(walls_xml)}
  </world>
</sdf>
'''
    return world


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--width', type=int, default=10)
    parser.add_argument('--height', type=int, default=10)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--output', type=str, default='worlds/maze.world')
    args = parser.parse_args()

    world = generate_world(args.width, args.height, args.seed)
    with open(args.output, 'w') as f:
        f.write(world)
    print(f'생성 완료: {args.output} ({args.width}x{args.height}, seed={args.seed})')


if __name__ == '__main__':
    main()
