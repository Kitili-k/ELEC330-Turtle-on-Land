import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def create_occupancy_grid(pcd_file, resolution=0.1):
    
    point_cloud = o3d.io.read_point_cloud(pcd_file)
    points = np.asarray(point_cloud.points)

    
    min_bound = points[:, :2].min(axis=0)  
    max_bound = points[:, :2].max(axis=0)

    
    map_size = np.ceil((max_bound - min_bound) / resolution).astype(int)
    
    
    occupancy_grid = np.zeros(map_size)

    
    grid_coords = ((points[:, :2] - min_bound) / resolution).astype(int)

    
    for x, y in grid_coords:
        if 0 <= x < map_size[0] and 0 <= y < map_size[1]:
            occupancy_grid[x, y] = 1

    return occupancy_grid, min_bound, map_size

def save_pgm(filename, occupancy_grid, resolution, origin):

    pgm_map = np.ones_like(occupancy_grid) * 254
    pgm_map[occupancy_grid == 1] = 0

    pgm_map = pgm_map.astype(np.uint8)


    yaml_content = f"""image: {filename}
resolution: {resolution}
origin: [{origin[0]}, {origin[1]}, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""

    with open(filename, 'wb') as f:

        height, width = pgm_map.shape
        f.write(f'P5\n{width} {height}\n255\n'.encode())

        pgm_map.T.tofile(f)  


    yaml_filename = filename.rsplit('.', 1)[0] + '.yaml'
    with open(yaml_filename, 'w') as f:
        f.write(yaml_content)

def main():

    pcd_file = "map.pcd"
    resolution = 0.1  # 10cm per grid
    output_file = "map.pgm"


    occupancy_grid, origin, map_size = create_occupancy_grid(pcd_file, resolution)

    save_pgm(output_file, occupancy_grid, resolution, origin)


    plt.figure(figsize=(10, 10))
    plt.imshow(occupancy_grid.T, cmap='binary', origin='lower')
    plt.title('Occupancy Grid Map')
    plt.colorbar(label='Occupancy')
    plt.xlabel('X (grid cells)')
    plt.ylabel('Y (grid cells)')
    

    plt.text(0.02, 0.98, 
             f'Resolution: {resolution}m/cell\n'
             f'Origin: ({origin[0]:.2f}, {origin[1]:.2f})\n'
             f'Size: {map_size[0]}x{map_size[1]} cells',
             transform=plt.gca().transAxes, 
             verticalalignment='top')
    
    plt.show()

if __name__ == "__main__":
    main()