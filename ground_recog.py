import numpy as np
from scipy.signal import convolve2d
from sklearn.cluster import KMeans

import rhinoscriptsyntax as rs
import ghpythonlib.treehelpers as th

# 初始坐标值
def get_xyz(Point3d):
    return [round(Point3d.X,1), round(Point3d.Y,1), round(Point3d.Z,1)]

def process_image_list(image_list,grid):
    """
    将一维图像数据列表转换为二维ndarray，并实现xy坐标的映射。
    """
    # 找到最大x和y坐标以确定ndarray的大小
    max_x = max(x for x, y, fxy in image_list)
    min_x = min(x for x, y, fxy in image_list)
    max_y = max(y for x, y, fxy in image_list)
    min_y = min(y for x, y, fxy in image_list)

    # 初始化ndarray
    image_array = np.zeros((int((max_x-min_x)/grid + 1), int((max_y-min_y)/grid + 1)), dtype=float)
    
    # 调试信息
    print(f"Grid: {grid}, Image Array Shape: {image_array.shape}")

    image_dict = {}
    # 将列表中的数据映射到ndarray中
    for index, image in enumerate(image_list):
        x_index = int((image[0] - min_x) / grid)
        y_index = int((image[1] - min_y) / grid)
        
        image_array[x_index, y_index] = image[2]
        image_dict[(x_index, y_index)] = index
    
    # print(image_array[50:60,50:60])
    return image_array, min_x, min_y, image_dict

def compute_gradient(image_array):
    """
    使用Sobel算子计算图像每个像素的梯度幅值。
    梯度幅值是图像强度在每个像素处的变化率。
    """    
    # Sobel算子
    sobel_x = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])
    sobel_y = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]])
    
    # 计算梯度
    gradient_x = convolve2d(image_array, sobel_x, mode='same', boundary='wrap')
    gradient_y = convolve2d(image_array, sobel_y, mode='same', boundary='wrap')
    
    # 计算梯度幅值
    gradient_array = np.sqrt(gradient_x ** 2 + gradient_y ** 2)
    gradient_array = np.round(gradient_array, 1)

    return gradient_array

def generate_seed_points_kmeans(gradient_array, n_clusters, seed = 0):
    """
    使用 K-means 聚类生成生长点。
    """
    # 将梯度幅值展平并去除零值
    flat_gradient = gradient_array.flatten()
    non_zero_indices = np.nonzero(flat_gradient)[0]
    non_zero_values = flat_gradient[non_zero_indices].reshape(-1, 1)
    
    if len(non_zero_values) < n_clusters:
        raise ValueError(f"非零梯度值数量({len(non_zero_values)})小于请求的聚类数量({n_clusters})")

    # 使用 K-means 聚类，设置初始化方法和最大迭代次数
    kmeans = KMeans(n_clusters=n_clusters, init='random', max_iter=500, random_state=seed)
    kmeans.fit(non_zero_values)

    # 获取聚类中心
    cluster_centers = kmeans.cluster_centers_.flatten()

    # 根据聚类中心生成生长点
    seed_points = []
    for center in cluster_centers:
        # 找到与聚类中心最接近的非零梯度值
        distances = np.abs(non_zero_values - center)
        closest_idx = np.argmin(distances)
        original_idx = non_zero_indices[closest_idx]
        x, y = divmod(original_idx, gradient_array.shape[1])
        seed_points.append((x, y))

    return seed_points

def generate_random_seeds(image_array, n_seeds):
    """
    在array.shape范围内随机生成N个seeds，作为生长点。
    
    参数:
    image_array: 输入图像数组
    n_seeds: 需要生成的种子点数量
    
    返回:
    seeds: 种子点列表，每个点为(x,y)坐标元组
    """
    height, width = image_array.shape
    seeds = []
    for _ in range(n_seeds):
        x = np.random.randint(0, height)
        y = np.random.randint(0, width)
        seeds.append((x, y))
    return seeds

def dfs_array_from_seeds(image_array, gradient_array, seed_points, gradient_range):
    """
    根据生长点进行深度优先搜索，返回多个连通邻域。
    """
    neighborhoods = []  # 用于存储所有连通邻域
    visited_global = set()  # 用于跟踪全局访问的点

    for start_point in seed_points:
        if start_point[0] < 0 or start_point[0] >= image_array.shape[0] or start_point[1] < 0 or start_point[1] >= image_array.shape[1]:
            raise ValueError(f"起始点 {start_point} 不在图像范围内。")
        
        stack = [start_point]
        visited = np.zeros(image_array.shape, dtype=bool)
        neighborhood = []

        while stack:
            x, y = stack.pop()
            if not visited[x, y]:
                visited[x, y] = True
                neighborhood.append((x, y))
                visited_global.add((x, y))  # 添加到全局访问集合
                for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:  # M-connected neighborhood
                    nx, ny = x + dx, y + dy
                    if (0 <= nx < image_array.shape[0] and 
                        0 <= ny < image_array.shape[1] and 
                        gradient_range[0] <= gradient_array[nx, ny] <= gradient_range[1] and 
                        (nx, ny) not in visited_global):  # 检查是否已访问
                        stack.append((nx, ny))
        
        neighborhoods.append(neighborhood)  # 将当前连通邻域添加到列表中

    return neighborhoods  # 返回所有连通邻域

def get_original_points(image_dict, neighborhoods):
    """
    根据DFS返回的邻域点，找到原始图像列表中的对应点。
    """
    original_points = []
    for neighborhood in neighborhoods:
        if len(neighborhood) >20:
            original_points.append([image_dict[xy] for xy in neighborhood if xy in image_dict])
    return original_points

try:
    if  len(pts) == 0:
        raise ValueError('empty cloud')
    
    image_list = [get_xyz(rs.coerce3dpoint(p)) for p in pts]

    image_array, min_x, min_y, image_dict = process_image_list(image_list,grid)
    gradient_array = compute_gradient(image_array)

    # click = get_xyz(rs.coerce3dpoint(CLICK))[:2]
    # start_point = [int((click[0]-min_x)/grid), int((click[1]-min_y)/grid)]
    # seed_points = generate_seed_points_kmeans(gradient_array, n_clusters,seed)
    seed_points = generate_random_seeds(image_array, n_seeds)
    gradient_range = Range

    neighborhoods = dfs_array_from_seeds(image_array, gradient_array, seed_points, gradient_range)

    original_points = get_original_points(image_dict, neighborhoods)
    # original_points = [list(map(int, points)) for points in original_points if points]
    original_points = th.list_to_tree(original_points)
    

except ValueError as e:
    print(e)