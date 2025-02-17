"""
3. 基于图结构的山地道路建模
"""

from collections import deque
import ghpythonlib.treehelpers as th
import Rhino.Geometry as rg
import rhinoscriptsyntax as rs

class Vertex:
    def __init__(self, index, point3d): 
        self.index = index
        self.point3d = point3d
        self.neighbors = []
    def add_neighbors(self, neighbors:list):
        for neighbor in neighbors:
            if neighbor not in self.neighbors:
                self.neighbors.append(neighbor)
class Edge:
    def __init__(self, vertex1, vertex2):
        self.vertices = [vertex1, vertex2]
        self.faces = []
    
    def linecurve(self, mesh):
        return rg.LineCurve(mesh.V[self.vertices[0]].point3d, mesh.V[self.vertices[1]].point3d)

    def add_face(self, face):
        self.faces.append(face)
    def remove_face(self, face):
        self.faces.remove(face)

class Face:
    def __init__(self, p1, p2, p3):
        self.triangle = [p1, p2, p3]

class Mesh:
    def __init__(self):
        self.V = {}
        self.E = {}
        self.F = {}

    def initial_mesh(self, faces, vertices):

        for face in faces:
            if len(face) != 3:
                raise ValueError(f'{face} is not a tri_mesh')

            # 创建并存储顶点
            A = self.V.get(face[0], Vertex(face[0], vertices[face[0]]))
            B = self.V.get(face[1], Vertex(face[1], vertices[face[1]]))
            C = self.V.get(face[2], Vertex(face[2], vertices[face[2]]))
            # 存储顶点
            self.V[face[0]] = A
            self.V[face[1]] = B
            self.V[face[2]] = C
            # 添加邻居
            A.add_neighbors([B.index, C.index])
            B.add_neighbors([A.index, C.index])
            C.add_neighbors([A.index, B.index])

            # 创建并存储边
            edge_AB = self.E.get(frozenset([face[0], face[1]]), Edge(face[0], face[1]))
            edge_AC = self.E.get(frozenset([face[0], face[2]]), Edge(face[0], face[2]))
            edge_BC = self.E.get(frozenset([face[1], face[2]]), Edge(face[1], face[2]))
            self.E[frozenset([face[0], face[1]])] = edge_AB
            self.E[frozenset([face[0], face[2]])] = edge_AC
            self.E[frozenset([face[1], face[2]])] = edge_BC
            
            # 创建并存储面，修改边的邻接面
            self.add_face(Face(A.index, B.index, C.index))

        return self
    

    #添加面
    def add_face(self, face):
        if frozenset(face.triangle) in self.F:
            raise ValueError('Duplicate face')

        self.F[frozenset(face.triangle)] = face
        #修改边的邻接表
        self.E[frozenset([face.triangle[0], face.triangle[1]])].add_face(face)
        self.E[frozenset([face.triangle[0], face.triangle[2]])].add_face(face)
        self.E[frozenset([face.triangle[1], face.triangle[2]])].add_face(face)
            
    #删除三角网格
    def remove_face(self, face):
        del self.F[frozenset(face.triangle)]
        #修改边的邻接表
        self.E[frozenset([face.triangle[0],face.triangle[1]])].remove_face(face)
        self.E[frozenset([face.triangle[0],face.triangle[2]])].remove_face(face)
        self.E[frozenset([face.triangle[1],face.triangle[2]])].remove_face(face)


    # 广度优先搜索曲线
    def bfs_curve(self, is_in_road_boundary, curve, sides:list, crv_start, threshold):
        # 初始化队列、已访问集合
        queue = deque([self.V[crv_start]])
        visited = set()
        new = set()
        while queue:

            node = queue.popleft()
            # print(f'node {node.index}', f'neighbors: {node.neighbors}')
            if node.index not in visited:
                visited.add(node.index)
                # 处理邻居节点
                self.process_neighbors(node, is_in_road_boundary, curve, sides, threshold, new, queue, visited)

        # 返回访问过的节点索引和新点
        road_pt = new | visited
        road_pt = list(road_pt)

        new_point = [self.V[index].point3d for index in road_pt]
        return new_point, road_pt

    # 处理邻居节点
    def process_neighbors(self, node, is_in_road_boundary, curve, sides, threshold, new, queue, visited):
        Neighbors = node.neighbors.copy()
        for neighbor_index in Neighbors:
            neighbor = self.V[neighbor_index]  # Get the neighbor vertex from index
            
            if neighbor.index not in visited and neighbor.index not in new and neighbor not in queue:
                # print(f'neighbor {neighbor_index}')
                # 判断是否在道路边界内
                if is_in_road_boundary(curve, neighbor.point3d, threshold):
                    queue.append(neighbor)
                else:
                    # 处理非道路边界节点
                    self.out_road_boundary(node, neighbor, sides, new, queue, visited)

    # 处理非道路边界节点
    def out_road_boundary(self, node, neighbor, sides, new, queue, visited):
        # 获取边线,面，及邻接点
        edge = self.E[frozenset([node.index, neighbor.index])]
        line = edge.linecurve(self)
        faces = edge.faces
        neighbor_a = (set(faces[0].triangle) - frozenset([node.index, neighbor.index])).pop()
        neighbor_b = (set(faces[1].triangle) - frozenset([node.index, neighbor.index])).pop() if len(faces) > 1 else None
        tolerance, overlap_tolerance = 2, 2

        # 查找交点
        intersection_point = None
        for side in sides:
            intersection_events = rg.Intersect.Intersection.CurveCurve(side, line, tolerance, overlap_tolerance)
            if intersection_events:
                intersection_point = intersection_events[0].PointA
        
        if intersection_point:
            # 添加新节点
            new_index = len(self.V)
            new_vertex = Vertex(new_index, intersection_point)
            new_vertex.add_neighbors([node.index, neighbor.index, neighbor_a, neighbor_b])
            self.V[new_index] = new_vertex
            
            new.add(new_index)

            # 添加点的邻接列表
            node.add_neighbors([new_index])
            neighbor.add_neighbors([new_index])
            self.V[neighbor_a].add_neighbors([new_index])
            if neighbor_b is not None:
                self.V[neighbor_b].add_neighbors([new_index])
            # 添加新边
            for new_neighbor in [node.index, neighbor.index, neighbor_a, neighbor_b]:
                if new_neighbor:
                    self.E[frozenset([new_index, new_neighbor])] = Edge(new_index, new_neighbor)

            # 添加新面
            self.add_face(Face(new_index, node.index, neighbor_a))
            self.add_face(Face(new_index, neighbor.index, neighbor_a))
            if neighbor_b is not None:
                self.add_face(Face(new_index, node.index, neighbor_b))
                self.add_face(Face(new_index, neighbor.index, neighbor_b))

            # 删除旧面
            for face in list(faces):
                self.remove_face(face)
            
            # 删除旧边
            if len(edge.faces) > 0:
                raise ValueError('unnaked edge')
            else:
                del self.E[frozenset([node.index, neighbor.index])]
                node.neighbors.remove(neighbor.index)

def is_in_road_boundary(curve, point, threshold):
    """
    通过点和道路中点的距离判断前者是否在道路范围内
    """
    point = rs.coerce3dpoint(point)
    
    t= curve.ClosestPoint(point)
    ClosestPoint = curve.PointAt(t[1])
    ClosestPoint = rg.Point3d(ClosestPoint.X, ClosestPoint.Y, 0)
    point = rg.Point3d(point.X, point.Y, 0)
    distance = point.DistanceTo(ClosestPoint)

    return  distance <= threshold

if __name__ == "__main__":
    face = [[f.A, f.B, f.C] for f in face]
    vertices = Vertices
    mesh_graph = Mesh().initial_mesh(face, vertices)

    threshold = width
    crv_start = nearest_point
    curve = rs.coercecurve(curve)
    sides = [rs.coercecurve(side) for side in sides]
    road_pts, I = mesh_graph.bfs_curve(is_in_road_boundary, curve, sides, crv_start, threshold)
    F = th.list_to_tree(road_pts)
    # mesh_graph.show() 
