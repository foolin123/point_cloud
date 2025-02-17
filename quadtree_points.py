import rhinoscriptsyntax as rs
import Rhino.Geometry as rh
from Grasshopper.Kernel.Data import GH_Path
from Grasshopper import DataTree
        
        
class Point:
	def __init__(self, x, y):
		self.x = x
		self.y = y
        
# 节点
class Node:
	def __init__(self, i, pos, data= None):
		self.index = i
		self.pos = pos
		self.data = data
		
# 四叉树类
class Quad:
	def __init__(self, topL, botR, threshold):
		self.topLeft = topL
		self.botRight = botR
		self.nodes = []
		self.topLeftTree = None
		self.topRightTree = None
		self.botLeftTree = None
		self.botRightTree = None
		self.threshold = threshold
        
	# 插入节点
	def insert(self, node):
		if node is None:
			return
        
		if not self.inBoundary(node.pos):
			return
        
		# 不能进一步细分四叉树时插入数据
		if abs(self.topLeft.x - self.botRight.x) <= self.threshold and abs(self.topLeft.y - self.botRight.y) <= self.threshold:
			self.nodes.append(node)
			return
        
		if (self.topLeft.x + self.botRight.x) / 2 >= node.pos.x:
			# topLeftTree
			if (self.topLeft.y + self.botRight.y) / 2 >= node.pos.y:
				if self.topLeftTree is None:
					self.topLeftTree = Quad(self.topLeft, Point((self.topLeft.x + self.botRight.x) / 2, (self.topLeft.y + self.botRight.y) / 2),self.threshold)
				self.topLeftTree.insert(node)
			# botLeftTree
			else:
				if self.botLeftTree is None:
					self.botLeftTree = Quad(Point(self.topLeft.x, (self.topLeft.y + self.botRight.y) / 2), Point((self.topLeft.x + self.botRight.x) / 2,self.botRight.y),self.threshold)
				self.botLeftTree.insert(node)
		else:
			# topRightTree
			if (self.topLeft.y + self.botRight.y) / 2 >= node.pos.y:
				if self.topRightTree is None:
					self.topRightTree = Quad(Point((self.topLeft.x + self.botRight.x) / 2, self.topLeft.y), Point(self.botRight.x, (self.topLeft.y + self.botRight.y) / 2),self.threshold)
				self.topRightTree.insert(node)
			# botRightTree
			else:
				if self.botRightTree is None:
					self.botRightTree = Quad(Point((self.topLeft.x + self.botRight.x) / 2, (self.topLeft.y + self.botRight.y) / 2), self.botRight,self.threshold)
				self.botRightTree.insert(node)
        				
	# 检查是否在当前四叉树范围内
	def inBoundary(self, p, p1=None ,p2=None):
		if p1:
			return p.x >= p1.x and p.x <= p2.x and p.y >= p1.y and p.y <= p2.y
		return p.x >= self.topLeft.x and p.x <= self.botRight.x and p.y >= self.topLeft.y and p.y <= self.botRight.y
        
	def traverse(self):
		"""遍历四叉树，返回所有节点"""
		result = []
		self._traverseHelper(result)
		return result
        	    
	def _traverseHelper(self, result):
		result.extend(self.nodes)
        	    
		if self.topLeftTree is not None:
			self.topLeftTree._traverseHelper(result)
		if self.topRightTree is not None:
			self.topRightTree._traverseHelper(result)
		if self.botLeftTree is not None:
			self.botLeftTree._traverseHelper(result)
		if self.botRightTree is not None:
			self.botRightTree._traverseHelper(result)
        	        
	def intersect(self, p1, p2):
		return not (p1.x > self.botRight.x or p2.x < self.topLeft.x or p1.y > self.botRight.y or p2.y < self.topLeft.y)
        	    
	def search_area(self, p1, p2):
		search_result = []
		self._search_areaHelper(search_result, p1, p2)
		return search_result
        	
	def _search_areaHelper(self, search_result, p1, p2):
		if self.intersect(p1,p2): 
			if len(self.nodes) > 0: 
				#都在检索范围内
				if self.inBoundary(self.topLeft, p1, p2) and self.inBoundary(self.botRight, p1, p2):
					search_result.extend(self.nodes)
				else: #部分在
					for node in self.nodes:
						if self.inBoundary(node.pos,p1,p2):
							search_result.append(node)
			else: #父节点的node节点为空
				if self.topLeftTree:
					self.topLeftTree._search_areaHelper(search_result, p1, p2)
				if self.topRightTree:
					self.topRightTree._search_areaHelper(search_result, p1, p2)
				if self.botLeftTree:
					self.botLeftTree._search_areaHelper(search_result, p1, p2)
				if self.botRightTree:
					self.botRightTree._search_areaHelper(search_result, p1, p2)

	def search_nearest(self, target):
		"""搜索最近的节点"""
		nearest_point = None
		nearest_distance = float('inf')
		nearest_point, nearest_distance = self._search_nearestHelper(target, nearest_distance, nearest_point)
		return nearest_point

	def _search_nearestHelper(self, target, nearest_distance, nearest_point):
		if not self.intersect(Point(target.x - nearest_distance, target.y - nearest_distance), 
							Point(target.x + nearest_distance, target.y + nearest_distance)):
			return nearest_point, nearest_distance
		
		if self.nodes:
			for node in self.nodes:
				distance = self.distance(node.pos, target)
				if distance < nearest_distance:
					nearest_distance = distance
					nearest_point = node
		else:
			if self.topLeftTree is not None:
				nearest_point, nearest_distance = self.topLeftTree._search_nearestHelper(
					target, nearest_distance, nearest_point)
			if self.topRightTree is not None:
				nearest_point, nearest_distance = self.topRightTree._search_nearestHelper(
					target, nearest_distance, nearest_point)
			if self.botLeftTree is not None:
				nearest_point, nearest_distance = self.botLeftTree._search_nearestHelper(
					target, nearest_distance, nearest_point)
			if self.botRightTree is not None:
				nearest_point, nearest_distance = self.botRightTree._search_nearestHelper(
					target, nearest_distance, nearest_point)
	
		return nearest_point, nearest_distance

	def distance(self, p1, p2):
		"""计算两点之间的距离"""
		return ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2) ** 0.5
		
	def optimized_search_nearest(self, target):
		"""优化的最近点搜索"""
		# 1. 找到目标点所在的叶子节点
		current = self
		path = [current]
		
		while current and not current.nodes:  # Check if current is not None
			mid_x = (current.topLeft.x + current.botRight.x) / 2
			mid_y = (current.topLeft.y + current.botRight.y) / 2
			
			if mid_x >= target.x:
				if mid_y >= target.y:
					current = current.topLeftTree
				else:
					current = current.botLeftTree
			else:
				if mid_y >= target.y:
					current = current.topRightTree
				else:
					current = current.botRightTree
			path.append(current)
		
		# 2. 在叶子节点中找到初始最近点
		nearest = None
		min_dist = float('inf')
		
		if current is not None:  # Check if current is not None
			for node in current.nodes:
				dist = self.distance(node.pos, target)
				if dist < min_dist:
					min_dist = dist
					nearest = node

			for quad in reversed(path):
				for tree in [quad.topLeftTree, quad.topRightTree, quad.botLeftTree, quad.botRightTree]:
					if tree and tree not in path:
						temp_nearest, temp_dist = tree._search_nearestHelper(target, min_dist, nearest)
						if temp_dist < min_dist:
							min_dist = temp_dist
							nearest = temp_nearest
		
		else:
			# 处理 current 为 None 的情况，回溯到父节点并搜索兄弟节点
			for quad in reversed(path[:-1]):

				for tree in [quad.topLeftTree, quad.topRightTree, quad.botLeftTree, quad.botRightTree]:
					if tree and tree not in path:
						temp_nearest, temp_dist = tree._search_nearestHelper(target, min_dist, nearest)
						if temp_dist < min_dist:
							min_dist = temp_dist
							nearest = temp_nearest
						
		return nearest
		
def get_xy(Point3d):
	return [Point3d.X, Point3d.Y]
        
def bounding_box(geo):
	if type(geo) == list:
		bounding =[get_xy(rh.BoundingBox(geo).Min), get_xy(rh.BoundingBox(geo).Max)]
	else:
		bounding = [get_xy(rh.GeometryBase.GetBoundingBox(geo,False).Min), get_xy(rh.GeometryBase.GetBoundingBox(geo,False).Max)]
	p1 = Point(bounding[0][0],bounding[0][1])
	p2 = Point(bounding[1][0],bounding[1][1])
	return p1, p2

# 预处理

pts = _pts
N = _step
P = []

for p in pts:
	P.append(rs.coerce3dpoint(p))
        
Botleft, Topright = bounding_box(P)
threshold = (0.5**N)*(0.5*abs(Botleft.x+Botleft.y - Topright.x -Topright.y))
Tree = Quad(Botleft, Topright,threshold)
        
for i, p in enumerate(P):
	p_xy = get_xy(p)
	a = Node(i,Point(p_xy[0], p_xy[1]),p)
	Tree.insert(a)
        
Quadtree = Tree
        
if traversal:
    step = len(_pts)
    print(step)
    if tra_ratio :
        step = int(step * min(100,tra_ratio)/100) - 1
        print(step)
    PointTree = DataTree[object]()
    traversal_result = [i.data for i in Tree.traverse()[:step]]

def get_xy(Point3d):
	return [Point3d.X, Point3d.Y]

target = get_xy(rs.coerce3dpoint(target))
target = Node(Point(target[0], target[1]),None)
nearest_point = Quadtree.optimized_search_nearest(target.pos)
P = nearest_point.data
i = nearest_point.index
