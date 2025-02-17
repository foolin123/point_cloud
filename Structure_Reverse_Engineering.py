import rhinoscriptsyntax as rs
import math
from Grasshopper import DataTree
import Rhino.Geometry as rg
from Grasshopper.Kernel.Data import GH_Path
from collections import defaultdict


#曲线方向
def dir(crv):
    tang =  crv.TangentAtStart
    dir = rg.Vector3d(round(tang.X,2),round(tang.Y,2),round(tang.Z,3))
    return dir

#自定义“L1范数”用于实现平行线的邻近排序
def get_L1(crv): 
    if type(crv) == rg.LineCurve:
        s, e = crv.PointAtStart, crv.PointAtEnd
        dir = crv.TangentAtStart
        dir.Rotate(math.pi*0.5-0.003,rg.Vector3d(0,0,1))

        L1 = (s.X + e.X)*dir.X +(s.Y + e.Y)*dir.Y

        return L1

#判断线段是否共线
def colinear(crv_a, crv_b):
    a_s, a_e ,b_s = crv_a.PointAtStart ,crv_a.PointAtEnd, crv_b.PointAtStart
    cross = (a_e.X - a_s.X)*(b_s.Y - a_s.Y) - (b_s.X - a_s.X)*(a_e.Y - a_s.Y)
    co = round(cross,2) == 0
    return co 

#断线连接的判断条件，可优化！！！
def if_join(crv_a, crv_b, max_val = 349, min_val = 0):
    a_e ,b_s = crv_a.PointAtEnd, crv_b.PointAtStart
    dis = rs.Distance(a_e,b_s)
    if_join = dis >= min_val and dis <= max_val
    return if_join

#首位相连组合曲线
def join(crv_a, crv_b): 
    return rg.LineCurve(crv_a.PointAtStart, crv_b.PointAtEnd)

#获取曲线中点
def mid(crv):
    s, e = crv.PointAtStart, crv.PointAtEnd
    mid = rg.Point3d((s.X + e.X)/2,(s.Y + e.Y)/2, (s.Z + e.Z)/2)
    return mid

class BEAM:
    def __init__(self,crvs):
        self.crvs = crvs 

    #获取单线方向以实现平行分类
    def att_dirs(self): 
        dirs = []

        for crv in self.crvs:
            _dir = dir(crv)
            dirs.append(_dir)

        return dirs

    #获取L1范数
    def att_L1(self): #
        L1 = []

        for crv in self.crvs:
            L1.append(get_L1(crv))

        return L1

    #初始数据集（方向，曲线，位置）构建
    def data_set(self):
        data = []

        crvs = self.crvs
        dir = self.att_dirs()
        l1 = self.att_L1()

        for i in range(len(crvs)):
            data.append([dir[i], crvs[i], l1[i]])

        return data

    #根据单线方向分类
    def group(self, group_index = 0):
        grouped_data = defaultdict(list)

        for item in self.data_set():
            grouped_data[item[group_index]].append(item)
        
        return grouped_data
    
    #分方向实现邻近排序，返回（方向，排序后的曲线）
    def sort(self, sort_index = 2, get_index =1):
        sorted_data = []
        i = 0

        for vec,dir_crvs in self.group().items():
            sorted_data.append([])
            sorted_data[i] = sorted(dir_crvs,key=lambda x: x[sort_index])
            i += 1
            
        modified_data = [[row[get_index] for row in layer] for layer in sorted_data]
        return modified_data

    #连接分组排序后同属一根梁的断线
    #需注意最大连接距离与搜索最大步数（mn），前者详if_join()
    def crv_join(self, sorted_crvs):
        for dir, crvs in enumerate(sorted_crvs):
            i = 0
            while i < len(crvs)-1:
                j = i+1
                while j < i+7 and j < len(crvs):
                    if colinear(crvs[i],crvs[j]) and if_join(crvs[i],crvs[j]):
                        crvs[i] = join(crvs[i],crvs[j])
                        del crvs[j]
                        j -= 1
                    j +=1
                i += 1

        return sorted_crvs
            
    #根据（中点位置接近 and 长度接近）两两分组        
    def Match(self,joined_crvs):

        for index, crvs in enumerate(joined_crvs):
            matched_crvs = []
            i = 0
            while i < len(crvs)-1:
                j = i+1
                while j < len(crvs):
                    loc_tol = mid(crvs[i]).DistanceTo(mid(crvs[j])) < 900
                    length_tol = abs(crvs[i].Domain[1] - crvs[j].Domain[1]) < 0.5*max(crvs[i].Domain[1],crvs[j].Domain[1])
                    
                    if loc_tol and length_tol:
                        matched_crvs.append([crvs[i],crvs.pop(j)])
                        break
                    j += 1
                i +=1

            joined_crvs[index] = matched_crvs

        return joined_crvs

    #输出树形数据实现可视化
    def Show(self, data):
        Show_crv = DataTree[rg.LineCurve]()

        for i,val in enumerate(data):
            for j,matched_crv in enumerate(val):
                Show_crv.AddRange(matched_crv, GH_Path(i,j))
        
        return Show_crv
        
#预处理
if _Run:
    BEAMS = BEAM(_crvs) #实例化
    sorted_data =BEAMS.sort() #1分组排序
    Direction = [dir(crvs[0]) for crvs in sorted_data] #返回不重复向量列表

    Join_crv = BEAMS.crv_join(sorted_data) #2焊接断线
    matched_crvs = BEAMS.Match(Join_crv) # 3两两配对

    Show_crv = BEAMS.Show(matched_crvs) 

