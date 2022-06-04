'''
Author: Harryhht
Date: 2021-09-11 15:21:36
LastEditTime: 2021-09-18 14:17:02
LastEditors: Please set LastEditors
Description: In User Settings Edit
FilePath: \src\decoder.py
'''

from xml.dom.minidom import parse
import xml.dom.minidom
import bisect
import math
import sys

# sys.setrecursionlimit(10000000)

# This is for timing


def timer(func):
    def func_wrapper(*args, **kwargs):
        from time import time
        time_start = time()
        result = func(*args, **kwargs)
        time_end = time()
        time_spend = time_end - time_start
        print('\n{0} cost time {1} s'.format(func.__name__, time_spend))
        return result
    return func_wrapper


# 节点数据结构


class node():
    def __init__(self, nodeid=-1, lat=-91.0, lon=-1.0):
        self.id = nodeid
        self.lat = lat
        self.lon = lon
        self.connection_nodes_type1 = []
        self.connection_nodes_type2 = []
        self.pre = None
        self.distance = sys.maxsize
        # self.type=None

    def id_in_connection(self, cn, id):
        if not cn:
            return False
        pos = bisect.bisect_left(cn, (id, None, None))
        if pos >= len(cn):
            return False
        # print(len(self.connection_nodes))
        # print(pos)
        # print(self.connection_nodes])
        if cn[pos][0] == id:
            return True
        else:
            return False

    def add_connection(self, node_id, nd, distance, azimuth, type):
        if not self.id_in_connection(self.connection_nodes_type1, node_id):
            bisect.insort(self.connection_nodes_type1,
                          (node_id, nd, distance, azimuth))
        if type == 2:
            if not self.id_in_connection(self.connection_nodes_type2, node_id):
                bisect.insort(self.connection_nodes_type2,
                              (node_id, nd, distance, azimuth))

    def __lt__(self, other):
        return self.id < other.id


# OSM数据解析器


class OSMParser():
    def __init__(self, datapath):
        self.nodes = []
        self.load(datapath)

    # def calculate_center(self,poslist):
    # 计算距离
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        # haversin
        '''
        p = 0.017453292519943295     #Pi/180
        a = 0.5 - math.cos((lat2 - lat1) * p)/2 + math.cos(lat1 * p) * \
                           math.cos(lat2 * p) * \
                                    (1 - math.cos((lon2 - lon1) * p)) / 2
        return 12742 * math.asin(math.sqrt(a))*1000
        '''
        lon_1, lat_1, lon_2, lat_2 = map(
            math.radians, [lon1, lat1, lon2, lat2])
        dlon = lon_2 - lon_1
        dlat = lat_2 - lat_1
        a = math.sin(dlat / 2) ** 2 + math.cos(lat_1) * \
            math.cos(lat_2) * math.sin(dlon / 2) ** 2
        c = 2 * math.asin(math.sqrt(a))
        r = 6371.393  # 地球半径

        mNear = c * r * 1000  # 单位为m的长度

        return mNear

    # 计算方向角（可用于优化最短路径，暂未使用）
    def calculate_azimuth(self, lat1, lon1, lat2, lon2):
        lon1_rad, lat1_rad, lon2_rad, lat2_rad = map(
            math.radians, [lon1, lat1, lon2, lat2])

        y = math.sin(lon2_rad - lon1_rad) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - \
            math.sin(lat1_rad) * math.cos(lat2_rad) * \
            math.cos(lon2_rad - lon1_rad)

        brng = math.atan2(y, x) * 180 / math.pi

        return float((brng + 360.0) % 360.0)

    # 二分法查找节点在list中位置
    def nodes_loc(self, nodelist, id):
        pos = bisect.bisect_left(nodelist, (id, node()))
        if(nodelist[pos][0] == id):
            return pos
        else:
            # print(id,nodelist[pos+1][0])
            return None

    # 判断距离坐标最近的节点
    def nearest_node(self, nodelist, lat, lon):
        distlist = []
        for nd in nodelist:
            nd_lat, nd_lon = nd[1].lat, nd[1].lon
            distlist.append(self.calculate_distance(nd_lat, nd_lon, lat, lon))
        dist = min(distlist)
        pos = distlist.index(dist)
        out = nodelist[pos]
        return out[0], out[1], dist

    # 判断道路类型
    def highway_classifier(self, highway_tag):
        # 用于分析判断道路信息
        type2 = ['tertiary', 'residential', 'service',
                 'primary', 'secondary', 'unclassified']
        type1 = ['footway', 'track', 'path', 'living_street', 'pedestrian']
        if highway_tag in type2:
            return 2
        if highway_tag in type1:
            return 1

    # 连接两节点
    def nodes_connection_path(self, id1, id2, highway_tag):
        nodeslist = self.nodes
        pos1 = self.nodes_loc(nodeslist, id1)
        pos2 = self.nodes_loc(nodeslist, id2)
        nd1 = nodeslist[pos1][1]
        nd2 = nodeslist[pos2][1]
        # print(highway_tag)
        tp = self.highway_classifier(highway_tag)
        distance = self.calculate_distance(nd1.lat, nd1.lon, nd2.lat, nd2.lon)
        azimuth = self.calculate_azimuth(nd1.lat, nd1.lon, nd2.lat, nd2.lon)
        nd1.add_connection(id2, nd2, distance, azimuth, tp)
        nd2.add_connection(id1, nd1, distance, (azimuth+180) % 360, tp)

    # 去除没有道路连接的节点数据（不属于学校）
    @timer
    def nodes_dropna(self):
        self.nodes_con = [x for x in self.nodes if len(
            x[1].connection_nodes_type1)]
        self.nodes_con_pro = [x for x in self.nodes_con if len(
            x[1].connection_nodes_type2)]

    # 导入道路数据
    @timer
    def load_ways(self, OSM):
        # nodelist=self.nodes
        _ways = OSM.getElementsByTagName("way")
        for _way in _ways:
            flag = "None"
            name = None
            for tag in _way.getElementsByTagName("tag"):
                if tag.getAttribute("k") == "highway":
                    flag = "highway"
                    highway_tag = tag.getAttribute("v")
                if tag.getAttribute("k") == "building":
                    flag = "building"
                    building_tag = tag.getAttribute("v")
                if tag.getAttribute("k") == "name":
                    name = tag.getAttribute("v")

            if flag == "highway":
                pre = None
                for nd in _way.getElementsByTagName("nd"):
                    post = int(nd.getAttribute("ref"))
                    if pre is not None:
                        self.nodes_connection_path(post, pre, highway_tag)
                    pre = post

    # 导入建筑数据
    @timer
    def load_buildings(self, OSM):
        nodelist = self.nodes
        nodelist_con = self.nodes_con
        nodelist_con_pro = self.nodes_con_pro

        self.building_name_list = []
        self.building_info_list = []

        _ways = OSM.getElementsByTagName("way")
        for _way in _ways:
            flag = "None"
            name = None
            for tag in _way.getElementsByTagName("tag"):
                if tag.getAttribute("k") == "highway":
                    flag = "highway"
                    highway_tag = tag.getAttribute("v")
                if tag.getAttribute("k") == "building" or tag.getAttribute("k") == "sport":
                    flag = "building"
                    building_tag = tag.getAttribute("v")
                if tag.getAttribute("k") == "name":
                    name = tag.getAttribute("v")

            if flag == "building" and name is not None:
                nearest_node_id = None
                nearest_node = None
                nearest_distance = sys.maxsize
                nearest_node_id_pro = None
                nearest_node_pro = None
                nearest_distance_pro = sys.maxsize
                for nd in _way.getElementsByTagName("nd"):
                    _id = int(nd.getAttribute("ref"))
                    _pos = self.nodes_loc(nodelist, _id)
                    lat, lon = nodelist[_pos][1].lat, nodelist[_pos][1].lon
                    node_id, node, distance = self.nearest_node(
                        nodelist_con, lat, lon)
                    node_id_pro, node_pro, distance_pro = self.nearest_node(
                        nodelist_con_pro, lat, lon)
                    if distance < nearest_distance:
                        nearest_node_id = node_id
                        nearest_node = node
                        nearest_distance = distance
                    if distance_pro < nearest_distance_pro:
                        nearest_node_id_pro = node_id_pro
                        nearest_node_pro = node_pro
                        nearest_distance_pro = distance_pro

                self.building_name_list.append(name)
                self.building_info_list.append(
                    (name, (nearest_node_id, nearest_node), (nearest_node_id_pro, nearest_node_pro)))
                # print(name, nearest_node_id, nearest_distance,
                #      nearest_node.lat, nearest_node.lon)
                # print(name,nearest_node_id_pro,nearest_distance_pro,nearest_node_pro.lat,nearest_node_pro.lon)
    # 导入节点数据

    @ timer
    def load_nodes(self, OSM):
        _nodes = OSM.getElementsByTagName("node")
        for _node in _nodes:
            node_id = int(_node.getAttribute("id"))
            node_lat = float(_node.getAttribute("lat"))
            node_lon = float(_node.getAttribute("lon"))
            bisect.insort(
                self.nodes, (node_id, node(node_id, node_lat, node_lon)))

    # 导入地图边框数据
    def load_bounds(self, OSM):
        _bounds = OSM.getElementsByTagName("bounds")
        for bound in _bounds:
            self.minlat = float(bound.getAttribute("minlat"))
            self.maxlat = float(bound.getAttribute("maxlat"))
            self.minlon = float(bound.getAttribute("minlon"))
            self.maxlon = float(bound.getAttribute("maxlon"))

    # 检查坐标是否在校园范围内
    def check_bounds(self, pos):
        if pos[0] < self.minlat or pos[0] > self.maxlat or pos[1] < self.minlon or pos[1] > self.maxlon:
            return False
        else:
            return True

    # 导入数据
    @ timer
    def load(self, datapath):
        DOMTree = xml.dom.minidom.parse(datapath)
        OSM = DOMTree.documentElement
        self.load_bounds(OSM)
        self.load_nodes(OSM)
        self.load_ways(OSM)
        self.nodes_dropna()
        self.load_buildings(OSM)

    # 根据节点算最短路径
    @ timer
    def Shortest_path_node(self, start_node, end_node, type):
        def node_clear():
            for node in self.nodes:
                node[1].pre = None
                node[1].distance = sys.maxsize

        node_clear()
        start_node[1].distance = 0

        if type == 1:
            nodelist = self.nodes_con.copy()
            del(nodelist[self.nodes_loc(nodelist, start_node[0])])

            for nd_con in start_node[1].connection_nodes_type1:
                nd_con[1].distance = nd_con[2]
                nd_con[1].pre = start_node[1]

            while True:
                if not len(nodelist):
                    break
                node_proc = min(nodelist, key=lambda x: x[1].distance)
                # print(node_proc[1].distance)
                for nd_next in node_proc[1].connection_nodes_type1:
                    pot_distance = nd_next[2]+node_proc[1].distance
                    if pot_distance < nd_next[1].distance:
                        nd_next[1].distance = nd_next[2]+node_proc[1].distance
                        nd_next[1].pre = node_proc[1]
                    # print(min(pot_distance,nd_next[1].distance))
                del(nodelist[self.nodes_loc(nodelist, node_proc[0])])
                if node_proc == end_node:
                    break

        if type == 2:
            nodelist = self.nodes_con_pro.copy()
            del(nodelist[self.nodes_loc(nodelist, start_node[0])])

            for nd_con in start_node[1].connection_nodes_type2:
                nd_con[1].distance = nd_con[2]
                nd_con[1].pre = start_node[1]

            while True:
                if not len(nodelist):
                    break
                node_proc = min(nodelist, key=lambda x: x[1].distance)
                # print(node_proc[1].distance)
                for nd_next in node_proc[1].connection_nodes_type2:
                    pot_distance = nd_next[2]+node_proc[1].distance
                    if pot_distance < nd_next[1].distance:
                        nd_next[1].distance = nd_next[2]+node_proc[1].distance
                        nd_next[1].pre = node_proc[1]
                    # print(min(pot_distance,nd_next[1].distance))
                del(nodelist[self.nodes_loc(nodelist, node_proc[0])])
                if node_proc == end_node:
                    break

        final_distance = end_node[1].distance
        route = [end_node[1].pre, end_node[1]]
        nd = end_node[1]
        while True:
            nd = nd.pre
            if nd == start_node[1]:
                break
            route.insert(0, nd.pre)
        return route, final_distance

        '''
        if type==1:
            nodes=self.nodes_con.copy()
            for nd in nodes:
                if nd==start_node:
                    nd=(nd[0],nd[1],True,0,None)
                elif nd in [x[0] for x in start_node[1].connection_nodes_type1]:
                    nd=(nd[0],nd[1],,start_node[1])



            # print(nd_notreached)
            # for nd_next in start_node.connection_nodes_type1:
        '''

    # 根据坐标算最短路径
    @ timer
    def Shortest_path_pos(self, start_pos, end_pos, type):
        if not self.check_bounds(start_pos) or not self.check_bounds(end_pos):
            print("Failed to Load Position")
            return [], -1
        if type == 1:
            start_id, start_node, start_distance = self.nearest_node(
                self.nodes_con, start_pos[0], start_pos[1])
            end_id, end_node, end_distance = self.nearest_node(
                self.nodes_con, end_pos[0], end_pos[1])

            route, distance = self.Shortest_path_node(
                (start_id, start_node), (end_id, end_node), 1)
            distance += (start_distance+end_distance)

            path = [start_pos]
            for nd in route:
                path.append((nd.lat, nd.lon))
            path.append(end_pos)

        if type == 2:
            start_id, start_node, start_distance = self.nearest_node(
                self.nodes_con_pro, start_pos[0], start_pos[1])
            end_id, end_node, end_distance = self.nearest_node(
                self.nodes_con_pro, end_pos[0], end_pos[1])

            route, distance = self.Shortest_path_node(
                (start_id, start_node), (end_id, end_node), 2)
            distance += (start_distance+end_distance)

            path = [start_pos]
            for nd in route:
                path.append((nd.lat, nd.lon))
            path.append(end_pos)

        return path, distance


if __name__ == '__main__':
    OP = OSMParser('../data/map.osm')
    print(OP.building_name_list)
    # for nd in OP.nodes:
    #    if nd[1].lat==31.3187102 and nd[1].lon==121.3873172:
    #        print(nd[1].id,nd[1].lat,nd[1].lon)
    #        for nd_c in nd[1].connection_nodes_type1:
    #            print(nd_c[0],nd_c[1].lat,nd_c[1].lon,nd_c[2],nd_c[3])
    # a = random.randint(0, len(OP.building_info_list)-1)
    a = OP.building_name_list.index("J楼")
    b = OP.building_name_list.index("A楼")
    # b = random.randint(0, len(OP.building_info_list)-1)
    type = 1

    print(OP.building_info_list[a][type][1].lat, OP.building_info_list[a]
          [type][1].lon, OP.building_info_list[a][0])
    print(OP.building_info_list[b][type][1].lat, OP.building_info_list[b]
          [type][1].lon, OP.building_info_list[b][0])
    route, distance = OP.Shortest_path_pos(
        [OP.building_info_list[a][type][1].lat, OP.building_info_list[a][type][1].lon], [OP.building_info_list[b][type][1].lat, OP.building_info_list[b][type][1].lon], type)


# print(nd[1].connection_nodes_type1)
# print(nd_near.lat,nd_near.lon)
'''
DOMTree = xml.dom.minidom.parse("../data/map.osm")
collection = DOMTree.documentElement

ways = collection.getElementsByTagName("way")
taglist=['name','highway','oneway','bicycle','foot','motor_vehicle','bridge']
for way in ways:
    print("*****WAY*****"+way.getAttribute("id"))
    # for node in way.getElementsByTagName("nd"):
    #    print(node.getAttribute("ref"))
    for tag in way.getElementsByTagName("tag"):
        if tag.getAttribute("k") not in taglist:
            taglist.append(tag.getAttribute("k"))
        if tag.getAttribute("k")=="building":
            print(tag.getAttribute("v"))
        if tag.getAttribute("k")=="name":
            print(tag.getAttribute("v"))

print(taglist)

nodes = collection.getElementsByTagName("node")
for node in nodes:
    # print("*****NODE*****")
    # print(node.getAttribute("id"))
    for tag in node.getElementsByTagName("tag"):
        if(tag.getAttribute("k")=='name'):
            print(tag.getAttribute("v"))


relations=collection.getElementsByTagName("relation")
for relation in relations:
    for tag in relation.getElementsByTagName("tag"):
        if tag.getAttribute("k")=="name":
            print(tag.getAttribute("v"))
'''
