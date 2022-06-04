'''
Author: Harryhht
Date: 2021-09-15 19:38:40
LastEditTime: 2021-09-19 00:50:06
LastEditors: Please set LastEditors
Description: In User Settings Edit
FilePath: \src\MapShower.py
'''
import folium
from folium.map import Popup
from folium.plugins import AntPath, MousePosition, MiniMap, Fullscreen
from OSMdecoder import OSMParser, timer
import random

# folium地图展示更新


class MapShower(object):
    # 地图初始化
    @timer
    def map_init(self):
        self.MAP = folium.Map(location=[31.3182, 121.3895],
                              zoom_start=16,
                              control_scale=True,
                              tiles="OpenStreetMap",
                              #tiles="Mapbox Bright",
                              # tiles="../data/2017-07-03_china_shanghai.mbtiles",
                              min_zoom=16,
                              max_zoom=19,
                              max_bounds=True,
                              min_lat=self.OP.minlat,
                              max_lat=self.OP.maxlat,
                              min_lon=self.OP.minlon,
                              max_lon=self.OP.maxlon,
                              attr='default')  # 地图初始化
        # folium.raster_layers.TileLayer("OpenStreetMap").add_to(self.MAP)
        # folium.raster_layers.TileLayer("stamentoner").add_to(self.MAP)
        # folium.LayerControl().add_to(self.MAP)

        formatter = "function(num) {return L.Util.formatNum(num, 5) + ' º ';};"
        MousePosition(
            position="topright",
            separator=" | ",
            empty_string="NaN",
            lng_first=True,
            num_digits=20,
            prefix="Coordinates:",
            lat_formatter=formatter,
            lng_formatter=formatter,
        ).add_to(self.MAP)  # 鼠标位置显示经纬度

        MiniMap(zoom_level_offset=-4).add_to(self.MAP)  # 小地图
        '''
        Fullscreen(
            position="topright",
            title="Expand me",
            title_cancel="Exit me",
            force_separate_button=True,
        ).add_to(self.MAP)#全屏
        '''

        # self.MAP.add_child(folium.LatLngPopup())
        self.building_info = self.OP.building_info_list
        self.building_name = self.OP.building_name_list
        self.MAP.save('school_map.html')  # 地图保存

    # 更新建筑间最短路径
    @timer
    def update_path_building(self, building_name1, building_name2, trans_type):
        # 根据建筑信息获取最短路径
        print(building_name1, building_name2)
        self.map_init()
        bd1_ = self.building_info[self.building_name.index(building_name1)]
        bd2_ = self.building_info[self.building_name.index(building_name2)]
        if trans_type == "步行":
            bd1_node = bd1_[1]
            bd2_node = bd2_[1]
            if bd1_node == bd2_node:
                route = [bd1_node[1], bd2_node[1]]
                distance = 0
            else:
                route, distance = self.OP.Shortest_path_node(
                    bd1_node, bd2_node, 1)
            speed = 1
        if trans_type == "骑行":
            bd1_node = bd1_[2]
            bd2_node = bd2_[2]
            if bd1_node == bd2_node:
                route = [bd1_node[1], bd2_node[1]]
                distance = 0
            else:
                route, distance = self.OP.Shortest_path_node(
                    bd1_node, bd2_node, 2)
            speed = 3
        timecost = distance/speed
        points = []
        for nd in route:
            points.append((nd.lat, nd.lon))
        # folium.PolyLine(points).add_to(self.MAP)
        # 使用Antpath绘制运动最短路径
        if trans_type == "步行":
            AntPath(points,
                    tooltip='Path from '+building_name1+' to '+building_name2+' by '+trans_type, delay=1000,
                    weight=8,
                    dash_array=[3, 500],
                    opacity=0.6,
                    hardware_acceleration=True,
                    ).add_to(self.MAP)
        if trans_type == "骑行":
            AntPath(points,
                    tooltip='Path from '+building_name1+' to '+building_name2+' by '+trans_type, delay=500,
                    weight=8,
                    dash_array=[3, 500],
                    opacity=0.6,
                    hardware_acceleration=True,
                    color='red'
                    ).add_to(self.MAP)
        # 标注起点与终点
        folium.Marker(location=points[0],
                      popup=Popup('Start: '+building_name1, max_width=100),
                      icon=folium.Icon(color='green'),
                      parse_html=True).add_to(self.MAP)
        folium.Marker(location=points[-1],
                      popup=Popup('End: '+building_name2, max_width=100),
                      icon=folium.Icon(color='red'),
                      parse_html=True).add_to(self.MAP)
        self.MAP.save('school_map.html')  # 保存地图
        return distance, timecost

    # 这个函数有bug暂时别用
    @timer
    def update_path_position(self, start_pos, end_pos, trans_type):
        if trans_type == '步行':
            type = 1
            speed = 1
        if trans_type == '骑行':
            type = 2
            speed = 3
        points, distance = self.OP.Shortest_path_pos(start_pos, end_pos, type)
        timecost = distance/speed
        folium.PolyLine(points, smooth_factor=0.5).add_to(self.MAP)
        folium.Marker(location=points[0],
                      popup=Popup('Start'),
                      icon=folium.Icon(color='green'),
                      parse_html=True).add_to(self.MAP)
        folium.Marker(location=points[-1],
                      popup=Popup('End'),
                      icon=folium.Icon(color='red'),
                      parse_html=True).add_to(self.MAP)
        self.MAP.save('school_map.html')
        return distance, timecost

    # 初始化
    def __init__(self, osmpath):
        self.OP = OSMParser(osmpath)
        self.map_init()


if __name__ == '__main__':
    MS = MapShower('../data/map_1.osm')
    print(
        MS.OP.building_info_list[MS.OP.building_name_list.index("生科楼")][1][1].lat)
    print(
        MS.OP.building_info_list[MS.OP.building_name_list.index("生科楼")][1][1].lon)
    MS.update_path_building('环化楼', '水泵房', '骑行')
    #MS.update_path_building('吾馨楼', '伟长楼', 'bicycling')
    # start_pos = (random.uniform(MS.OP.minlat, MS.OP.maxlat),
    #             random.uniform(MS.OP.minlon, MS.OP.maxlon))
    # end_pos = (random.uniform(MS.OP.minlat, MS.OP.maxlat),
    #           random.uniform(MS.OP.minlon, MS.OP.maxlon))
    #MS.update_path_position(start_pos, end_pos, 'bicycling')
