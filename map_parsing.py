import math
import xml.dom.minidom
from cmath import cos
from math import sin

def calculate_distance(x1, y1, x2, y2):
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    return distance
def get_attrvalue(node, attrname):
    return node.getAttribute(attrname) if node else ''

def get_nodevalue(node, index=0):
    return node.childNodes[index].nodeValue if node else ''

def get_xmlnode(node, name):
    return node.getElementsByTagName(name) if node else []

def get_xml_data(filename,road_node_list):
    doc = xml.dom.minidom.parse(filename)
    root = doc.documentElement
    road_node = get_xmlnode(root, 'road')
    for node in road_node:
        road_node_list.append(node)

def get_road_node(road_node_list,id):
    for node in road_node_list:
        if int(get_attrvalue(node,'id'))==id:
            return node

def get_road_referencelines(node):
    planView=get_xmlnode(node,'planView');
    referenceline=get_xmlnode(planView[0],'geometry')
    return referenceline # road list


class Road:
    def __init__(self, node):
        self.id=int(node.getAttribute('id'))
        self.planView = get_xmlnode(node, 'planView');
        self.length=float(node.getAttribute('length'))
        self.lanes = get_xmlnode(node, 'lanes');
        self.referencelines = get_xmlnode(self.planView[0], 'geometry')
        self.idx=0
        self.ds=0.0
        self.s=0.0
        self.laneOffsets=get_xmlnode(self.lanes[0], 'laneOffset')
        self.laneSections = get_xmlnode(self.lanes[0], 'laneSection')
        self.direction=1 #  1:+ -1: -
        self.backward=False
        self.eps=0.03

    def set_idx(self,idx):
        self.idx=idx
    def set_s(self,s):
        self.s=s
    def set_backward(self,backward):
        self.backward=backward

    def get_straight_project_point(self,node,vx, vy):  # 转换(carla to opendrive)后的vx,vy
        # 计算车辆坐标在参考线上的投影点坐标
        x = float(node.getAttribute('x'))
        y = float(node.getAttribute('y'))
        hdg = float(node.getAttribute('hdg'))
        dx = vx - x
        dy = vy - y
        line_vector_x = cos(hdg).real  # 哪个角，反？
        line_vector_y = sin(hdg).real
        vector_a = (line_vector_x, line_vector_y)
        vector_b = (dx, dy)
        cross_product = vector_a[0] * vector_b[1] - vector_a[1] * vector_b[0]
        if cross_product >= 0:
            self.direction = 1
        else:
            self.direction = -1
        dot_product = dx * line_vector_x + dy * line_vector_y
        projection_x = x + dot_product * line_vector_x
        projection_y = y + dot_product * line_vector_y
        return (projection_x, projection_y)

    def get_arc_project_point(self,node, vx, vy):
        # 计算车辆坐标在参考线上的投影点坐标
        x = float(node.getAttribute('x'))
        y = float(node.getAttribute('y'))
        hdg = float(node.getAttribute('hdg'))
        curvature = float(node.getElementsByTagName('arc')[0].getAttribute('curvature'))
        radius = 1 / abs(curvature)
        if curvature <= 0: #顺时针
            if hdg <= 0:
                Center_x = x + radius * sin(hdg).real
                Center_y = y - radius * cos(hdg).real
            else:
                hdg = math.pi - hdg
                Center_x = x + radius * sin(hdg).real
                Center_y = y + radius * cos(hdg).real

            Vector_x = vx - Center_x
            Vector_y = vy - Center_y

            distance_to_center = math.sqrt(Vector_x ** 2 + Vector_y ** 2)
            distance_left = distance_to_center-radius
            direction_vector = [vx - Center_x, vy - Center_y]
            perpendicular_vector = [-direction_vector[1], direction_vector[0]]
            slope = perpendicular_vector[1] / perpendicular_vector[0]
            angle_radians = math.atan(slope)

            if angle_radians>=0:
                angle_radians = math.pi - angle_radians
                if distance_to_center > radius:
                    projection_x = vx - distance_left * sin(angle_radians).real
                    projection_y = vy - distance_left * cos(angle_radians).real
                else:
                    projection_x = vx + distance_left * sin(angle_radians).real
                    projection_y = vy + distance_left * cos(angle_radians).real

            else:
                if distance_to_center > radius:
                    projection_x = vx + distance_left * sin(angle_radians).real
                    projection_y = vy - distance_left * cos(angle_radians).real
                else:
                    projection_x = vx - distance_left * sin(angle_radians).real
                    projection_y = vy + distance_left * cos(angle_radians).real

                if distance_to_center <= radius:
                    self.direction = -1
                else:
                    self.direction = 1

        else:
            if hdg < 0:
                Center_x = x - radius * sin(hdg).real
                Center_y = y + radius * cos(hdg).real
            else:
                hdg = math.pi - hdg
                Center_x = x - radius * sin(hdg).real
                Center_y = y - radius * cos(hdg).real

            Vector_x = vx - Center_x
            Vector_y = vy - Center_y

            distance_to_center = math.sqrt(Vector_x ** 2 + Vector_y ** 2)
            distance_left = radius-distance_to_center
            direction_vector = [vx - Center_x, vy - Center_y]
            perpendicular_vector = [-direction_vector[1], direction_vector[0]]
            slope = perpendicular_vector[1] / perpendicular_vector[0]
            angle_radians = math.atan(slope)
            if angle_radians >= 0:
                angle_radians = math.pi - angle_radians
                if distance_to_center > radius:
                    projection_x = vx + distance_left * sin(angle_radians).real
                    projection_y = vy + distance_left * cos(angle_radians).real
                else:
                    projection_x = vx - distance_left * sin(angle_radians).real
                    projection_y = vy - distance_left * cos(angle_radians).real

            else:
                if distance_to_center > radius:
                    projection_x = vx - distance_left * sin(angle_radians).real
                    projection_y = vy + distance_left * cos(angle_radians).real
                else:
                    projection_x = vx + distance_left * sin(angle_radians).real
                    projection_y = vy - distance_left * cos(angle_radians).real
            if distance_to_center <= radius:
                self.direction = 1
            else:
                self.direction = -1

        return (projection_x, projection_y)

    def get_vehicle_project_point(self,vx, vy):
        cnt = len(self.referencelines)
        if self.idx >= cnt or self.idx<0:
            return (1e9, 1e9)
        node = self.referencelines[self.idx]
        if len(node.getElementsByTagName('arc')) == 1:  # Arc
            (projection_x, projection_y) = self.get_arc_project_point(node, vx, vy)
        else:
            (projection_x, projection_y) = self.get_straight_project_point(node, vx, vy)
        return (projection_x, projection_y)

    def get_offset_to_start(self,px,py):  #到当前参考线起点的s
        node = self.referencelines[self.idx]
        x = float(node.getAttribute('x'))
        y = float(node.getAttribute('y'))
        ns = float(node.getAttribute('s'))
        #length=float(node.getAttribute('length'))
        if len(node.getElementsByTagName('arc')) == 1:  # Arc
            hdg = float(node.getAttribute('hdg'))
            curvature = float(node.getElementsByTagName('arc')[0].getAttribute('curvature'))
            radius = 1 / abs(curvature)  # 使用半径的绝对值
            if curvature <= 0:
                if hdg <= 0:
                    Center_x = x + radius * sin(hdg).real
                    Center_y = y - radius * cos(hdg).real
                else:
                    hdg = math.pi - hdg
                    Center_x = x + radius * sin(hdg).real
                    Center_y = y + radius * cos(hdg).real
            else:
                if hdg <= 0:
                    Center_x = x - radius * sin(hdg).real
                    Center_y = y + radius * cos(hdg).real
                else:
                    hdg = math.pi - hdg
                    Center_x = x - radius * sin(hdg).real
                    Center_y = y - radius * cos(hdg).real

            vector1_x = x - Center_x
            vector1_y = y - Center_y

            # 计算第二个点与圆心的向量
            vector2_x = px - Center_x
            vector2_y = py - Center_y

            # 计算点积
            dot_product = vector1_x * vector2_x + vector1_y * vector2_y

            # 计算向量长度
            magnitude1 = math.sqrt(vector1_x ** 2 + vector1_y ** 2)
            magnitude2 = math.sqrt(vector2_x ** 2 + vector2_y ** 2)

            # 计算夹角余弦值
            cosine_theta = dot_product / (magnitude1 * magnitude2)

            # 计算夹角（弧度）
            angle_radians = math.acos(cosine_theta)

            # 计算弧长
            arc_length = angle_radians * radius
            self.ds=arc_length
        else:
            distance=calculate_distance(x, y, px, py)
            self.ds=distance
        #print(self.ds)
        self.s = self.ds + ns


    def to_next_referenceline(self):
        node =self.referencelines[self.idx]
        length = float(node.getAttribute('length'))
        #print(self.ds)
        if not self.backward:
            if self.ds >= length:
                self.idx += 1;
                self.ds = 0;
        else:
            if self.ds<=self.eps:
                self.idx -= 1;
                self.ds = 0;


    def get_laneOffset(self):
        for laneOffset in reversed(self.laneOffsets):
            os = float(laneOffset.getAttribute('s'))
            if self.s >= os:
                return laneOffset
    def get_lane_section(self):
        for lanesection in reversed(self.laneSections):
            ls = float(lanesection.getAttribute('s'))
            if self.s >= ls:
                return lanesection

    def get_offset(self,node):
        a = float(node.getAttribute('a'))
        b = float(node.getAttribute('b'))
        c = float(node.getAttribute('c'))
        d = float(node.getAttribute('d'))
        if node.hasAttribute('s'):
            ns = float(node.getAttribute('s'))
        elif node.hasAttribute('sOffset'):
            ns = float(node.getAttribute('sOffset'))
        ds=self.s-ns
        return a + ds * (b + ds * (c + ds * d))

    def find_width(self,widths):
        for width in reversed(widths):
            sOffset = float(width.getAttribute('sOffset'))
            if self.s >= sOffset:
                return self.get_offset(width)

    def get_widths(self,lanes):
        widths=[]
        for lane in lanes:
            ws=lane.getElementsByTagName('width')
            widths.append(self.find_width(ws))
        return widths

    def get_lane_id(self,d):
        lanesection=self.get_lane_section()
        laneOffset=self.get_laneOffset()
        #print(f"id:{self.id},s:{self.s},laneOffset:{laneOffset.getAttribute('s')}")
        offset=self.get_offset(laneOffset)
        on_left=True
        if self.direction==1:
            d-=offset
            if d<0:
                on_left=False
        else:
            d+=offset
            if d>0:
                on_left=False

        if len(lanesection.getElementsByTagName('left')) == 1 and len(lanesection.getElementsByTagName('right'))==0:
            on_left=True
        elif len(lanesection.getElementsByTagName('left')) == 0 and len(lanesection.getElementsByTagName('right'))==1:
            on_left = False

        if on_left:
            left_lanes = lanesection.getElementsByTagName("left")[0]
            lanes=left_lanes.getElementsByTagName('lane')
        else:

            right_lanes = lanesection.getElementsByTagName('right')[0]
            lanes = right_lanes.getElementsByTagName('lane')

        widths=self.get_widths(lanes)
        if on_left:
            widths=list(reversed(widths))

        sum=0.0
        res=0
        n=len(widths)
        while res<n and sum+widths[res]<=d:
            sum+=widths[res]
            res+=1
        res+=1
        if not on_left:
            res=-res
        return res

if __name__ == "__main__":
    road_list=[]
    road_id_list=[1]
    cur=0
    vx = 89.33
    vy = -119.27
    get_xml_data('Town10HD.xodr', road_list)
    road_node = get_road_node(road_list, road_id_list[cur])
    r = Road(road_node)
    r.set_s(77.99)
    r.set_index(2)
    (px,py)=r.get_vehicle_project_point(vx,vy)

    r.get_offset_to_start(px, py)
    d = calculate_distance(px, py, vx, vy)
    lane_id = r.get_lane_id(d)
    #print(f"vx:{vx},vy:{vy}")
    #print(f"road_id:{road_id_list[cur]},lane_id:{lane_id},s:{r.s}")
    r.to_next_referenceline()
