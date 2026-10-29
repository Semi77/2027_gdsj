"""二维地图和坐标换算，供Windows绘图及树莓派目标校验共用。"""
import math


def pose_values(data):
    """@brief 校验外部x、y和yaw，返回米与弧度，拒绝非有限值及布尔值。"""
    result=[]
    for key in ('x','y','yaw'):
        value=data.get(key)
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
            raise ValueError('位置与朝向必须为有限数值')
        result.append(float(value))
    if abs(result[0])>1000 or abs(result[1])>1000 or abs(result[2])>100*math.pi:
        raise ValueError('位置或朝向超出支持范围')
    result[2]=(result[2]+math.pi)%(2*math.pi)-math.pi
    return tuple(result)


class MapGeometry:
    def __init__(self, data):
        """@brief 从栅格元数据建立原点变换，origin为x、y、yaw。"""
        self.width,self.height=data['width'],data['height']
        self.resolution=data['resolution']
        origin=data['origin']
        if (type(self.width) is not int or type(self.height) is not int or
            self.width<=0 or self.height<=0 or self.width*self.height>1000000 or
            isinstance(self.resolution,bool) or not isinstance(self.resolution,(int,float)) or
            not math.isfinite(self.resolution) or not 0<self.resolution<=10 or len(origin)!=3):
            raise ValueError('地图尺寸、分辨率或原点无效')
        self.ox,self.oy,self.yaw=pose_values(dict(zip(('x','y','yaw'),origin)))
        self.frame=data.get('frame','')

    def cell_to_world(self, x, y):
        """@brief 将连续栅格边界坐标x、y转换为地图米制坐标，不隐式加半格。"""
        c,s=math.cos(self.yaw),math.sin(self.yaw)
        return (self.ox+self.resolution*(x*c-y*s),self.oy+self.resolution*(x*s+y*c))

    def world_to_cell(self, x, y):
        """@brief 将地图米制坐标x、y逆变换为整数栅格，地图外返回None。"""
        dx,dy=x-self.ox,y-self.oy;c,s=math.cos(self.yaw),math.sin(self.yaw)
        ix=math.floor((dx*c+dy*s)/self.resolution)
        iy=math.floor((-dx*s+dy*c)/self.resolution)
        return (ix,iy) if 0<=ix<self.width and 0<=iy<self.height else None

    def free(self, cells, x, y):
        """@brief 检查x、y是否处于已知空闲栅格，未知区和地图外均不允许选目标。"""
        index=self.world_to_cell(x,y)
        if index is None or len(cells)!=self.width*self.height:return False
        value=cells[index[1]*self.width+index[0]]
        return isinstance(value,int) and 0<=value<25


def transform_points(points, pose):
    """@brief 用pose的x、y、yaw将二维points转换到目标坐标系。"""
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
    return [[x+px*c-py*s,y+px*s+py*c] for px,py in points]
