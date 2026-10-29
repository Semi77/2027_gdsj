"""仅供本次联调用的静态雷达地图匹配，不发送任何运动命令。"""
import math
import numpy as np


def match_scan(grid, scan):
    """@brief 将scan的有效距离与grid占据边界匹配，返回初始位姿及残差指标。"""
    width,height=grid.info.width,grid.info.height
    resolution=grid.info.resolution
    ox,oy=grid.info.origin.position.x,grid.info.origin.position.y
    cells=np.array(grid.data,dtype=np.int16).reshape(height,width)
    walls=np.argwhere(cells>=65)
    if len(walls)<10:raise ValueError('地图占据点不足')
    yy,xx=np.indices(cells.shape)
    field=np.sqrt(np.min((xx[...,None]-walls[:,1])**2+(yy[...,None]-walls[:,0])**2,axis=2))*resolution
    free=np.argwhere((cells==0)&(field>.18))
    positions=np.stack((ox+(free[:,1]+.5)*resolution,oy+(free[:,0]+.5)*resolution),axis=1)
    indices=np.arange(len(scan.ranges))
    ranges=np.asarray(scan.ranges)
    valid=np.isfinite(ranges)&(ranges>=max(.08,scan.range_min))&(ranges<=min(3.5,scan.range_max))
    angles=scan.angle_min+indices[valid]*scan.angle_increment
    ranges=ranges[valid]
    if len(ranges)<40:raise ValueError('当前有效激光点不足')
    sx,sy=ranges*np.cos(angles),ranges*np.sin(angles)
    def evaluate(points,angle):
        """@brief 计算候选中心points和朝向angle下，激光到地图墙面的截断平均距离。"""
        c,s=math.cos(angle),math.sin(angle)
        ix=np.floor((points[:,0,None]+sx*c-sy*s-ox)/resolution).astype(int)
        iy=np.floor((points[:,1,None]+sx*s+sy*c-oy)/resolution).astype(int)
        inside=(ix>=0)&(ix<width)&(iy>=0)&(iy<height)
        values=np.full(ix.shape,.30)
        values[inside]=np.minimum(field[iy[inside],ix[inside]],.30)
        scores=np.mean(values,axis=1)
        best=int(np.argmin(scores))
        return float(scores[best]),points[best],float(np.mean(values[best]<.12))
    best=(float('inf'),None,None,None)
    for angle in np.arange(-math.pi,math.pi,math.radians(5)):
        score,point,ratio=evaluate(positions,float(angle))
        if score<best[0]:best=(score,point,float(angle),ratio)
    center=best[1];theta=best[2]
    offsets=np.array([(x,y) for x in np.arange(-.04,.041,.01) for y in np.arange(-.04,.041,.01)])
    points=center+offsets
    ix=((points[:,0]-ox)/resolution).astype(int);iy=((points[:,1]-oy)/resolution).astype(int)
    safe=(ix>=0)&(ix<width)&(iy>=0)&(iy<height)
    points=points[safe];ix=ix[safe];iy=iy[safe]
    points=points[(cells[iy,ix]==0)&(field[iy,ix]>.18)]
    for angle in np.arange(theta-math.radians(5),theta+math.radians(5.1),math.radians(1)):
        score,point,ratio=evaluate(points,float(angle))
        if score<best[0]:best=(score,point,float(angle),ratio)
    if best[0]>.12 or best[3]<.60:raise ValueError('当前雷达与保存地图不匹配：'+str(best[0]))
    return {'x':float(best[1][0]),'y':float(best[1][1]),'yaw':best[2],
            'mean_wall_error_m':best[0],'matched_fraction':best[3]}
