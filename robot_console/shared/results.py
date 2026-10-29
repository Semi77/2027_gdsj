"""K230 检测接口，模型类别表由训练结果决定。"""
import math


def validate_detection(data):
    """@brief 校验检测结果对象，中心坐标以原始图像左上角为像素原点。"""
    if not isinstance(data, dict) or data.get('version') != 1:
        raise ValueError('检测消息需要 version=1')
    for key in ('frame_id','width','height'):
        if type(data.get(key)) is not int or data[key] < (0 if key=='frame_id' else 1):
            raise ValueError(f'{key} 必须为有效整数')
    fps=data.get('fps')
    if not isinstance(fps,(int,float)) or not math.isfinite(fps) or not 0 <= fps <= 10000:
        raise ValueError('fps 无效')
    items=data.get('detections')
    if not isinstance(items,list) or len(items)>100:
        raise ValueError('detections 必须为至多 100 项的数组')
    for item in items:
        if not isinstance(item,dict) or type(item.get('class_id')) is not int or item['class_id']<0:
            raise ValueError('class_id 无效')
        if not isinstance(item.get('label'),str) or len(item['label'])>128:
            raise ValueError('label 无效')
        for key,bound in [('cx',data['width']),('cy',data['height']),('confidence',1)]:
            value=item.get(key)
            if not isinstance(value,(int,float)) or not math.isfinite(value) or not 0<=value<=bound:
                raise ValueError(f'{key} 超出范围')
    return data
