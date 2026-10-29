"""原子保存ROS参数，JSON文档也属于有效的YAML格式。"""
import json
import os
from pathlib import Path
import stat
import tempfile


def save_parameters(path, parameters):
    """@brief 将parameters保存为chassis_controller参数文件，失败时保留原文件。"""
    path = Path(path)
    content = json.dumps({'chassis_controller':{'ros__parameters':parameters}},
                         ensure_ascii=False, allow_nan=False, indent=2)+'\n'
    mode = stat.S_IMODE(path.stat().st_mode) if path.exists() else 0o600
    descriptor, temporary = tempfile.mkstemp(prefix='.chassis_params_', dir=path.parent)
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8', newline='\n') as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, mode)
        os.replace(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)
