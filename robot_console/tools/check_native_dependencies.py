"""静态检查打包清单及 Qt 导入符号，不启动 GUI 或连接硬件。"""
import ast
import os
from pathlib import Path
import sys

import pefile


def main():
    """@brief 从 Analysis 清单校验 Qt 所需 DLL 来源及导出符号。"""
    root=Path(__file__).resolve().parents[1]
    toc=ast.literal_eval((root/'build/RobotConsole/Analysis-00.toc').read_text(encoding='utf-8'))
    files={}
    for group in toc:
        if isinstance(group,list):
            for item in group:
                if isinstance(item,tuple) and len(item)==3 and item[2]=='BINARY':
                    files[item[0].replace('\\','/').lower()]=Path(item[1])
    exports={}
    errors=[]
    system=Path(os.environ['SystemRoot'])/'System32'
    prefixes=['','pyside6/','shiboken6/']
    for name,path in files.items():
        if Path(name).name.startswith('icu') and not path.is_relative_to(system):
            errors.append(f'不允许从其他软件捆绑 ICU：{path}')
        if Path(name).name not in ('qtcore.pyd','qtgui.pyd','qtwidgets.pyd','qt6core.dll',
                'qt6gui.dll','qt6widgets.dll','pyside6.abi3.dll','shiboken6.abi3.dll'):
            continue
        binary=pefile.PE(str(path), max_symbol_exports=65536)
        for dependency in getattr(binary,'DIRECTORY_ENTRY_IMPORT',[]):
            dll=dependency.dll.decode().lower()
            if dll.startswith(('api-','ext-')):
                continue
            keys=[str(Path(name).parent/dll).replace('\\','/'),*[prefix+dll for prefix in prefixes]]
            candidates=[files[key] for key in keys if key in files]+[system/dll,Path(sys.base_prefix)/dll]
            target=next((candidate for candidate in candidates if candidate.is_file()),None)
            if target is None:
                errors.append(f'{name} 缺少依赖 {dll}')
                continue
            if target not in exports:
                # QtGui/QtWidgets 导出数超过 pefile 默认的 8192 项，必须完整读取。
                library=pefile.PE(str(target), max_symbol_exports=65536)
                exports[target]={symbol.name for symbol in library.DIRECTORY_ENTRY_EXPORT.symbols
                                 if symbol.name} if hasattr(library,'DIRECTORY_ENTRY_EXPORT') else set()
            missing=[symbol.name.decode(errors='replace') for symbol in dependency.imports
                     if symbol.name and symbol.name not in exports[target]]
            if missing:
                errors.append(f'{name} -> {dll} 缺少符号：{missing[:5]}')
    if errors:
        raise SystemExit('\n'.join(errors))
    print('Qt 原生依赖静态检查通过；未启动应用或连接硬件。')


if __name__=='__main__':
    main()
