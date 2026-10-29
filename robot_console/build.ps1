param([switch]$SkipInstall)
$ErrorActionPreference = 'Stop'
$originalBuildPath = $env:Path
Push-Location $PSScriptRoot
try {
    $consolePython = Join-Path $PSScriptRoot '.venv/Scripts/python.exe'
    if (-not (Test-Path -LiteralPath $consolePython)) {
        python -m venv .venv
        if ($LASTEXITCODE -ne 0) { throw '创建 Python 环境失败' }
    }
    # 隔离其他软件的 DLL，避免将 Poppler/Conda 的同名 ICU 错收进 exe。
    $env:Path = (@(
        (Join-Path $PSScriptRoot '.venv/Scripts'),
        (Join-Path $env:SystemRoot 'System32'),
        $env:SystemRoot
    ) -join ';')
    if (-not $SkipInstall) {
        & $consolePython -m pip install -r requirements.txt
        if ($LASTEXITCODE -ne 0) { throw '安装构建依赖失败' }
    }
    # 将树莓派网关和协议一起打包，用户双击 exe 时不依赖本机 Python。
    & $consolePython -m PyInstaller --noconfirm --clean --onefile --windowed `
        --name RobotConsole --add-data 'pi;pi' --add-data 'shared;shared' `
        --hidden-import PySide6.QtNetwork main.py
    if ($LASTEXITCODE -ne 0) { throw 'exe 打包失败' }
    & $consolePython tools/check_native_dependencies.py
    if ($LASTEXITCODE -ne 0) { throw 'Qt 原生依赖静态检查失败，请勿使用本次产物' }
    Copy-Item -LiteralPath README.md -Destination dist/使用说明.md -Force
    Copy-Item -LiteralPath PI_CONTROL.md -Destination dist/PI_CONTROL.md -Force
    Copy-Item -LiteralPath NAVIGATION.md -Destination dist/导航使用说明.md -Force
    Copy-Item -LiteralPath PROTOCOL.md -Destination dist/PROTOCOL.md -Force
    Copy-Item -LiteralPath PROTOCOL.md -Destination dist/接口协议.md -Force
    Copy-Item -LiteralPath UART3_接线与烧录.md -Destination dist/串口接线说明.md -Force
    # 随交付目录保留已安装第三方依赖的许可证文本。
    $licenseRoot = Join-Path $PSScriptRoot 'dist/第三方许可'
    New-Item -ItemType Directory -Path $licenseRoot -Force | Out-Null
    Get-ChildItem -LiteralPath '.venv/Lib/site-packages' -Directory -Filter '*.dist-info' | ForEach-Object {
        $sourceLicense = Join-Path $_.FullName 'licenses'
        if (Test-Path -LiteralPath $sourceLicense) {
            $packageLicense = Join-Path $licenseRoot $_.Name
            New-Item -ItemType Directory -Path $packageLicense -Force | Out-Null
            Copy-Item -LiteralPath $sourceLicense -Destination $packageLicense -Recurse -Force
        }
    }
    Write-Output '已生成 dist/RobotConsole.exe；构建不等于运行或硬件验证。'
}
finally {
    $env:Path = $originalBuildPath
    Pop-Location
}
