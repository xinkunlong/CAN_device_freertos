@echo off

rem 激活虚拟环境
call .venv\Scripts\activate

rem 执行Python文件
cmd /k python can_device_test.py --channel COM22


