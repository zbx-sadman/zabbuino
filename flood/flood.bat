@echo off
:restart
zabbix_get.exe -s 192.168.0.91  -k sys.all
timeout 1 > NUL
goto restart