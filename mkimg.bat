@echo off
if not exist img mkdir img
copy dridle.sys img
copy tstidle.exe img
mkimage -o dridle.img img
