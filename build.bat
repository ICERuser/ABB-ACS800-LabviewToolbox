@echo off
rem Build without Maven (JDK 17+ required)
cd /d %~dp0
if exist build rmdir /s /q build
mkdir build\classes
dir /s /b src\main\java\*.java > build\sources.txt
javac --release 17 -encoding UTF-8 -d build\classes @build\sources.txt || exit /b 1
echo Main-Class: windemu.Main> build\manifest.txt
jar cfm wind-emulator.jar build\manifest.txt -C build\classes .
echo Built wind-emulator.jar  -^>  java -jar wind-emulator.jar emulator.properties
