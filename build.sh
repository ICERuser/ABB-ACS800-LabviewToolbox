#!/bin/sh
# Build without Maven (JDK 17+ required)
set -e
cd "$(dirname "$0")"
rm -rf build && mkdir -p build/classes
javac --release 17 -encoding UTF-8 -d build/classes $(find src/main/java -name '*.java')
printf 'Main-Class: windemu.Main\n' > build/manifest.txt
jar cfm wind-emulator.jar build/manifest.txt -C build/classes .
echo "Built wind-emulator.jar  ->  java -jar wind-emulator.jar emulator.properties"
