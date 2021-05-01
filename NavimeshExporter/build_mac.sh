#!/bin/bash

cd proj.mac && \
xcodebuild clean && \
xcodebuild -parallelizeTargets -jobs 4 && \
cp -r build/Release/NavimeshExporter.bundle ../../../Assets/NavimeshMarker/Script/Plugins/ && \
cd -
