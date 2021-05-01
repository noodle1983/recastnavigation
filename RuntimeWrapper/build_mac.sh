#!/bin/bash

cd proj.mac && \
xcodebuild clean && \
xcodebuild -parallelizeTargets -jobs 4 && \
cp -r build/Release/DetourRuntime.bundle ../../../Assets/NavimeshMarker/RuntimePlugins/ && \
cd -
