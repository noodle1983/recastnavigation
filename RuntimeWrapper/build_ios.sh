#!/bin/bash

cd proj.ios && \
xcodebuild clean && \
xcodebuild -parallelizeTargets -jobs 4 && \
cp -r build/Release-iphoneos/libDetourRuntime.a ../../../Assets/NavimeshMarker/RuntimePlugins/iOS && \
cd -
