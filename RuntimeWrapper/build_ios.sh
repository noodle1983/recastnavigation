#!/bin/bash

cd proj.ios && \
xcodebuild clean && \
xcodebuild -parallelizeTargets -jobs 4 && \
cp -r build/Release-iphoneos/libRuntimeWrapper.a ../ && \
cd -
