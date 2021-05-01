#!/bin/bash

cd proj.mac && \
xcodebuild clean && \
xcodebuild -parallelizeTargets -jobs 4 && \
cp -r build/Release/RuntimeWrapper.bundle ../ && \
cd -
