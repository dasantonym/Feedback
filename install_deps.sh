#!/usr/bin/env bash

INSTALL_DIR=${HOME}/openframeworks/addons

if [ -d ${INSTALL_DIR} ]
then
    ADDON_DEPS=('https://github.com/neilmendoza/ofxGpuParticles.git', 'https://github.com/kylemcdonald/ofxCv.git', 'https://github.com/timscaffidi/ofxOpticalFlowFarneback.git', 'https://github.com/dasantonym/ofxFBOTexture.git')
    pushd ${INSTALL_DIR}
    for i in $(seq 0 $((${#ADDON_DEPS[@]} - 1)) )
    do
	git clone ${ADDON_DEPS[i]}
    done
    popd
    exit 0
else
    echo "Install dir ${INSTALL_DIR} not found" >&2
    exit 1
fi
exit 1