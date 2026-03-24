#!/bin/sh -xe

sudo apt-get update -qq

sudo apt-get install -q -y \
  zlib1g-dev libpng-dev libcairo2-dev libfreetype6-dev libjson-c-dev \
  libfontconfig1-dev libgtkmm-3.0-dev libpangomm-1.4-dev libgl-dev \
  libgl-dev libglu-dev libspnav-dev libtbb-dev \
  libocct-data-exchange-dev libocct-draw-dev libocct-foundation-dev \
  libocct-modeling-algorithms-dev libocct-modeling-data-dev \
  libocct-ocaf-dev libocct-visualization-dev

git submodule update --init extlib/libdxfrw extlib/mimalloc extlib/eigen
