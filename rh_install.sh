#!/usr/bin/env bash

set -e

if test -e lib/RadioHead; then
  echo Is RadioHead already installed?
  echo Please, remove the directory \'lib/RadioHead\'.
  exit 1
fi

cd lib
wget http://www.airspayce.com/mikem/arduino/RadioHead/RadioHead-1.121.zip
unzip RadioHead-1.121.zip
rm -f RadioHead-1.121.zip

echo Please, re-init platformio project.