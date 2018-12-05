#!/bin/bash

mkdir -p ROOT/tmp/ACEDome_X2/
cp "../ACEDome.ui" ROOT/tmp/ACEDome_X2/
cp "../ACE.png" ROOT/tmp/ACEDome_X2/
cp "../domelist ACEDome.txt" ROOT/tmp/ACEDome_X2/
cp "../build/Release/libACEDome.dylib" ROOT/tmp/ACEDome_X2/

if [ ! -z "$installer_signature" ]; then
# signed package using env variable installer_signature
pkgbuild --root ROOT --identifier org.rti-zone.ACEDome_X2 --sign "$installer_signature" --scripts Scripts --version 1.0 ACEDome_X2.pkg
pkgutil --check-signature ./ACEDome_X2.pkg
else
pkgbuild --root ROOT --identifier org.rti-zone.ACEDome_X2 --scripts Scripts --version 1.0 ACEDome_X2.pkg
fi

rm -rf ROOT
