#!/bin/sh

scons production=true

retVal=$?
if [ $retVal -ne 0 ]; then
    echo "Error during build, skipping output build copy"
	exit $retval
fi

#rm '/home/almic/.local/share/Steam/steamapps/common/Godot Engine/godot.x11.opt.tools.64'
#ln '/home/almic/repos/godot/bin/godot.linuxbsd.editor.double.x86_64' '/home/almic/.local/share/Steam/steamapps/common/Godot Engine/godot.x11.opt.tools.64'

