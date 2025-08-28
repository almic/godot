#!/bin/sh

scons production=true precision=double

rm '/home/almic/.local/share/Steam/steamapps/common/Godot Engine/godot.x11.opt.tools.64'
ln '/home/almic/repos/godot/bin/godot.linuxbsd.editor.double.x86_64' '/home/almic/.local/share/Steam/steamapps/common/Godot Engine/godot.x11.opt.tools.64'

