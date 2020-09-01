#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

xacro "$DIR"/cupboard_drawers/cupboard_drawers.urdf.xacro > "$DIR"/cupboard_drawers/cupboard_drawers.urdf
xacro "$DIR"/container/container_no_lid.urdf.xacro > "$DIR"/container/container_no_lid.urdf
xacro "$DIR"/container/lid.urdf.xacro > "$DIR"/container/lid.urdf
xacro "$DIR"/container/container_sliding_lid.urdf.xacro > "$DIR"/container/container_sliding_lid.urdf
xacro "$DIR"/box_panda_hand.urdf.xacro > "$DIR"/box_panda_hand_pb.urdf
sed -i 's/package:\/\/franka_description/\/opt\/ros\/melodic\/share\/franka_description/g' "$DIR"/box_panda_hand_pb.urdf
