#! /bin/bash

# The script is used to check that the rqt_gui plugins are discoverable using the rqt_gui command line options.

source /entrypoint.sh

echo "Testing that the RQt Plugins are discoverable"

EXIT_CODE=0

function check_plugin() {
  local PLUGIN_NAME=$1

  QT_QPA_PLATFORM=offscreen \
    ros2 run rqt_gui rqt_gui \
      --force-discover \
      --list-plugins | grep -qx "$PLUGIN_NAME"

  if [ $? -eq 0 ]; then
    echo "✅ Plugin '$PLUGIN_NAME' is discoverable by rqt_gui"
  else
    echo "❌ Plugin '$PLUGIN_NAME' not found in rqt_gui --list-plugins"
    EXIT_CODE=1
  fi

}

check_plugin "ros_virtual_joystick/plugins/rqt_gui/SinglePadJoystickPlugin"
check_plugin "ros_virtual_joystick/plugins/rqt_gui/DualPadJoystickPlugin"

exit $EXIT_CODE
