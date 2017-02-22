#!/bin/bash

rosservice call /cob_phidgets_toolchanger/ifk_toolchanger/set_digital "{uri: 'tool_changer_pin2', state: 1}"
rosservice call /cob_phidgets_toolchanger/ifk_toolchanger/set_digital "{uri: 'tool_changer_pin2', state: 0}"
