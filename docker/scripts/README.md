# Multiple instances of Apollo 6.0


* git clone the repository.

* you will need to edit 3 files to each Apollo instance and change the name of the container for each one:

1. docker/scripts/dev_start_multiuser.sh
1. docker/scripts/dev_into_multiuser.sh
1. bootstrap_multiuser.sh


* change the name of the containers in  ./docker/scripts/dev_start_multiuser.sh  and the dreamview port:
```txt
APOLLO_DEV="apollo_dev_${USER}"
 -->   APOLLO_DEV="apollo_dev_${USER}2"

HOST_DREAMVIEW_PORT=$((DREAMVIEW_PORT + ($UID - 1000)))
-->  HOST_DREAMVIEW_PORT=$((DREAMVIEW_PORT + ($UID - 999)))

HOST_BRIDGE_PORT=$((BRIDGE_PORT + ($UID - 1000)))
-->  HOST_BRIDGE_PORT=$((BRIDGE_PORT + ($UID - 999)))

local map_volume="apollo_map_volume-${map_name}_${USER}"
-->  local map_volume="apollo_map_volume-${map_name}_${USER}2"

local audio_volume="apollo_audio_volume_${USER}"
-->  local audio_volume="apollo_audio_volume_${USER}2"

local faster_rcnn_volume="apollo_faster_rcnn_volume_${USER}"
-->  local faster_rcnn_volume="apollo_faster_rcnn_volume_${USER}2"

local smoke_volume="apollo_smoke_volume_${USER}"
-->  local smoke_volume="apollo_smoke_volume_${USER}2"

change the container name in  ./docker/scripts/dev_into_multiuser.sh
DEV_CONTAINER="apollo_dev_${USER}"
-->  DEV_CONTAINER="apollo_dev_${USER}2"

```
* change the port related lines in ./scripts/bootstrap.sh or use immediately ./scripts/bootstrap_multiuser.sh



