#!/bin/bash
# spawn_objects.sh - populate the running Gazebo world with COCO-class
# objects (people, cars, trucks) plus buildings and trees so the YOLO
# detector has interesting things to see.
#
# Why a runtime spawn (not in the world SDF):
#   Including Fuel-model URIs in the world's <include> blocks forces Gazebo
#   to download every model before the world is ready, which causes the
#   PX4 launch to time out. Spawning at runtime via `gz service` calls
#   sidesteps that — the simulator is already up and stable.
#
# Prerequisites: PX4 SITL must already be running with the x500_mono_cam
# airframe (pxh> prompt visible) and the gz CLI on PATH.
#
# Drone spawns at origin, looking +X. Objects are arranged across a ~120°
# arc in front of and around the drone to exercise the wide camera FOV.
#
# Usage:
#   ./spawn_objects.sh            # spawn all entities
#   ./spawn_objects.sh -h         # show help
case "${1:-}" in
  -h|--help)
    sed -n '2,22p' "$0"
    exit 0
    ;;
esac

set -e

spawn_model() {
  local name=$1
  local pose=$2
  local uri=$3
  echo "Spawning ${name}..."
  gz service -s /world/default/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "sdf: '<?xml version=\"1.0\"?><sdf version=\"1.6\"><include><name>${name}</name><pose>${pose}</pose><uri>${uri}</uri></include></sdf>'"
}

# === People (COCO class: person) ===
# Spread laterally so wide FOV is exercised
spawn_model "person1" "8 0 0 0 0 1.57"     "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing person"
spawn_model "person2" "10 4 0 0 0 -1.0"    "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Casual female"
spawn_model "person3" "12 -3 0 0 0 1.0"    "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Walking person"
spawn_model "person4" "15 6 0 0 0 -1.57"   "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing person"

# === Vehicles (COCO classes: car, truck) ===
spawn_model "car1"     "18 -6 0 0 0 0.5"   "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Suv"
spawn_model "car2"     "22 4 0 0 0 -0.3"   "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Hatchback"
spawn_model "truck1"   "25 -4 0 0 0 1.2"   "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pickup"

# === Buildings (visual variety; not COCO classes but improve the scene) ===
spawn_model "building1" "30 10 0 0 0 0"    "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Residential"
spawn_model "building2" "35 -12 0 0 0 1.57" "https://fuel.gazebosim.org/1.0/OpenRobotics/models/School"

# === Trees (visual variety) ===
spawn_model "tree1"   "5 -8 0 0 0 0"       "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Oak tree"
spawn_model "tree2"   "8 9 0 0 0 0"        "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine Tree"
spawn_model "tree3"   "20 -10 0 0 0 0"     "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine Tree"

echo ""
echo "Spawn complete. ${0} populated 12 entities into /world/default."
echo ""
echo "Next: in another terminal, view detections with"
echo "  ros2 run rqt_image_view rqt_image_view"
echo "and select /yolo/annotated_image from the topic dropdown."