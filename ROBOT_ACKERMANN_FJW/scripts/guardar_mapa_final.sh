#!/usr/bin/env bash
# Guarda el mapa REAL publicado por SLAM Toolbox como YAML y PGM.
set -euo pipefail
WS="${HOME}/robot_fjw_ws"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
mkdir -p "$WS/mapas"
if ! ros2 topic list | grep -qx '/map'; then
  echo 'ERROR: no se publica /map. Deja Gazebo y SLAM abiertos.' >&2
  exit 1
fi
OUT="$WS/mapas/mapa_fjw"
echo "Guardando /map en ${OUT}.pgm y ${OUT}.yaml ..."
ros2 run nav2_map_server map_saver_cli -f "$OUT" \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=20.0 \
  -p map_subscribe_transient_local:=true
for f in "$OUT.pgm" "$OUT.yaml"; do
  if [[ ! -s "$f" ]]; then
    echo "ERROR: no se generó el archivo o está vacío: $f" >&2
    exit 2
  fi
done
printf '\nMAPA GUARDADO CORRECTAMENTE:\n'
ls -lh "$OUT.pgm" "$OUT.yaml"
