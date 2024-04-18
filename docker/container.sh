docker run -it --name doppler_odom_cntnr \
  --privileged \
  --network=host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ${HOME}/Experiments:/home/ASRL/Experiments:rw \
  -v ${HOME}/Data:/home/ASRL/data:ro \
  doppler_odom_img
