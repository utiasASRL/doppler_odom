docker run -it --name doppler_odom_cntnr \
  --privileged \
  --network=host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ${HOME}/ASRL:/home/ASRL:rw \
  doppler_odom_img
