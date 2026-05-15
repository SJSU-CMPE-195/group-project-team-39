FROM torizon/debian:3-bookworm

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 python3-pip \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Python source files
COPY calibration_config.py  /app/calibration_config.py
COPY vision_preprocess.py   /app/vision_preprocess.py
COPY green_puck_detect.py    /app/green_puck_detect.py
COPY camera.py              /app/camera.py
COPY ipc_handler.py         /app/ipc_handler.py
COPY main.py                /app/main.py

COPY crop_calibrate.py      /app/crop_calibrate.py
COPY test_detection.py      /app/test_detection.py

COPY requirements-ipc.txt /app/requirements-ipc.txt
RUN pip install --no-cache-dir --break-system-packages -r /app/requirements-ipc.txt
RUN pip install --no-cache-dir --break-system-packages opencv-python-headless numpy scipy

# Match cppsvc: run as the torizon user (uid 1000) so POSIX shm/sem
# perms line up between the two services.  Add video/plugdev so V4L2
# access to /dev/video* keeps working.
RUN usermod -a -G video,plugdev torizon
USER torizon

# This image installs opencv-python-headless (no GTK), so any cv2 GUI
# call would crash.  Force HEADLESS=1 so main.py skips namedWindow /
# imshow / waitKey.  Override at runtime if swapping to a GUI build.
ENV HEADLESS=1

ENTRYPOINT ["python3", "main.py"]
