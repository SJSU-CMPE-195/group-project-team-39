FROM torizon/debian:3-bookworm

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 python3-pip \
    python3-opencv \
    ca-certificates \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY python_apalis.py /app/python_apalis.py

# -u forces unbuffered output, which guarantees the live FPS logs 
# show up instantly in your MCU's docker logs without getting trapped.
ENTRYPOINT ["python3", "-u", "/app/python_apalis.py"]
