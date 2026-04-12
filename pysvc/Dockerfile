FROM torizon/debian:3-bookworm

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 python3-pip \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY main.py /app/main.py
COPY requirements-ipc.txt /app/requirements-ipc.txt
RUN pip install --no-cache-dir --break-system-packages -r /app/requirements-ipc.txt

ENTRYPOINT ["python3", "/app/main.py"]
