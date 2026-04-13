FROM python:3.11-bookworm

WORKDIR /app

RUN pip install --no-cache-dir opencv-python-headless numpy

COPY main.py .

ENTRYPOINT ["python3", "main.py"]


# FROM torizon/debian:3-bookworm

# RUN apt-get update && apt-get install -y --no-install-recommends \
#     python3 python3-pip \
#     ca-certificates \
#  && rm -rf /var/lib/apt/lists/*

# WORKDIR /app
# COPY main.py /app/main.py

# ENTRYPOINT ["python3", "main.py"]