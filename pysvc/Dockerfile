FROM python:3.11-bookworm

WORKDIR /app

RUN pip install --no-cache-dir opencv-python-headless numpy scipy posix_ipc

COPY ipc.py .
COPY nam.py .

ENTRYPOINT ["python3", "nam.py"]
