FROM python:3.11-bookworm

WORKDIR /app

RUN pip install --no-cache-dir opencv-python-headless numpy

COPY main.py .

ENTRYPOINT ["python3", "main.py"]
