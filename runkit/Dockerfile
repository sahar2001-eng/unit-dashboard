# The run kit: the camera's own code (unix MicroPython + x64 C modules) and
# the video tools, in a small Linux x64 box. run.py builds it once.
FROM --platform=linux/amd64 python:3.12-slim
RUN pip install --no-cache-dir numpy==1.26.4 opencv-python-headless==4.10.0.84 imageio-ffmpeg==0.5.1
WORKDIR /kit
COPY kit.json engine.py /kit/
COPY cam /kit/cam
RUN chmod +x /kit/cam/micropython
ENTRYPOINT ["python3", "/kit/engine.py"]
