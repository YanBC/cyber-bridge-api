FROM carlasim/carla:0.9.11 as carlasim
FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
apt-get install -yq vim git tree less lsof net-tools cmake wget curl telnet pciutils iproute2 iputils-ping sudo python3.7 python3-dev python3-pip

RUN cd /usr/bin/ && \
rm python3 && \
ln -s python3.7 python3 && \
ln -s python3 python && \
useradd -m -s /bin/bash carla && \
usermod -aG sudo carla && \
echo "carla:carla" | chpasswd

WORKDIR /third-parties/carla

RUN apt-get install -yq python3-setuptools
RUN python3.7 -m pip install --upgrade pip

COPY --from=carlasim /home/carla/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg .
COPY --from=carlasim /home/carla/PythonAPI/examples/requirements.txt .
COPY --from=carlasim /home/carla/PythonAPI/ .

RUN python3.7 -m easy_install carla-0.9.11-py3.7-linux-x86_64.egg
RUN python3.7 -m pip install -r requirements.txt

WORKDIR /third-parties

RUN git clone -b 0.9.11 https://github.com/carla-simulator/scenario_runner.git

WORKDIR /third-parties/carla
COPY --from=carlasim /home/carla/PythonAPI .

RUN apt-get install -yq python3-opencv && \
python3.7 -m pip install opencv-python && \
python3.7 -m pip install -r /third-parties/scenario_runner/requirements.txt

ENV PYTHONPATH=/third-parties/scenario_runner:/third-parties/carla:/third-parties/carla/carla
ENV SCENARIO_RUNNER_ROOT=/third-parties/scenario_runner

WORKDIR /workspace

COPY requirements.txt .
RUN python3.7 -m pip install -r requirements.txt && \
rm requirements.txt

USER carla
