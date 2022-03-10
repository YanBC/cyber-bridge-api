FROM carlasim/carla:0.9.11 as carlasim
FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

COPY ./CN.sources.list /etc/apt/sources.list

RUN apt-get update && \
    apt-get install -yq \
        vim \
        git \
        cmake \
        sudo \
        python3.7 \
        python3-dev \
        python3-pip \
        python3-opencv \
        python3-setuptools \
        protobuf-compiler \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /third-parties/carla
RUN python3.7 -m pip install --upgrade pip
COPY --from=carlasim /home/carla/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg .
COPY --from=carlasim /home/carla/PythonAPI/examples/requirements.txt .
COPY --from=carlasim /home/carla/PythonAPI ./PythonAPI
RUN python3.7 -m easy_install -i https://mirrors.aliyun.com/pypi/simple carla-0.9.11-py3.7-linux-x86_64.egg && \
    python3.7 -m pip install -r requirements.txt -i https://mirrors.aliyun.com/pypi/simple && \
    rm carla-0.9.11-py3.7-linux-x86_64.egg requirements.txt

WORKDIR /simulator
COPY . .

RUN bash compile_proto.sh
RUN python3.7 -m pip install -i https://mirrors.aliyun.com/pypi/simple opencv-python
RUN python3.7 -m pip install -r ./scenario_runner/requirements.txt -i https://mirrors.aliyun.com/pypi/simple
RUN python3.7 -m pip install -r ./requirements.txt -i https://mirrors.aliyun.com/pypi/simple

ENV PYTHONPATH=/simulator/scenario_runner:/third-parties/carla/PythonAPI:/third-parties/carla/PythonAPI/carla
ENV SCENARIO_RUNNER_ROOT=/simulator/scenario_runner

RUN cd /usr/bin/ && \
rm python3 && \
ln -s python3.7 python3 && \
ln -s python3 python && \
useradd -m -s /bin/bash carla && \
usermod -aG sudo carla && \
echo "carla:carla" | chpasswd && \
chown -R carla:carla /simulator
USER carla
