FROM ubuntu:22.04

ARG git_username
ARG git_password 

LABEL my_label="pybullet trial"

COPY requirements.apt requirements/requirements.apt
COPY requirements.pip requirements/requirements.pip

RUN apt update && \
apt install sudo && \   
sudo apt update && \
sudo apt install $(cat requirements/requirements.apt| tr ' ' "\n") -y && \
python3 -m pip install -r requirements/requirements.pip


RUN echo "${git_username} ${git_password}">args.txt && \
git clone https://github.com/adubredu/pybullet_kitchen
