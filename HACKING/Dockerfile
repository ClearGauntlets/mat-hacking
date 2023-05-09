FROM docker.io/debian

# You're going to have to add your user to the group that the device's port uses. That's usually dialout or uucp
RUN apt-get -y update && \
    apt-get -y install flex bison gperf python3 python3-pip python3-setuptools python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0 unzip openssl
RUN apt-get -y install vim git minicom # This is extra stuff I want to put in a separate layer
WORKDIR /root/esp
RUN git clone --recursive https://github.com/espressif/esp-idf.git -b v5.0.1
WORKDIR esp-idf
RUN ./install.sh esp32
RUN echo '. /root/esp/esp-idf/export.sh' >> /root/.bashrc
RUN echo 'cd /workspace' >> /root/.bashrc
