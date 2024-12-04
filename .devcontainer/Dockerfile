# Use the official ROS Humble perception image as the base
FROM ros:humble-perception

# Arguments for user configuration
ARG USERNAME=pi
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Set environment variables
ENV SHELL /bin/bash
ENV PYTHONPATH=/usr/local/lib/python3.10/dist-packages

# Update system dependencies
RUN apt-get update && apt-get install -y \
    sudo \
    python3-pip \
    openssh-server \
    net-tools \
    nano \
    vim \
    apt-utils \
    ufw \
    iputils-ping \
    software-properties-common \
    ros-humble-teleop-twist-keyboard \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    # ML/AI specific dependencies
    cmake \
    gfortran \
    libopenblas-dev \
    liblapack-dev \
    libhdf5-dev \
    # GPU support libraries
    ocl-icd-opencl-dev \
    clinfo

# Install Python ML libraries
RUN pip3 install --upgrade pip && \
    pip3 install \
    numpy \
    scipy \
    pandas \
    scikit-learn \
    matplotlib \
    seaborn \
    tensorflow \
    torch \
    torchvision \
    ultralytics \
    opencv-python \
    # Embedded AI specific libraries
    jetson-stats \
    onnx \
    onnxruntime

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN echo $USERNAME:$USERNAME | chpasswd

# Source the ROS setup.bash file.
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "sudo service ssh restart > /dev/null" >> /home/$USERNAME/.bashrc
# Nvidia Repo for Jetson
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && \
    apt-get update
# Install TensorRT (Jetson-specific for embedded AI)
RUN pip3 install tensorrt

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["service ssh restart"]

# Set the entrypoint to bash
ENTRYPOINT ["/bin/bash"]