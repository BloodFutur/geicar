# Use the official ROS Humble perception image as the base
FROM ros:humble-perception

# Arguments for user configuration
ARG USERNAME=pi
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Set environment variables
ENV SHELL /bin/bash

# Update system (not useful)
# RUN apt-get update && apt-get upgrade -y

# Install dependencies required for building packages
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
    # Intel GPU drivers
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    # Dépendances pour PyTorch 
    libopenblas-dev \
    libomp-dev \
    # Dépendances pour TensorFlow
    libhdf5-dev \
    libc-ares-dev \
    libeigen3-dev \
    libatlas-base-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install Python libraries for AI training and data vision: 
RUN pip3 install --no-cache-dir --upgrade pip \
    && pip3 install --no-cache-dir \
    tensorflow \
    torch torchvision torchaudio \
    numpy \
    pandas \
    matplotlib \
    scikit-learn \
    opencv-python-headless \
    pillow \
    seaborn \
    jupyter \
    ipykernel
# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN echo $USERNAME:$USERNAME | chpasswd

# Source the ROS setup.bash file.
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
# Enable SSH
RUN echo "sudo service ssh restart > /dev/null" >> /home/$USERNAME/.bashrc

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["service ssh restart"]

# Set the entrypoint to bash
ENTRYPOINT ["/bin/bash"]