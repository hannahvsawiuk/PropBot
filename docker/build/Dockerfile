# Start with Ubuntu 18.04 ros melodic full desktop
FROM osrf/ros:melodic-desktop-full-bionic

# Update the system
RUN apt-get update && apt-get upgrade -y

# Install some basic dev dependencies
RUN apt-get update && apt-get install -y vim curl git sudo

RUN apt-get install -y firefox

# Create container user
ARG USERNAME
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID -o $USERNAME && \
        useradd -m -u $UID -g $GID -G sudo -o -s /bin/bash $USERNAME && \
        echo $USERNAME "ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/90-sudonopass && \
        chmod 440 /etc/sudoers.d/90-sudonopass
USER $USERNAME

# Go to home
WORKDIR /home/$USERNAME

ENTRYPOINT ["/bin/bash"]
CMD []

# Copy in the bashrc
COPY bashrc .bashrc
