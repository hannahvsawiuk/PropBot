# Start with Ubuntu 18.04 ros melodic full desktop
FROM osrf/ros:melodic-desktop-full-bionic

# Update the system
RUN apt-get update && apt-get upgrade -y

RUN apt-get install -y firefox

# Create container user
ARG USERNAME
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID -o $USERNAME && \
        useradd -m -u $UID -g $GID -o -s /bin/bash $USERNAME
USER $USERNAME

# Go to home
WORKDIR /home/$USERNAME

ENTRYPOINT ["/bin/bash"]
CMD []

# Copy in the bashrc
COPY bashrc .bashrc
