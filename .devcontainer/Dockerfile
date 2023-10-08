FROM ros:humble


ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip ssh

# Install setuptools version that doesn't have deprecation warnings
RUN pip install setuptools==58.2.0
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

COPY --chown=$USERNAME:$USER_UID .bashrc /home/$USERNAME/.bashrc

USER $USERNAME

ENTRYPOINT [ "entrypoint.sh", "/bin/bash" ]