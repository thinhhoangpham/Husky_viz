FROM ros:noetic-ros-core

# move-base-msgs = MoveBaseAction/Goal for the operator; ros-noetic-actionlib
# is pulled in by it. tf for quaternion_from_euler; nav/geometry msgs for
# telemetry types. NO gazebo. NO attack code.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-noetic-move-base-msgs ros-noetic-actionlib ros-noetic-tf iproute2 \
    && rm -rf /var/lib/apt/lists/*

# operator.py is bind-mounted with the repo at /repo at runtime.
WORKDIR /repo
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
