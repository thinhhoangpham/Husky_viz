FROM ros:noetic-ros-core

# nmap = Phase 1 reachability scan; iproute2 = derive container IP for ROS_IP;
# ros-noetic-move-base-msgs = MoveBaseActionGoal type used by attack_goal.py.
RUN apt-get update \
    && apt-get install -y --no-install-recommends nmap iproute2 ros-noetic-move-base-msgs \
    && rm -rf /var/lib/apt/lists/*

# Attack scripts are bind-mounted here at runtime (see docker-compose.yml).
WORKDIR /repo
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
