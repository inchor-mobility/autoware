# Autoware co-sim image (INC-345)
#
# WHY THIS BUILDS FROM SOURCE instead of basing on ghcr.io/autowarefoundation/autoware:
# this tree is a self-contained fork (origin: inchor-mobility, upstream: SaferDrive-AI).
# Its co-sim packages under src/mcity depend on autoware_auto_perception_msgs,
# autoware_auto_vehicle_msgs and autoware_auto_system_msgs — legacy message
# packages that modern upstream Autoware has dropped. The official image cannot
# satisfy them. All 309 packages (including those legacy msgs, vendored under
# src/core/external/) live here, so we compile the tree the way the host does.
#
# Build:  docker build -t autoware:dev .      # 2-4h cold, incremental after
# Run:    docker run --rm -it --gpus all --network host \
#           -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
#           -v $PWD/map:/autoware/map autoware:dev

# Global ARG so it can name a stage — COPY --from does not do variable expansion,
# so the swappable terasim source is declared as a named stage instead.
ARG TERASIM_IMAGE=terasim-cosim:dev
FROM ${TERASIM_IMAGE} AS terasim_pkgs

FROM ros:humble

ARG ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
# graphics: RViz renders over X11 (D5); compute/utility for any GPU node
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential cmake git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /autoware

# src only — see .dockerignore (build/, install/, log/, map/ stay out)
COPY src ./src

# rosdep is the risk concentrated in this image: this fork predates current
# upstream, so a dep may no longer resolve. Kept as its own layer so a failure
# surfaces in ~20 min instead of after the 4h compile.
# ros:humble has already run `rosdep init`, so only update is needed.
RUN apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*

# hiredis cannot come from rosdep: src/mcity/redis_client is a bare .h/.cpp pair
# with no package.xml (not a ROS package at all), which mcity_abc includes by
# relative path — so the <hiredis/hiredis.h> dependency is declared nowhere.
# The host happens to have libhiredis-dev installed by hand, which is exactly
# why this never surfaced until a clean build. Own layer to avoid invalidating
# the rosdep layer above.
RUN apt-get update && apt-get install -y --no-install-recommends libhiredis-dev \
    && rm -rf /var/lib/apt/lists/*

# The co-sim bridge (T2, TeraSim's scripts/autoware_cosim.py) runs in THIS
# container, not the terasim one: it imports rclpy and autoware_cosim_http —
# both ROS, present only here — and reaches the TeraSim service over HTTP.
# It does NOT need libsumo: autoware_cosim.py only parses .net.xml as text to
# compute the UTM offset. TeraSim's code itself is bind-mounted (see compose)
# and reached via PYTHONPATH; only these pure-python deps must be in the image.
RUN pip3 install --no-cache-dir --break-system-packages \
        requests pyyaml pyproj omegaconf loguru \
    || pip3 install --no-cache-dir requests pyyaml pyproj omegaconf loguru

# Build the tree. --continue-on-error is deliberate, not laziness: this fork is
# from ~2024 and some leaf packages no longer compile against current Humble
# patch releases. Example: system_monitor derives from rclcpp::Node but only
# includes diagnostic_updater.hpp, relying on it to pull rclcpp in transitively;
# newer diagnostic_updater tightened its headers, so it breaks. Such packages
# (system monitoring, sensor drivers, alternative localizers) are irrelevant to
# co-sim. Without this flag one broken leaf aborts the other 300.
# What actually matters is asserted in the verify layer below.
#
# build/ is a cache mount: incremental on retry and never lands in the image —
# which is also why --symlink-install is NOT used (its symlinks would dangle
# into a build/ that doesn't exist in the final image).
#
# On system_monitor: it cannot simply be skipped. colcon sources every
# dependency's package.sh before building a package, so skipping it fails
# tier4_system_launch (which exec_depends on it), which in turn drops
# autoware_launch — the one package co-sim actually needs. It is instead fixed
# at its root via a force-include in its CMakeLists (see the comment there).
RUN --mount=type=cache,target=/autoware/build \
    . /opt/ros/${ROS_DISTRO}/setup.sh \
    && (colcon build --continue-on-error \
            --cmake-args -DCMAKE_BUILD_TYPE=Release \
        || echo ">>> colcon reported failures — the verify layer decides if they matter") \
    && rm -rf log

# Acceptance: this image is good if the co-sim can run, not if all 309 packages
# built. Required = the mcity co-sim packages + what planning_simulator needs.
# Anything missing here fails the build.
RUN set -e; \
    for p in autoware_cosim autoware_cosim_http mcity_msgs mcity_route \
             preview_control gnss_decoder mcity_abc \
             autoware_launch tier4_simulator_launch \
             dummy_perception_publisher map_loader; do \
        if [ -d "install/$p" ]; then echo "OK built $p"; \
        else echo "MISSING (required) $p"; exit 1; fi; \
    done; \
    echo "--- packages built: $(ls install | grep -vcE '^(setup|local_setup|_local_setup|COLCON_IGNORE)') ---"

# --- TeraSim's python packages, for T2 (the co-sim bridge) ---
# run_experiment_cnde.py --autoware imports terasim_service, whose chain pulls
# pydantic -> redis -> terasim (Cython + eclipse-sumo). This build context cannot
# reach ../TeraSim, so lift the already-installed packages out of the terasim
# image instead of duplicating the install. Both images are ubuntu 22.04 /
# python 3.10, so the ABI matches. ROS's own python lives under
# /opt/ros/humble/... and is untouched by this.
#
# TeraSim is pip-installed -e over there, so these .pth files resolve to
# /app/TeraSim/packages/... — exactly where compose bind-mounts TeraSim here.
# Changing that mount path breaks T2.
#
# Slimmed in D6: terasim-cosim now installs only terasim / terasim-nde-nade /
# terasim-service, so tensorflow (1.9G, via terasim-datazoo) and fiftyone (246M,
# via terasim-envgen) are gone from this copy too — the co-sim path never
# imported them. Source is the `terasim_pkgs` stage (TERASIM_IMAGE global ARG).
COPY --from=terasim_pkgs /usr/local/lib/python3.10/dist-packages /usr/local/lib/python3.10/dist-packages

# Bake the TeraSim SOURCE too (packages + scripts), so the delivery image needs
# NO source bind-mount. The editable .pth finder copied above resolves to
# /app/TeraSim/packages/..., which must exist at this exact path — this provides
# it. configs/ and examples/maps/ are still volume-mounted at run time
# (client-editable data), overriding these baked copies. The terasim image's
# .dockerignore already kept maps/output/cnde-weights out, so this stays lean.
COPY --from=terasim_pkgs /app/TeraSim /app/TeraSim

# Co-sim entry (T3). use_sim_time:=true is REQUIRED under co-sim — without it
# TLS signals are silently dropped; standalone must NOT set it.
ENV AUTOWARE_INSTALL=/autoware/install
CMD ["bash", "-lc", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /autoware/install/setup.bash && exec bash"]
