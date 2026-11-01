# Deployment: the rover runs one Docker image

The rover is a **Jetson Orin**. It runs the whole stack from one image,
`ghcr.io/alexanderhalley/lunabot_drive/robot:<tag>`, started by `docker/compose.robot.yml`. The
tag is the deployment: it names exactly which commit is on the rover, and putting the previous tag
back is the rollback.

| | |
|---|---|
| Image | `docker/Dockerfile.robot`: ROS 2 Jazzy + rosdep deps + the workspace built Release into `/opt/lunabot` |
| Built by | CI (`robot-image` job), natively for `arm64` (Orin) and `amd64` (desktop), on every push and PR |
| Tested by | the same job: the mock stack from inside the image, driven, `check_stack.py --drive` |
| Published | pushes to `develop-2027` → `:develop-2027` and `:<commit sha>` (multi-arch) |
| Run by | `docker/compose.robot.yml` + `docker/robot.env` (rover) or `docker/sim.env` (desktop + Isaac) |

## Why Docker on the Orin

JetPack 6, the current release for Orin, is Ubuntu **22.04**, whose ROS is Humble. This workspace is
Jazzy, which is 24.04. The container brings its own 24.04 userspace, so the host OS stops mattering:
whatever JetPack is on the Orin, the same image runs. It also means the rover never compiles
anything: no build on target, no `install/` copied between architectures, no "works on the
desktop".

Nothing in the stack needs the Orin's GPU today: rtabmap runs on the CPU, the OAK-D computes depth on
the camera, and cuVSLAM is not in Jazzy (docs/SLAM.md). So the container runs without the NVIDIA
runtime. See [GPU](#gpu) for what changes when something does need it.

## The rover, once

`sudo docker/setup_rover_host.sh --user <rover user> [--can native|slcan|none]` does steps 1–5
below (idempotent; rerun after a reflash). The steps, for what it does and why:

1. **JetPack 6** on the Orin (SDK Manager). Docker usually comes with it; if `docker --version`
   fails: `sudo apt install docker.io docker-compose-v2`.
2. `sudo usermod -aG docker $USER`, then log out and back in.
3. **DDS receive buffer.** The image asks CycloneDDS for 10 MB (`docker/cyclonedds.xml`); the kernel
   caps it, and with the host network the cap is the host's:

   ```bash
   echo 'net.core.rmem_max=10485760' | sudo tee /etc/sysctl.d/60-lunabot-dds.conf
   sudo sysctl --system
   ```

   Without it the stack still starts, with a smaller buffer and dropped camera frames under load.
4. **CAN at boot** (1 Mbit/s, docs/HARDWARE_CAN.md). The container uses `can0`; the HOST brings it
   up. Native CAN on the Orin, with systemd-networkd:

   ```ini
   # /etc/systemd/network/80-can0.network
   [Match]
   Name=can0
   [CAN]
   BitRate=1M
   ```

   `sudo systemctl enable --now systemd-networkd`. For the 2026 USB adapter, run the `slcand`
   lines from HARDWARE_CAN.md in a systemd unit instead.
5. The data tree: `mkdir -p ~/lunabotics/data/{bags,logs/ros}` (bags and ROS logs land there).
6. The repository, for the compose and env files only: `git clone` it anywhere. Nothing is built.
7. If the package on ghcr.io is private (GitHub → the repo → Packages → robot → settings):
   `docker login ghcr.io -u <github user>` with a token that has `read:packages`.

## Deploy, update, roll back

```bash
cd lunabot_drive
docker/deploy.sh                 # pull, up -d, print the running commit, check_stack.py inside
```

`deploy.sh` exits non-zero, and leaves the containers up for a look, if `check_stack.py` fails
against the live graph. It never drives the rover. By hand, the same thing:

```bash
docker compose -f docker/compose.robot.yml --env-file docker/robot.env pull
docker compose -f docker/compose.robot.yml --env-file docker/robot.env up -d
docker compose -f docker/compose.robot.yml --env-file docker/robot.env logs -f stack
```

- `stack` runs `ros2 launch lunabot_bringup $LUNABOT_LAUNCH` (robot.env: real hardware, Nav2,
  mission control's field profile, `wait_for_start`, recording). `mission` runs `mission.py` with
  `autostart:=false`. Both restart after a crash or a power cut (`restart: unless-stopped`, and
  Docker starts at boot): the rover comes up in READY and **does not move until Start** in mission
  control (docs/MONITORING.md "Mission control").
- **Pin a commit for a competition run:** set `LUNABOT_IMAGE=...robot:<sha>` in robot.env. The
  branch tag moves with every push; a sha never does. Roll back by setting the previous sha and
  running `up -d` again.
- `docker compose ... stop` sends SIGINT and waits 30 s, so the bag gets closed properly.
- What is running: `docker inspect --format '{{ .Config.Labels }}' <container>` shows the commit;
  the stack also logs `lunabot image <sha>` first thing.
- A shell in the running image: `docker compose -f docker/compose.robot.yml --env-file
  docker/robot.env exec stack bash` (ROS is sourced in bash: `ros2 topic list`, `candump can0`).
  For one command without a shell, go through the entrypoint: `... exec stack robot_entrypoint.sh
  ros2 topic list` (`exec` skips it, so plain `ros2` is not on the PATH).
- Bags: `~/lunabotics/data/bags/run_<date>_<time>/`. ROS logs: `~/lunabotics/data/logs/ros/`.
  Both are written as root (the container runs as root for device access).

## The same image on the desktop, against Isaac

Before a tag goes on the rover, run it against the sim: Isaac natively as usual, then the amd64
build of the same image in place of the host workspace.

```bash
src/lunabot_sim/scripts/run_isaac_sim.sh --seed 7 --headless &      # wait for /clock
docker compose -f docker/compose.robot.yml --env-file docker/sim.env up
```

Then connect Foxglove (docs/MONITORING.md) and press Start. Building it locally instead of pulling:

```bash
docker build -f docker/Dockerfile.robot --build-arg GIT_SHA=$(git rev-parse HEAD) \
  -t ghcr.io/alexanderhalley/lunabot_drive/robot:develop-2027 .
```

Docker on the desktop is installed (2026-10-31); a new machine needs Docker Engine from Docker's
apt repository (docs.docker.com/engine/install/ubuntu), `sudo usermod -aG docker $USER` (then log
in again, or `sg docker -c ...`), and the same sysctl as the rover.

## The images

| File | What it is |
|---|---|
| `docker/Dockerfile.robot` | **The rover.** Source copied in and built; the image is the deployment unit |
| `docker/Dockerfile.jazzy` + `docker-compose.yml` + `.devcontainer/` | Development: source bind-mounted, build in the container |
| `docker/Dockerfile.isaac` | Isaac Sim in a container. Unmaintained (pins 4.5.0); Isaac runs natively on the desktop |

Third-party sources (`lunabot.repos`) are pinned to commits, so rebuilding an unchanged tree gives
the same image. Bump a pin deliberately and let CI build it.

## GPU

Not used yet. When something needs CUDA or TensorRT on the Orin (a TensorRT detector, Isaac ROS),
the container needs the NVIDIA container runtime (`runtime: nvidia` in compose) AND user-space CUDA
libraries matching the host's L4T driver — NVIDIA's L4T images are 22.04-based, so a Jazzy (24.04)
image with GPU access is a deliberate piece of work, not a flag. Build TensorRT engines on the Orin
itself (lunabotics-filesystem-setup.md §8).

## What has been verified (2026-10-31, desktop, amd64)

- The image builds (cold: ~7 min; after a code-only change the dependency layers are cached).
- Mock stack inside it: `check_stack.py --drive`, 13/13.
- **The image against Isaac**, started exactly as above (compose + sim.env), driven through
  foxglove_bridge by `~/lunabotics/tools/mission_control/live_check.sh` with `CONTAINER=1`:
  wait-for-start, e-stop (wheels stopped in 1.2 s), pause during the dig, the operator camera,
  the field profile refusing a laptop publish, abort, and the bag in `data/bags`.
- `docker/deploy.sh docker/sim.env --no-pull` against Isaac: up, commit printed, check_stack
  17/17, `compose down` in 2 s with both containers exiting 0.
- What those runs found and fixed: `ros2 run` does not forward SIGINT (the mission container was
  SIGKILLed on stop, so it runs the script directly); `-p` before `--params-file` is overridden
  by the file (later wins); a hard 10 MB DDS buffer minimum stops every node on a kernel with a
  lower cap; `compose exec` skips the entrypoint (ROS not sourced: deploy.sh goes through it, and
  the image's bashrc sources ROS); check_stack.py failed every ekf + rtabmap stack because two
  nodes advertise /tf with their TF output turned off (it now reads their gate parameters);
  the image was in UTC (bag names); `nav2_bringup` pulled in Gazebo, and once it was
  gone the Smac planner and RPP controller were missing: lunabot_navigation never declared the Nav2
  plugins it loads (now declared, and test_nav2_params.py checks every `plugin:` package is).

## Not done yet

- **Not yet run on the Orin itself.** CI builds and smoke-tests the arm64 image on an arm64
  runner; the first run on the rover proves the devices (CAN, OAK-D over USB, joystick).
- **Size: 6.2 GB on disk, 1.4 GB to pull.** No apt recommends and no nav2_bringup already took
  1.6 GB off. Most of the rest is build-time only (PCL/VTK headers and their Java, compilers) or
  rviz2's graphics stack (several LLVMs): an exec-only runtime stage would drop it, once CI can
  prove that image still runs.
