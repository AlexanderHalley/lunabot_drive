#!/usr/bin/env bash
#
# Deploy the rover image and check it came up: pull, start, report which commit
# is running, then run check_stack.py inside the stack container against the
# live graph. Exits non-zero if the check fails. docs/DEPLOYMENT.md.
#
#   docker/deploy.sh                       # the rover: docker/robot.env
#   docker/deploy.sh docker/sim.env        # the desktop, against Isaac (start Isaac first)
#   docker/deploy.sh docker/robot.env --no-pull
#
# It never drives: check_stack.py runs WITHOUT --drive, and the mission waits
# for Start (autostart false).

set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")/.."

ENV_FILE=${1:-docker/robot.env}
PULL=1
[[ "${2:-}" == --no-pull ]] && PULL=0
C=(docker compose -f docker/compose.robot.yml --env-file "$ENV_FILE")

# The launch line decides what check_stack must expect.
LAUNCH=$(sed -n 's/^LUNABOT_LAUNCH=//p' "$ENV_FILE")
case "$LAUNCH" in
  sim.launch.py*) PROFILE=sim ;;
  *hw:=real*) PROFILE=real ;;
  *) PROFILE=mock ;;
esac
SLAM=$(grep -oE 'slam:=[a-z]+' <<<"$LAUNCH" | cut -d= -f2 || true)
SLAM=${SLAM:-none}

if [[ $PULL == 1 ]]; then
  echo "== pull"
  "${C[@]}" pull
fi
echo "== up"
"${C[@]}" up -d
echo "== image: $("${C[@]}" exec -T stack printenv LUNABOT_GIT_SHA)"

# Under sim, the stack waits for /clock; give Nav2 and SLAM time to come up.
echo "== waiting 40 s for the graph to settle"
sleep 40
echo "== check_stack --profile $PROFILE --slam $SLAM"
# Through the entrypoint: `compose exec` skips it, so plain `ros2` is not on PATH.
if "${C[@]}" exec -T stack robot_entrypoint.sh ros2 run lunabot_bringup check_stack.py \
     --profile "$PROFILE" --slam "$SLAM" --timeout 60; then
  echo "== DEPLOYED: the stack passed. Mission control: Foxglove -> ws://$(hostname -I | cut -d' ' -f1):8765"
else
  echo "== CHECK FAILED -- the containers are left running for a look:" >&2
  echo "   ${C[*]} logs --tail 100 stack" >&2
  exit 1
fi
