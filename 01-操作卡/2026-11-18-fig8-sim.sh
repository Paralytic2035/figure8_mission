#!/bin/bash
# 重启 figure8 仿真（虚拟机 ~/fig8/fig8-sim.sh）
# 一条命令做完：停旧仿真 → 借 DISPLAY → 起 Gazebo → 轮询自检直到就绪
# 用法：  bash ~/fig8/fig8-sim.sh
# 跑完再执行： bash ~/template2026_ws/tools/run_task.sh figure8

set -u
WS=$HOME/template2026_ws
PORT=11411

echo "===== 1. 停掉旧仿真 ====="
bash "$WS/tools/stop_sim.sh" 2>/dev/null || true
pkill -f 'gzserver|gzclient' 2>/dev/null || true
sleep 3

echo
echo "===== 2. 借 DISPLAY（Gazebo 窗口要画到 UTM 桌面上） ====="
for p in $(pgrep -u "$(id -un)" -f 'gnome-session-binary|gnome-shell|ibus-daemon|nautilus' 2>/dev/null); do
    d=$(tr '\0' '\n' < "/proc/$p/environ" 2>/dev/null | grep -m1 '^DISPLAY=')
    [ -z "$d" ] && continue
    a=$(tr '\0' '\n' < "/proc/$p/environ" 2>/dev/null | grep -m1 '^XAUTHORITY=')
    export "$d"
    [ -n "$a" ] && export "$a"
    break
done
[ -z "${DISPLAY:-}" ] && for s in /tmp/.X11-unix/X*; do [ -S "$s" ] && export DISPLAY=":${s##*X}" && break; done
export LIBGL_ALWAYS_SOFTWARE=1
echo "DISPLAY=${DISPLAY:-空}"
if [ -z "${DISPLAY:-}" ]; then
    echo "!! DISPLAY 是空的：去 UTM 桌面开个终端敲 xhost +local: 再重跑"
    exit 1
fi

echo
echo "===== 3. 起仿真（1-2 分钟，别 Ctrl+C） ====="
cd "$WS" && bash tools/start_sim.sh --gui
# 这个脚本最后常报 "Startup failed"，那是它自检超时的误报，仿真其实在跑

echo
echo "===== 4. 轮询自检 ====="
source /opt/ros/noetic/setup.bash 2>/dev/null || true
source "$WS/devel/setup.bash" 2>/dev/null || true
for i in $(seq 1 20); do
    if ROS_MASTER_URI="http://127.0.0.1:$PORT" python3 "$WS/tools/check_ready.py" 2>/dev/null; then
        echo
        echo "✅ 就绪。下一步："
        echo "   bash ~/template2026_ws/tools/run_task.sh figure8"
        exit 0
    fi
    echo "  还没就绪（第 $i 次），等 10 s ..."
    sleep 10
done
echo "!! 3 分钟仍没就绪。看 UTM 桌面有没有 Gazebo 窗口，或 bash ~/template2026_ws/tools/stop_sim.sh 重来"
exit 1
