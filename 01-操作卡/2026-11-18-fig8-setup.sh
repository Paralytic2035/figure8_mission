#!/bin/bash
# figure8 上机脚本（虚拟机 / 机载电脑通用）
# 幂等：重复跑不会重复追加；默认只装+编译，不动 square/hover/avoidance 的任何文件
#
#   bash fig8-setup.sh           装 + 编译
#   bash fig8-setup.sh --check   只体检，不改任何文件
#
# 需要同目录下有：figure8.cpp / figure8.yaml / figure8.launch / figure8.sh / additions.h
set -e

HERE=$(cd "$(dirname "$0")" && pwd)
WS=${WS:-$HOME/template2026_ws}
STAMP=$(date +%m%d-%H%M)
CHECK_ONLY=0
[ "$1" = "--check" ] && CHECK_ONLY=1

echo "===== 0. 体检 ====="
[ -d "$WS/src/mission" ]  || { echo "!! 找不到 $WS/src/mission";  exit 1; }
[ -d "$WS/src/template" ] || { echo "!! 找不到 $WS/src/template"; exit 1; }
for f in figure8.cpp figure8.yaml figure8.launch figure8.sh additions.h; do
    [ -f "$HERE/$f" ] || { echo "!! 缺 $HERE/$f（压缩包没解全？）"; exit 1; }
done
echo "工作空间 : $WS"
echo "安装包   : $HERE"
echo "ROS      : ${ROS_DISTRO:-未 source}"

echo
echo "===== 1. 拷文件（只新增 figure8 四件套） ====="
if [ "$CHECK_ONLY" = "0" ]; then
    # 仿真工作空间可能没有 shell/ 目录（实机版才有），缺了就建
    mkdir -p "$WS/src/mission/src" "$WS/src/mission/config" "$WS/src/mission/launch" "$WS/shell"
    install -m 644 "$HERE/figure8.cpp"    "$WS/src/mission/src/figure8.cpp"
    install -m 644 "$HERE/figure8.yaml"   "$WS/src/mission/config/figure8.yaml"
    install -m 644 "$HERE/figure8.launch" "$WS/src/mission/launch/figure8.launch"
    install -m 755 "$HERE/figure8.sh"     "$WS/shell/figure8.sh"
    echo "[ok] figure8.cpp / .yaml / .launch / .sh 就位"
else
    echo "[check] 跳过"
fi

echo
echo "===== 2. template.h 追加连续轨迹原语 ====="
H=$WS/src/template/include/template.h
if grep -q "函数 11~16：连续轨迹跟踪原语" "$H"; then
    echo "[skip] 已含追加段"
elif [ "$CHECK_ONLY" = "1" ]; then
    echo "[check] 未追加"
else
    cp "$H" "$H.bak.$STAMP"
    cat "$HERE/additions.h" >> "$H"
    echo "[ok] 追加 $(wc -l < "$HERE/additions.h") 行；备份 $(basename "$H").bak.$STAMP"
fi
echo "  traj_send 出现次数: $(grep -c 'traj_send' "$H" || true)（应 ≥1）"

echo
echo "===== 3. mission/CMakeLists.txt 追加 figure8 目标 ====="
C=$WS/src/mission/CMakeLists.txt
if grep -q "add_executable(figure8" "$C"; then
    echo "[skip] 已含 figure8 目标"
elif [ "$CHECK_ONLY" = "1" ]; then
    echo "[check] 未追加"
else
    cp "$C" "$C.bak.$STAMP"
    cat >> "$C" <<'EOF'

add_executable(figure8 src/figure8.cpp)
target_link_libraries(figure8 ${catkin_LIBRARIES})
add_dependencies(figure8 ${catkin_EXPORTED_TARGETS})
EOF
    echo "[ok] 追加 3 行；备份 CMakeLists.txt.bak.$STAMP"
fi

[ "$CHECK_ONLY" = "1" ] && exit 0

echo
echo "===== 4. 编译 ====="
cd "$WS"
catkin_make -j2 2>&1 | tail -25

echo
echo "===== 5. 结果 ====="
if [ -x "$WS/devel/lib/mission/figure8" ]; then
    echo "✅ 二进制就位"
    ls -l "$WS/devel/lib/mission/figure8"
    echo
    echo "下一步（仿真）: roslaunch mission figure8.launch if_debug:=1"
else
    echo "❌ 没生成 figure8，把上面的报错发回来"
    exit 1
fi
