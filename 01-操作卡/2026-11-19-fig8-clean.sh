#!/bin/bash
# 飞完清理：把 figure8 的源码、二进制、数据从机载电脑上删干净（防止别的组直接拿去飞）
# 用法：
#   bash fig8-clean.sh        只预览，什么都不删
#   bash fig8-clean.sh --go   真删
# Mac 上留着完整源码，删了不会丢；官方 square 不受影响。
set -u

WS=${WS:-$HOME/template2026_ws}
INC=$WS/src/template/include
GO=0
[ "${1:-}" = "--go" ] && GO=1
say() { echo "  $*"; }

echo "===== 1. 待删：figure8 的四个文件 + 二进制 ====="
FILES=(
  "$WS/src/mission/src/figure8.cpp"
  "$WS/src/mission/config/figure8.yaml"
  "$WS/src/mission/config/figure8.yaml.final"
  "$WS/src/mission/config/figure8.yaml.bak"*
  "$WS/src/mission/launch/figure8.launch"
  "$WS/shell/figure8.sh"
  "$WS/devel/lib/mission/figure8"
)
for f in "${FILES[@]}"; do
  for g in $f; do [ -e "$g" ] && say "$g"; done
done

echo
echo "===== 2. 待删：安装包与源码副本 ====="
for d in "$HOME/fig8" "$HOME/fig8v2"; do
  [ -d "$d" ] && say "$d/  （整个目录，里面有 additions.h）"
done
ls -1 "$HOME"/fig8*.tgz 2>/dev/null | while read -r f; do say "$f"; done
ls -1 "$HOME"/2026-11-18-fig8*.tgz 2>/dev/null | while read -r f; do say "$f"; done

echo
echo "===== 3. 待删：飞行数据 ====="
ls -1 "$HOME"/.ros/fig8-*.csv 2>/dev/null | while read -r f; do say "$f"; done
ls -1d "$WS/validation/runtime/roslog"/* 2>/dev/null | while read -r f; do say "$f  （ros 日志，可选）"; done

echo
echo "===== 4. template.h 里的追加段（这是真正的源码，必须清） ====="
if [ -f "$INC/template.h" ]; then
  echo "  当前 traj_send 出现次数: $(grep -c 'traj_send' "$INC/template.h" 2>/dev/null || true)"
  echo "  备份文件：$(ls -1 "$INC"/template.h.bak.* 2>/dev/null | wc -l | tr -d ' ') 个"
fi

echo
echo "===== 5. CMakeLists 里的 figure8 目标 ====="
echo "  出现次数: $(grep -c 'figure8' "$WS/src/mission/CMakeLists.txt" 2>/dev/null || true)"

if [ "$GO" = 0 ]; then
  echo
  echo "（预览模式，什么都没删。确认后重跑：bash $0 --go）"
  exit 0
fi

echo
echo "===== 开始删除 ====="
rm -fv "$WS/src/mission/src/figure8.cpp" \
       "$WS/src/mission/config/figure8.yaml" \
       "$WS/src/mission/config/figure8.yaml.final" \
       "$WS/src/mission/launch/figure8.launch" \
       "$WS/shell/figure8.sh" 2>/dev/null
rm -f  "$WS/src/mission/config/figure8.yaml.bak"*
rm -rf "$HOME/fig8" "$HOME/fig8v2"
rm -f  "$HOME"/fig8*.tgz "$HOME"/2026-11-18-fig8*.tgz
rm -f  "$HOME"/.ros/fig8-*.csv
rm -f  "$WS/devel/lib/mission/figure8"
rm -rf "$WS/build/mission/CMakeFiles/figure8.dir"

echo
echo "===== 还原 template.h（取最早的备份 = 追加之前的原件） ====="
BAK=$(ls -1tr "$INC"/template.h.bak.* 2>/dev/null | head -1)
if [ -n "${BAK:-}" ]; then
  cp -v "$BAK" "$INC/template.h"
  echo "  已用 $BAK 还原"
  if [ "$(grep -c 'traj_send' "$INC/template.h" 2>/dev/null || true)" != "0" ]; then
    echo "  !! 还原后仍有追加段，改用截断法"
    python3 - "$INC/template.h" <<'PY2'
import sys
p = sys.argv[1]
s = open(p).read()
key = "函数 11~16：连续轨迹跟踪原语"
i = s.find(key)
if i >= 0:
    j = s.rfind("/****", 0, i)
    open(p, "w").write(s[:j if j >= 0 else i])
    print("  已截掉追加段")
PY2
  fi
else
  echo "  !! 没找到 template.h.bak.*，改用截断法清追加段"
  python3 - "$INC/template.h" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
key = "函数 11~16：连续轨迹跟踪原语"
i = s.find(key)
if i >= 0:
    j = s.rfind("/****", 0, i)
    open(p, "w").write(s[:j if j >= 0 else i])
    print("  已截掉追加段")
else:
    print("  文件里没有追加段")
PY
fi

echo
echo "===== 还原 CMakeLists.txt ====="
C=$WS/src/mission/CMakeLists.txt
BAK=$(ls -1tr "$C".bak.* 2>/dev/null | head -1)
if [ -n "${BAK:-}" ]; then
  cp -v "$BAK" "$C"
  if [ "$(grep -c 'figure8' "$C" 2>/dev/null || true)" != "0" ]; then
    echo "  !! 备份里还带 figure8，直接删行"
    sed -i '/add_executable(figure8/d; /target_link_libraries(figure8/d; /add_dependencies(figure8/d' "$C"
  fi
else
  echo "  没找到备份，直接删掉 figure8 的三行"
  sed -i '/add_executable(figure8/d; /target_link_libraries(figure8/d; /add_dependencies(figure8/d' "$C"
fi

echo
echo "===== 重新编译（确保没有残留二进制） ====="
cd "$WS" && catkin_make -j2 2>&1 | tail -12

echo
echo "===== 验收（四条都要对） ====="
echo "1) template.h 里 traj_send 次数（应为 0）: $(grep -c 'traj_send' "$INC/template.h" 2>/dev/null || true)"
echo "2) CMakeLists 里 figure8 次数（应为 0）: $(grep -c 'figure8' "$C" 2>/dev/null || true)"
echo "3) devel/lib/mission 下应只剩官方三个："
ls -1 "$WS/devel/lib/mission" 2>/dev/null | sed 's/^/     /'
echo "4) 源码目录与包（应为空）："
ls -1 "$WS/src/mission/src" | sed 's/^/     /'
tgz=$(ls -1 "$HOME"/fig8*.tgz "$HOME"/2026-11-18-fig8*.tgz 2>/dev/null | wc -l | tr -d ' ')
echo "     残留 tgz 数量: ${tgz}（应为 0）"

echo
echo "===== 官方 square 仍可用（自己判断） ====="
echo "  （官方 square/hover/avoidance 的源码没被动过，删完照常可用）"
