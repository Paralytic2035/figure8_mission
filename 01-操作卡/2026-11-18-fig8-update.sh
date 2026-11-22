#!/bin/bash
# 更新已装好的 figure8（替换 template.h 里的追加段 + 覆盖任务源 + 重编）
# 用法：bash fig8-update.sh
set -e

HERE=$(cd "$(dirname "$0")" && pwd)
WS=${WS:-$HOME/template2026_ws}
H=$WS/src/template/include/template.h
STAMP=$(date +%m%d-%H%M)

[ -f "$H" ] || { echo "!! 找不到 $H"; exit 1; }
[ -f "$HERE/additions.h" ] || { echo "!! 缺 $HERE/additions.h"; exit 1; }

echo "===== 0. 备份 ====="
cp "$H" "$H.bak.$STAMP"
echo "已备份 $H.bak.$STAMP"

echo
echo "===== 1. 截掉旧的追加段 ====="
python3 - "$H" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
key = "函数 11~16：连续轨迹跟踪原语"
i = s.find(key)
if i < 0:
    raise SystemExit("!! 文件里找不到追加段标记（是不是没装过 figure8？）")
j = s.rfind("/****", 0, i)
if j < 0:
    j = i
open(p, "w").write(s[:j])
print("已截掉旧段，保留 %d 行" % s[:j].count("\n"))
PY

echo
echo "===== 2. 追加新版 ====="
cat "$HERE/additions.h" >> "$H"
echo "[ok] 追加 $(wc -l < "$HERE/additions.h") 行"
grep -c "TRAJ_CTRL_POS " "$H" || true

echo
echo "===== 3. 覆盖任务节点与配置 ====="
install -m 644 "$HERE/figure8.cpp"  "$WS/src/mission/src/figure8.cpp"
install -m 644 "$HERE/figure8.yaml" "$WS/src/mission/config/figure8.yaml"
echo "[ok] figure8.cpp / figure8.yaml 已更新"
grep -n "^ctrl_mode" "$WS/src/mission/config/figure8.yaml"

echo
echo "===== 4. 编译 ====="
cd "$WS"
catkin_make -j2 2>&1 | tail -20

echo
echo "===== 5. 结果 ====="
if [ -x "$WS/devel/lib/mission/figure8" ]; then
    echo "✅ 更新完成"
    ls -l "$WS/devel/lib/mission/figure8"
else
    echo "❌ 没生成 figure8；回滚用：cp $H.bak.$STAMP $H"
    exit 1
fi
