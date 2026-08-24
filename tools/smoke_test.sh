#!/usr/bin/env bash
# 端到端冒烟验证: lidar_sim → datacache_node → 触发事件 → zstd 落盘 →
# record_reader 校验/导出 → RPC(receiver_node) 回传闭环。
# 用法: 在已 source ROS2 与 install 环境的 shell 中执行 tools/smoke_test.sh
set -euo pipefail

SHARE="$(ros2 pkg prefix datacache)/share/datacache"
WORK=/tmp/datacache_smoke
INBOX=$WORK/upload_inbox
EXPORT=$WORK/export
CONFIG=$WORK/config.txt

echo "=== [0] 清理与准备 ==="
cleanup() {
    pkill -f upload_receiver_node 2>/dev/null || true
    pkill -f datacache_node 2>/dev/null || true
    pkill -f event_router_node 2>/dev/null || true
    pkill -f lidar_sim_node 2>/dev/null || true
    pkill -f keyboard_trigger_node 2>/dev/null || true
}
trap cleanup EXIT
cleanup
sleep 1
rm -rf "$WORK"
mkdir -p "$WORK"
cp "$SHARE/config.txt" "$CONFIG"
sed -i 's/upload_enabled=false/upload_enabled=true/' "$CONFIG"
# 缩短手动按键事件窗口加快验证
sed -i 's/event_manual_capture_pre_time=5/event_manual_capture_pre_time=2/' "$CONFIG"
sed -i 's/event_manual_capture_post_time=5/event_manual_capture_post_time=0/' "$CONFIG"

echo "=== [1] 启动 RPC 接收端与节点 ==="
(cd "$WORK" && nohup ros2 run datacache upload_receiver_node "$INBOX" \
    > "$WORK/receiver.log" 2>&1 &)
(cd "$WORK" && nohup ros2 run datacache lidar_sim_node \
    --ros-args -p pcd_path:="$SHARE/pcd/sample.pcd" \
    > "$WORK/lidar.log" 2>&1 &)
(cd "$WORK" && nohup ros2 run datacache datacache_node \
    --ros-args -p config_path:="$CONFIG" > "$WORK/node.log" 2>&1 &)
(cd "$WORK" && nohup ros2 run datacache event_router_node \
    --ros-args -p cooldown_ms:=0 > "$WORK/router.log" 2>&1 &)
sleep 4

ros2 service list 2>/dev/null | grep -q upload_store
echo "<- RPC 接收端服务已就绪"
grep -q "LidarSim\|publish" "$WORK/lidar.log"
echo "雷达仿真已启动"
grep "initialized" "$WORK/node.log" | tail -1
grep "Event router ready" "$WORK/router.log" | tail -1

echo "=== [2] 模拟按键 m 触发 manual_capture 事件 ==="
(cd "$WORK" && (sleep 1; printf 'm') | timeout 20 ros2 run datacache keyboard_trigger_node \
    --ros-args -p wait_for_upload:=true \
    > "$WORK/keyboard.log" 2>&1 &)

echo "=== [3] 等待落盘 + 回传 (6s) ==="
sleep 6

shopt -s nullglob
EVENT_DIRS=("$WORK"/records/manual_capture_*)
if [ "${#EVENT_DIRS[@]}" -eq 0 ]; then
    echo "!! 未找到按键事件目录"
    tail -20 "$WORK/keyboard.log"
    tail -20 "$WORK/node.log"
    exit 1
fi
EVENT_DIR="${EVENT_DIRS[0]}"
echo "事件目录: $EVENT_DIR"
ls -la "$EVENT_DIR"
test -f "$EVENT_DIR/.complete"

echo "=== [4] record_reader 概要与校验 ==="
ros2 run datacache record_reader "$EVENT_DIR"
ros2 run datacache record_reader "$EVENT_DIR" --verify
echo "verify 退出码: $? (0=通过)"
ros2 run datacache record_reader "$EVENT_DIR" --export "$EXPORT" --limit 3
echo "导出内容:"; ls "$EXPORT" 2>/dev/null

echo "=== [5] 回传闭环 ==="
test -f "$EVENT_DIR/.uploaded"
grep -q "accepted" "$WORK/keyboard.log"
grep -q "stored" "$WORK/keyboard.log"
grep -q "uploaded" "$WORK/keyboard.log"
echo ".uploaded 标记内容:"
cat "$EVENT_DIR/.uploaded"
echo "RPC 接收端落盘:"
find "$INBOX" -type f | head -20
test -n "$(find "$INBOX" -type f -print -quit)"
echo "接收端日志尾部:"
tail -12 "$WORK/receiver.log"

echo "=== [6] 收尾 ==="
cleanup
echo "冒烟验证完成"
