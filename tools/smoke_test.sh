#!/usr/bin/env bash
# 端到端冒烟验证: lidar_sim → datacache_node → 触发事件 → zstd 落盘 →
# record_reader 校验/导出 → mock_server 回传闭环。
# 用法: 在已 source ROS2 与 install 环境的 shell 中执行 tools/smoke_test.sh
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WORK=/tmp/datacache_smoke
INBOX=$WORK/upload_inbox
EXPORT=$WORK/export
CONFIG=$WORK/config.txt

echo "=== [0] 清理与准备 ==="
pkill -f mock_server.py 2>/dev/null
pkill -f datacache_node 2>/dev/null
pkill -f lidar_sim_node 2>/dev/null
sleep 1
rm -rf "$WORK"
mkdir -p "$WORK"
cp "$ROOT/install/datacache/share/datacache/config.txt" "$CONFIG"
sed -i 's/upload_enabled=false/upload_enabled=true/' "$CONFIG"
# 缩短窗口加快验证: pre 2s, post 3s
sed -i 's/event_collision_pre_time=5/event_collision_pre_time=2/' "$CONFIG"
sed -i 's/event_collision_post_time=10/event_collision_post_time=3/' "$CONFIG"

echo "=== [1] 启动 mock 接收端与节点 ==="
(cd "$WORK" && nohup python3 "$ROOT/tools/mock_server.py" 8080 "$INBOX" \
    > "$WORK/mock.log" 2>&1 &)
(cd "$WORK" && nohup ros2 run datacache lidar_sim_node \
    --ros-args -p pcd_path:="$ROOT/install/datacache/share/datacache/pcd/sample.pcd" \
    > "$WORK/lidar.log" 2>&1 &)
(cd "$WORK" && nohup ros2 run datacache datacache_node \
    --ros-args -p config_path:="$CONFIG" > "$WORK/node.log" 2>&1 &)
sleep 4

curl -s http://127.0.0.1:8080/health && echo " <- mock 服务健康"
grep -c "LidarSim\|publish" "$WORK/lidar.log" > /dev/null && echo "雷达仿真已启动"
grep "initialized" "$WORK/node.log" | tail -1

echo "=== [2] 触发 collision 事件 ==="
ros2 service call /trigger_event datacache/srv/EventTrigger \
    "{event_name: 'collision'}"

echo "=== [3] 等待 post 窗口 + 落盘 + 回传 (10s) ==="
sleep 10

EVENT_DIR=$(ls -d "$WORK"/records/collision_* 2>/dev/null | head -1)
if [ -z "$EVENT_DIR" ]; then
    echo "!! 未找到事件目录"; tail -20 "$WORK/node.log"; exit 1
fi
echo "事件目录: $EVENT_DIR"
ls -la "$EVENT_DIR"

echo "=== [4] record_reader 概要与校验 ==="
ros2 run datacache record_reader "$EVENT_DIR"
ros2 run datacache record_reader "$EVENT_DIR" --verify
echo "verify 退出码: $? (0=通过)"
ros2 run datacache record_reader "$EVENT_DIR" --export "$EXPORT" --limit 3
echo "导出内容:"; ls "$EXPORT" 2>/dev/null

echo "=== [5] 上传闭环 ==="
if [ -f "$EVENT_DIR/.uploaded" ]; then
    echo ".uploaded 标记内容:"; cat "$EVENT_DIR/.uploaded"
else
    echo "!! 无 .uploaded 标记"; tail -20 "$WORK/node.log"
fi
echo "mock 接收端落盘:"
find "$INBOX" -type f | head -20
echo "mock 日志尾部:"
tail -12 "$WORK/mock.log"

echo "=== [6] 收尾 ==="
pkill -f mock_server.py 2>/dev/null
pkill -f datacache_node 2>/dev/null
pkill -f lidar_sim_node 2>/dev/null
echo "冒烟验证完成"
