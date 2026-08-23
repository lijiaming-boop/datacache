#!/usr/bin/env bash
# datacache 集成测试: 端到端跑通完整链路
#
# 流程:
#   1. 启动 lidar_sim_node(10Hz 点云) + 合成相机(30fps) + datacache_node + event_trigger_node
#   2. 等待 /trigger_event 服务可用, 预热 7 秒积累事件前缓冲区数据
#   3. 通过 /request_trigger 依次触发 collision 与 hard_brake 事件
#   4. 等待事件后窗口结束(collision post=10s)并留出落盘时间
#   5. 停止节点, 用 validate_records.py 校验录制产物
#
# 用法: bash test/run_integration_test.sh
# 注意: ROS 的 setup.bash 内部使用了未定义变量, 必须在 set -u 之前 source
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$REPO_DIR/test/logs"
RECORDS_DIR="$REPO_DIR/records"

source /opt/ros/jazzy/setup.bash
source "$REPO_DIR/install/setup.bash"
set -u

cd "$REPO_DIR"

rm -rf "$RECORDS_DIR" "$LOG_DIR"
mkdir -p "$LOG_DIR"

PIDS=()

cleanup() {
    for pid in "${PIDS[@]:-}"; do
        kill "$pid" 2>/dev/null
    done
    wait 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "=== [1/5] 启动节点 ==="
ros2 run datacache lidar_sim_node \
    --ros-args -p pcd_path:="$REPO_DIR/pcd/sample.pcd" -p hz:=10 \
    > "$LOG_DIR/lidar_sim.log" 2>&1 &
PIDS+=($!)

python3 "$REPO_DIR/test/test_camera_publisher.py" --fps 30 \
    > "$LOG_DIR/camera_pub.log" 2>&1 &
PIDS+=($!)

ros2 run datacache datacache_node \
    --ros-args -p config_path:="$REPO_DIR/config.txt" \
    > "$LOG_DIR/datacache.log" 2>&1 &
PIDS+=($!)

ros2 run datacache event_trigger_node \
    --ros-args -p interval:=0 \
    > "$LOG_DIR/event_trigger.log" 2>&1 &
PIDS+=($!)

echo "=== [2/5] 等待 /trigger_event 服务与传感器数据流就绪 ==="
ready=0
for _ in $(seq 1 30); do
    if ros2 service list 2>/dev/null | grep -q '/trigger_event' \
        && ros2 topic list 2>/dev/null | grep -q '/image_raw' \
        && ros2 topic list 2>/dev/null | grep -q '/point_cloud'; then
        ready=1
        break
    fi
    sleep 1
done
if [ "$ready" -ne 1 ]; then
    echo "FAIL: 节点未在 30 秒内就绪, 查看日志: $LOG_DIR"
    exit 1
fi

# 打印约 3 秒的话题频率, 确认数据流正常
ros2 topic hz /image_raw --window 30 > "$LOG_DIR/hz_image.txt" 2>&1 &
HZ_PID=$!
sleep 3
kill $HZ_PID 2>/dev/null
echo "--- /image_raw 频率 ---"
grep 'average rate' "$LOG_DIR/hz_image.txt" | tail -1 || echo "(无频率数据)"

# 预热: 积累事件前(collision pre=5s)的缓冲数据
sleep 7

echo "=== [3/5] 触发 collision 与 hard_brake 事件 ==="
ros2 service call /request_trigger datacache/srv/EventTrigger "{event_name: 'collision'}"
ros2 service call /request_trigger datacache/srv/EventTrigger "{event_name: 'hard_brake'}"

echo "=== [4/5] 等待事件后窗口结束并落盘 (约 16s) ==="
# collision post=10s, hard_brake post=6s; 额外 6s 供后台存储线程写盘
sleep 16

echo "--- datacache_node 关键日志 ---"
grep -E "Event|Recording|triggered" "$LOG_DIR/datacache.log" | tail -8 || true

echo "=== [5/5] 停止节点并校验录制产物 ==="
cleanup
trap - EXIT INT TERM
sleep 1

python3 "$REPO_DIR/test/validate_records.py"
result=$?

echo
echo "===== 录制产物概览 ====="
find "$RECORDS_DIR" -maxdepth 1 -type d | sort | while read -r dir; do
    [ "$dir" = "$RECORDS_DIR" ] && continue
    echo "$(basename "$dir"): $(find "$dir" -type f | wc -l) 个文件"
done

if [ "$result" -eq 0 ]; then
    echo
    echo "INTEGRATION TEST PASSED"
else
    echo
    echo "INTEGRATION TEST FAILED"
fi
exit "$result"
