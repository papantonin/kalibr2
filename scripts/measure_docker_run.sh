#!/usr/bin/env bash
set -u

if [ "$#" -lt 5 ] || [ "$4" != "--" ]; then
  echo "usage: $0 CONTAINER_NAME METRICS_JSON LOG_FILE -- docker run ..." >&2
  exit 2
fi

container_name=$1
metrics_file=$2
log_file=$3
shift 4

mkdir -p "$(dirname "$metrics_file")" "$(dirname "$log_file")"
start_ns=$(date +%s%N)
"$@" >"$log_file" 2>&1 &
runner_pid=$!

container_pid=""
for _ in $(seq 1 200); do
  container_pid=$(docker inspect -f '{{.State.Pid}}' "$container_name" 2>/dev/null || true)
  if [ -n "$container_pid" ] && [ "$container_pid" != "0" ]; then
    break
  fi
  sleep 0.05
done

peak_bytes=0
peak_anon_bytes=0
peak_file_bytes=0
peak_kernel_bytes=0
cpu_usage_usec=0
samples=0
cgroup_dir=""
if [ -n "$container_pid" ] && [ "$container_pid" != "0" ]; then
  cgroup_relative=$(awk -F: '$1 == "0" { print $3 }' "/proc/$container_pid/cgroup")
  cgroup_dir="/sys/fs/cgroup${cgroup_relative}"
fi

while kill -0 "$runner_pid" 2>/dev/null; do
  if [ -n "$cgroup_dir" ] && [ -r "$cgroup_dir/memory.peak" ]; then
    measured_peak=$(<"$cgroup_dir/memory.peak")
    if [ "$measured_peak" -gt "$peak_bytes" ]; then
      peak_bytes=$measured_peak
    fi
    measured_anon=$(awk '$1 == "anon" { print $2 }' "$cgroup_dir/memory.stat")
    measured_file=$(awk '$1 == "file" { print $2 }' "$cgroup_dir/memory.stat")
    measured_kernel=$(awk '$1 == "kernel" { print $2 }' "$cgroup_dir/memory.stat")
    if [ -n "$measured_anon" ] && [ "$measured_anon" -gt "$peak_anon_bytes" ]; then
      peak_anon_bytes=$measured_anon
    fi
    if [ -n "$measured_file" ] && [ "$measured_file" -gt "$peak_file_bytes" ]; then
      peak_file_bytes=$measured_file
    fi
    if [ -n "$measured_kernel" ] && [ "$measured_kernel" -gt "$peak_kernel_bytes" ]; then
      peak_kernel_bytes=$measured_kernel
    fi
    measured_cpu=$(awk '$1 == "usage_usec" { print $2 }' "$cgroup_dir/cpu.stat")
    if [ -n "$measured_cpu" ]; then
      cpu_usage_usec=$measured_cpu
    fi
    samples=$((samples + 1))
  fi
  sleep 0.1
done

wait "$runner_pid"
exit_code=$?
end_ns=$(date +%s%N)
wall_ns=$((end_ns - start_ns))

awk -v wall_ns="$wall_ns" -v peak="$peak_bytes" -v peak_anon="$peak_anon_bytes" \
    -v peak_file="$peak_file_bytes" -v peak_kernel="$peak_kernel_bytes" -v cpu_us="$cpu_usage_usec" \
    -v samples="$samples" -v status="$exit_code" -v name="$container_name" \
  'BEGIN {
    printf "{\n"
    printf "  \"container\": \"%s\",\n", name
    printf "  \"exit_code\": %d,\n", status
    printf "  \"wall_seconds\": %.6f,\n", wall_ns / 1000000000
    printf "  \"cpu_seconds\": %.6f,\n", cpu_us / 1000000
    printf "  \"peak_memory_bytes\": %d,\n", peak
    printf "  \"peak_memory_mib\": %.3f,\n", peak / 1048576
    printf "  \"peak_anon_mib\": %.3f,\n", peak_anon / 1048576
    printf "  \"peak_file_cache_mib\": %.3f,\n", peak_file / 1048576
    printf "  \"peak_kernel_mib\": %.3f,\n", peak_kernel / 1048576
    printf "  \"samples\": %d\n", samples
    printf "}\n"
  }' >"$metrics_file"

exit "$exit_code"
