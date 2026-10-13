#!/usr/bin/env python3
"""Standalone read-only clock check. Owns one MAVROS, never starts mission nodes.

Run after sourcing /opt/ros/humble/setup.bash. Refuses an occupied flight lock
or serial device; stops only its own process group on completion/interruption.
"""
import argparse
import fcntl
import os
from pathlib import Path
import signal
import statistics
import sys
import subprocess
import tempfile
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from mavros_msgs.msg import TimesyncStatus, State
from mavros_msgs.srv import MessageInterval
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from deployment_config import WORKSPACE, parse_serial_url, serial_lock_path


def main():
    parser = argparse.ArgumentParser(description="只读检查飞控时钟与消息时延；不启动飞行任务。")
    parser.add_argument("--fcu-url", default=os.environ.get("CUADC_FCU_URL", ""), help="必须显式提供的飞控串口 URL")
    parser.add_argument("--callbacks", action="store_true", help="同时运行 V16 C++ 时钟回调诊断")
    parser.add_argument("--callback-executable", type=Path, default=Path(os.environ.get("CUADC_BUILD_BASE", str(WORKSPACE / "build"))) / "cuadc_full_mission_20260803" / "test_time_alignment", help="编译后的回调诊断程序路径")
    args = parser.parse_args()
    try:
        device, _ = parse_serial_url(args.fcu_url)
    except ValueError as error:
        parser.error(str(error))
    if args.callbacks and not args.callback_executable.is_file():
        parser.error(f"回调诊断程序不存在：{args.callback_executable}；请启用 BUILD_TESTING 编译")
    with serial_lock_path().open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if subprocess.run(['pgrep','-x','mavros_node'], stdout=subprocess.DEVNULL).returncode == 0:
            raise RuntimeError('MAVROS already running; refusing a second instance')
        if subprocess.run(['fuser',os.path.realpath(device)], stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL).returncode == 0:
            raise RuntimeError('FCU serial device is occupied')
        logdir = Path(tempfile.mkdtemp(prefix='cuadc-clock-check-'))
        print('Diagnostic MAVROS log:',logdir,flush=True)
        with (logdir/'mavros.log').open('w') as log:
            proc = subprocess.Popen(['ros2','launch','cuadc_full_mission_20260803','mission_mavros.launch.py',
                'fcu_url:='+args.fcu_url,'namespace:=mavros'],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            node = None
            try:
                rclpy.init(); node=rclpy.create_node('cuadc_readonly_clock_check')
                records=[]; ages={'odom':[],'imu':[]}; state={'armed':False}; offset=[]
                def sync(msg):
                    records.append(msg)
                def sample(name,msg):
                    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
                    ages[name].append((time.monotonic(), time.time()-stamp))
                node.create_subscription(TimesyncStatus,'/mavros/timesync_status',sync,qos_profile_sensor_data)
                node.create_subscription(Odometry,'/mavros/local_position/odom',lambda m:sample('odom',m),qos_profile_sensor_data)
                node.create_subscription(Imu,'/mavros/imu/data',lambda m:sample('imu',m),qos_profile_sensor_data)
                node.create_subscription(State,'/mavros/state',lambda m:state.update(armed=m.armed),10)
                client=node.create_client(GetParameters,'/mavros/time/get_parameters')
                future=None; start=time.monotonic(); last=start
                stream=node.create_client(MessageInterval,'/mavros/set_message_interval')
                stream_futures=[]; monitor=None; monitor_log=None
                while time.monotonic()-start < 75:
                    if proc.poll() is not None: raise RuntimeError('MAVROS exited')
                    rclpy.spin_once(node,timeout_sec=0.1)
                    if state['armed']: raise RuntimeError('Aircraft is armed; ending diagnostic')
                    if not stream_futures and stream.service_is_ready():
                        # Telemetry-rate requests only, matching mission rates.
                        stream_futures=[stream.call_async(MessageInterval.Request(
                            message_id=i,message_rate=30.0)) for i in (30,32)]
                        if args.callbacks:
                            monitor_log=(logdir/'callbacks.log').open('w')
                            monitor=subprocess.Popen([
                                str(args.callback_executable),
                                '--clock-monitor','--ros-args','-p','measurement_time_sync:=true'],
                                stdout=monitor_log,stderr=subprocess.STDOUT)
                    if future is None and client.service_is_ready():
                        req=GetParameters.Request(names=['convergence_window','timesync_mode','timesync_rate','max_rtt_sample','max_deviation_sample'])
                        future=client.call_async(req)
                    if future is not None and future.done() and not offset:
                        offset.extend(future.result().values)
                        print('FCU_TIME_CONFIG',[(p.type,p.integer_value,p.double_value,p.string_value) for p in offset],flush=True)
                    if time.monotonic()-last >= 10:
                        last=time.monotonic()
                        if records:
                            m=records[-1]
                            print('elapsed',round(last-start),'sync_samples',len(records),'rtt_ms',m.round_trip_time_ms,
                                  'offset_ns',m.estimated_offset_ns,'residual_ms',(m.observed_offset_ns-m.estimated_offset_ns)/1e6,
                                  'odom',len(ages['odom']),'imu',len(ages['imu']),flush=True)
                recent=records[-200:]
                print('FINAL rtt_ms(min,median,max)',min(m.round_trip_time_ms for m in recent),
                      statistics.median(m.round_trip_time_ms for m in recent),max(m.round_trip_time_ms for m in recent))
                for name,rows in ages.items():
                    data=[age*1000 for stamp,age in rows if stamp>time.monotonic()-10]
                    print(name,'last_10s_samples',len(data),'age_ms(min,median,max)',
                          (min(data),statistics.median(data),max(data)) if data else None)
                if monitor is not None:
                    print('CALLBACK_TEST_EXIT',monitor.wait(timeout=5))
                    monitor_log.close()
                    print((logdir/'callbacks.log').read_text()[-1200:])
            finally:
                if 'monitor' in locals() and monitor is not None and monitor.poll() is None:
                    monitor.terminate(); monitor.wait(timeout=5)
                if node is not None: node.destroy_node()
                if rclpy.ok(): rclpy.try_shutdown()
                if proc.poll() is None:
                    os.killpg(proc.pid,signal.SIGINT)
                    try: proc.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid,signal.SIGTERM); proc.wait(timeout=5)


if __name__=='__main__': main()
