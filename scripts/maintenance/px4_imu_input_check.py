#!/usr/bin/env python3
"""Read-only PX4 raw IMU observation; no stream-rate or flight parameter writes.

Reports transport and original header intervals separately. A successful input
check is not an extrinsic/time calibration or a flight-readiness assessment.
"""
import argparse
import json
import math
from pathlib import Path
import time

TOPIC='/mavros/imu/data_raw'


class Observation:
    def __init__(self):
        self.samples=[]
        self.invalid=0
        self.nonincreasing=0
        self.last_stamp=None

    def add(self,stamp_ns,receive_s,ros_ns,frame,acc,gyro):
        if stamp_ns<=0 or not all(math.isfinite(v) for v in (*acc,*gyro)):
            self.invalid+=1
            return
        if self.last_stamp is not None and stamp_ns<=self.last_stamp:self.nonincreasing+=1
        self.last_stamp=stamp_ns
        self.samples.append((stamp_ns,receive_s,(ros_ns-stamp_ns)/1e9,frame,
                             math.sqrt(sum(v*v for v in acc))))

    def report(self,publishers):
        def stats(values):
            return dict(min=min(values),mean=sum(values)/len(values),max=max(values)) if values else None
        n=len(self.samples)
        stamp_gaps=[(b[0]-a[0])/1e9 for a,b in zip(self.samples,self.samples[1:])]
        receive_gaps=[b[1]-a[1] for a,b in zip(self.samples,self.samples[1:])]
        frames=sorted({s[3] for s in self.samples})
        ok=n>=2 and not self.invalid and not self.nonincreasing and len(publishers)==1 and frames==['base_link']
        return dict(schema_version=1,topic=TOPIC,result='SUCCESS' if ok else 'FAIL',
                    scope='input presence, finite vectors, original stamps and body FLU frame only',
                    samples=n,invalid_samples=self.invalid,nonincreasing_stamps=self.nonincreasing,
                    frames=frames,publisher_identities=sorted(publishers),
                    original_stamp_gap_s=stats(stamp_gaps),receive_gap_s=stats(receive_gaps),
                    header_age_s=stats([s[2] for s in self.samples]),
                    acceleration_norm_mps2=stats([s[4] for s in self.samples]),
                    original_stamp_rate_hz=((n-1)/((self.samples[-1][0]-self.samples[0][0])/1e9)
                                            if n>1 and self.samples[-1][0]>self.samples[0][0] else None),
                    delay_calibrated=False,flight_readiness_assessed=False,
                    rate_or_gap_acceptance_limit=None,parameters_written=False)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=float,default=10.)
    p.add_argument('--report',type=Path,required=True)
    args=p.parse_args()
    if not math.isfinite(args.seconds) or args.seconds<=0:p.error('seconds must be finite and positive')
    if args.report.exists():p.error('use a new report path')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    rclpy.init();node=rclpy.create_node('px4_imu_input_observer');observation=Observation();publishers=set()
    def receive(m):
        s=m.header.stamp;a=m.linear_acceleration;g=m.angular_velocity
        observation.add(s.sec*10**9+s.nanosec,time.monotonic(),node.get_clock().now().nanoseconds,
                        m.header.frame_id,(a.x,a.y,a.z),(g.x,g.y,g.z))
    sub=node.create_subscription(Imu,TOPIC,receive,qos_profile_sensor_data)
    print(f'PX4 raw IMU observation: {args.seconds:g} s; no parameter writes.',flush=True)
    end=time.monotonic()+args.seconds
    try:
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
            publishers.update(bytes(i.endpoint_gid).hex() for i in node.get_publishers_info_by_topic(TOPIC))
        report=observation.report(publishers)
        with args.report.open('x') as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
        print(json.dumps(report,indent=2));print('REPORT:',args.report)
        return 0 if report['result']=='SUCCESS' else 1
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
