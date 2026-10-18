"""Keep real vision sampling active until concurrent preparation is complete."""
import contextlib
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/analysis'))
from control_chain_age_check import monitor


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='195' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise SystemExit('Test requires private localhost-only ROS domain 195')
    context=Context();rclpy.init(context=context)
    node=rclpy.create_node('parallel_vision_fixture',context=context)
    types={'/Odometry':Odometry,'/super_debug/aligned_pose_cov':PoseWithCovarianceStamped,
           '/mavros/vision_pose/pose_cov':PoseWithCovarianceStamped,'/mavros/local_position/pose':PoseStamped}
    publishers={topic:node.create_publisher(kind,topic,10) for topic,kind in types.items()}
    settings=dict(bad_frame=False,drop=False)
    def publish():
        for topic,kind in types.items():
            if topic=='/mavros/vision_pose/pose_cov' and settings['drop']:continue
            msg=kind();msg.header.stamp=node.get_clock().now().to_msg()
            msg.header.frame_id='camera_init' if topic=='/Odometry' else ('map' if topic=='/mavros/local_position/pose' else 'px4_enu_preview')
            if topic=='/Odometry':msg.child_frame_id='body'
            if topic=='/mavros/vision_pose/pose_cov' and settings['bad_frame']:msg.header.frame_id='wrong'
            publishers[topic].publish(msg)
    timer=node.create_timer(.01,publish)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    thread=threading.Thread(target=executor.spin);thread.start();results=[]
    try:
        for case in ('healthy','late_integrity_fault','late_temporal_gap'):
            settings.update(bad_frame=False,drop=False)
            with tempfile.TemporaryDirectory() as directory:
                ready=Path(directory)/'ready'
                def complete_preparation():
                    if case=='healthy':time.sleep(3.)
                    elif case=='late_integrity_fault':
                        time.sleep(2.);settings['bad_frame']=True;time.sleep(1.)
                    else:
                        time.sleep(1.7);settings['drop']=True;time.sleep(.5);settings['drop']=False
                    ready.touch()
                preparation=threading.Thread(target=complete_preparation);preparation.start()
                output=io.StringIO();began=time.monotonic()
                with contextlib.redirect_stdout(output):code=monitor(1.,vision_chain=True,wait_stable=True,ready_file=ready)
                duration=time.monotonic()-began;preparation.join()
                if case=='late_integrity_fault':
                    assert code==2 and 'unexpected frames' in output.getvalue(),output.getvalue()
                else:
                    assert code==0 and duration>=2.9,(duration,output.getvalue())
                    if case=='late_temporal_gap':assert 'observation window interrupted' in output.getvalue(),output.getvalue()
                results.append(dict(case=case,code=code,duration_s=duration))
        evidence=dict(result='PASS',scope='Real ROS with synthetic publishers; one-second test windows only. Production startup retains 30 seconds.',cases=results)
        if len(sys.argv)>1:Path(sys.argv[1]).write_text(json.dumps(evidence,indent=2)+'\n')
        print(json.dumps(evidence,indent=2),flush=True)
    finally:
        executor.shutdown();thread.join();node.destroy_node();rclpy.shutdown(context=context)


if __name__=='__main__':main()
