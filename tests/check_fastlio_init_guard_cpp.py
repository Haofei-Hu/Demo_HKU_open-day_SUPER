"""Compile actual patched startup control flow with lightweight dependency stubs.

Usage: python3 tests/check_fastlio_init_guard_cpp.py /path/to/original/IMU_Processing.hpp
This exercises construction/reset/Process branching, not the real ESKF or deskew.
The upstream header is supplied explicitly rather than vendored into this repo.
"""
from pathlib import Path
import subprocess
import sys
import tempfile
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/maintenance'))
from apply_fastlio_init_guard import render
scratch=tempfile.TemporaryDirectory(prefix='fastlio_init_test_')
root=Path(scratch.name)
s=render(Path(sys.argv[1]).read_bytes()).decode()
def function(sig):
 a=s.index(sig);b=s.index('{',a);n=1;i=b+1
 while n:
  if s[i]=='{':n+=1
  elif s[i]=='}':n-=1
  i+=1
 return s[a:i]
cls=s[s.index('class ImuProcess'):s.index('ImuProcess::ImuProcess()')].replace('private:','public:')
stub=r'''
#include <cassert>
#include <cmath>
#include <deque>
#include <vector>
#include <memory>
#include <fstream>
#include <iostream>
using namespace std;
struct V3D {double x,y,z; V3D()=default; V3D(double a,double b,double c):x(a),y(b),z(c){} double norm()const{return sqrt(x*x+y*y+z*z);} V3D& operator*=(double v){x*=v;y*=v;z*=v;return *this;}};
struct M3D{}; const M3D Eye3d{};const V3D Zero3d(0,0,0);struct Pose6D{};
namespace Eigen{template<class T,int A,int B>using Matrix=M3D;}
#define MD(A,B) M3D
#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#define MAX_INI_COUNT 10
#define DEBUG_FILE_DIR(X) "/dev/null"
const double G_m_s2=9.81;
double omp_get_wtime(){return 0;}
M3D process_noise_cov(){return {};}
namespace sensor_msgs {namespace msg {struct Imu{using ConstSharedPtr=shared_ptr<const Imu>;};}}
struct PointCloudXYZI{using Ptr=shared_ptr<PointCloudXYZI>;vector<int> points;void clear(){points.clear();}};
struct MeasureGroup{deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu;PointCloudXYZI::Ptr lidar;double lidar_end_time=100.02;};
struct state_ikfom{};struct input_ikfom{};
namespace esekfom{template<class T,int N,class U>struct esekf{T get_x(){return {};}};}
'''
hooks=r'''
void ImuProcess::IMU_init(const MeasureGroup&,esekfom::esekf<state_ikfom,12,input_ikfom>&,int& n){n=11;mean_acc=V3D(0,0,1);}
void ImuProcess::UndistortPcl(const MeasureGroup&,esekfom::esekf<state_ikfom,12,input_ikfom>&,PointCloudXYZI& out){out.points.push_back(42);}
int main(){
 ImuProcess p;assert(p.last_lidar_end_time_==-1.0);assert(p.acc_s_last.norm()==0);
 auto cloud=make_shared<PointCloudXYZI>();cloud->points.push_back(123);
 MeasureGroup meas;meas.lidar=make_shared<PointCloudXYZI>();esekfom::esekf<state_ikfom,12,input_ikfom> kf;
 p.Process(meas,kf,cloud);assert(cloud->points.empty());assert(p.imu_need_init_);
 meas.imu.push_back(make_shared<sensor_msgs::msg::Imu>());cloud->points.push_back(321);
 p.Process(meas,kf,cloud);assert(!p.imu_need_init_);assert(p.last_lidar_end_time_==100.02);assert(cloud->points.empty());assert(p.acc_s_last.norm()==0);
 p.Process(meas,kf,cloud);assert(cloud->points==vector<int>{42});
 meas.imu.clear();p.Process(meas,kf,cloud);assert(cloud->points.empty());
 p.acc_s_last=V3D(1,2,3);p.last_lidar_end_time_=999;p.Reset();assert(p.acc_s_last.norm()==0);assert(p.last_lidar_end_time_==-1);assert(p.imu_need_init_);
 cout<<"constructor/reset/init-boundary/valid-scan/empty-IMU: PASS"<<endl;
}
'''
code=stub+cls+'\n'.join(function(sig) for sig in ['ImuProcess::ImuProcess()','ImuProcess::~ImuProcess()','void ImuProcess::Reset()','void ImuProcess::Process('])+hooks
(root/'control_flow.cpp').write_text(code)
subprocess.run(['g++','-std=c++14','-Wall','-Wextra','-fsanitize=address,undefined','-g',str(root/'control_flow.cpp'),'-o',str(root/'control_flow')],check=True)
subprocess.run([str(root/'control_flow')],check=True)

scratch.cleanup()
