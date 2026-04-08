#define _USE_MATH_DEFINES
#include <cmath>
#include "sbf/robot/robot.h"
#include "sbf/robot/interval_math.h"
#include "sbf/envelope/endpoint_source.h"
#include "sbf/envelope/crit_sample.h"
#include "sbf/envelope/gcpc.h"
#include "sbf/robot/fk.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <random>
#include <vector>
using namespace sbf;
using namespace sbf::envelope;

int main() {
    std::vector<DHParam> dh = {
        {0,0,0.1575,0,0},{-HALF_PI,0,0,0,0},{HALF_PI,0,0.2025,0,0},
        {HALF_PI,0,0,0,0},{-HALF_PI,0,0.2155,0,0},{-HALF_PI,0,0,0,0},{HALF_PI,0,0.081,0,0}
    };
    JointLimits lim; lim.limits={{-1.865,1.866},{-0.1,1.087},{-0.663,0.662},{-2.094,-0.372},{-0.619,0.62},{-1.095,1.258},{1.05,2.091}};
    DHParam tool{0,0,0.185,0,0};
    std::vector<double> radii={0.08,0.08,0.06,0.06,0.04,0.04,0.04,0.03};
    Robot robot("iiwa14",dh,lim,tool,radii);
    int n_act = robot.n_active_links();
    
    EndpointSourceConfig cfg_ifk = EndpointSourceConfig::ifk();
    EndpointSourceConfig cfg_crit; cfg_crit.method = EndpointSource::CritSample;
    
    std::mt19937 rng(42);
    double widths[] = {0.05,0.1,0.15,0.2,0.3,0.4,0.5,0.7,1.0};
    
    // Warmup
    for (int i = 0; i < 5; i++) {
        std::vector<Interval> iv(7);
        for (int j=0;j<7;j++) { iv[j].lo=lim.limits[j].lo; iv[j].hi=lim.limits[j].lo+0.1*(lim.limits[j].hi-lim.limits[j].lo); }
        auto r1 = compute_endpoint_iaabb(cfg_ifk, robot, iv);
        auto r2 = compute_endpoint_iaabb(cfg_crit, robot, iv);
    }
    
    printf("source,trial,width,time_ms\n");
    for (int t=0; t<100; t++) {
        double frac = widths[t % 9];
        std::vector<Interval> iv(7);
        for (int j=0;j<7;j++) {
            double lo=lim.limits[j].lo, hi=lim.limits[j].hi, range=hi-lo;
            std::uniform_real_distribution<double> d(lo,hi);
            double c=d(rng), hw=range*frac*0.5;
            iv[j].lo=std::max(lo,c-hw); iv[j].hi=std::min(hi,c+hw);
        }
        for (auto& [name,cfg] : std::vector<std::pair<const char*,EndpointSourceConfig*>>{{"IFK",&cfg_ifk},{"CritSample",&cfg_crit}}) {
            compute_endpoint_iaabb(*cfg, robot, iv); // warmup
            double times[3];
            for (int r=0;r<3;r++) {
                auto t0=std::chrono::high_resolution_clock::now();
                compute_endpoint_iaabb(*cfg, robot, iv);
                auto t1=std::chrono::high_resolution_clock::now();
                times[r]=std::chrono::duration<double,std::milli>(t1-t0).count();
            }
            std::sort(times,times+3);
            printf("%s,%d,%.3f,%.6f\n",name,t,frac,times[1]);
        }
    }
}
