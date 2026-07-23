#include "src/physics/LambertSolver.h"
#include "src/physics/Integrators.h"
#include "src/mission/DeepSpaceMissionPlanner.h"
#include "src/environment/Planet.h"
#include "src/core/Constants.h"
#include <cstdio>
using namespace DeepSpace;
int main(){
    Planet earth("Earth", 5.9722e24, 6.371e6, Atmosphere(101325.0,8500.0));
    Planet moon("Moon", 7.342e22, 1.7371e6, Atmosphere(0.0,0.0));
    DeepSpaceMissionPlanner planner(earth, moon);

    auto tli = planner.PlanTransLunarInjection(6.771e6, 3.844e8);
    std::printf("TLI dv=%.1f m/s ok=%d next=%d\n", tli.deltaV, (int)tli.success, (int)tli.nextPhase);
    auto lox = planner.PlanLunarOrbitInsertion(Vec3d(1000,0,0), Vec3d(200,0,0));
    std::printf("LOI dv=%.1f ok=%d\n", lox.deltaV, (int)lox.success);

    // 真实闭环：用合理的遥测序列推进状态机
    MissionPhase p = planner.GetInitialPhase();
    const char* names[] = {"PreLaunch","EarthOrbit","TLI","CoastToMoon","LOI","Landing","Completed","Failed"};
    // 序列: 升空->入轨(高度ok,低速) -> TLI(速度到10km/s) -> 滑行(高度增大超过月球影响球)
    //       -> 近月(高度降到环月半径) -> 制动(速度降到2km/s) -> 着陆(高度<1,速度<5)
    struct T { double alt; double spd; } seq[] = {
        {2.0e5, 50.0},    // PreLaunch -> EarthOrbit
        {2.0e5, 1.2e4},   // EarthOrbit -> TLI
        {7.0e7, 1.0e3},   // TLI -> CoastToMoon
        {1.9e6, 1.5e3},   // CoastToMoon -> LOI (高度降到环月半径内)
        {1.9e6, 1.5e3},   // LOI -> Landing (速度降到<=2km/s)
        {0.5,   3.0},     // Landing -> Completed
    };
    int guard=0;
    std::printf("phase trace: %s", names[(int)p]);
    while(p!=MissionPhase::Completed && p!=MissionPhase::Failed && guard++<10){
        auto t = seq[guard-1 < 6 ? guard-1 : 5];
        auto r = planner.AdvancePhase(p, t.alt, t.spd);
        p = r.nextPhase;
        std::printf(" -> %s", names[(int)p]);
    }
    std::printf("\nfinal phase=%d (Completed=6)\n", (int)p);
    return (p==MissionPhase::Completed)?0:1;
}
