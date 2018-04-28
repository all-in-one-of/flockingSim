#include <QCoreApplication>
#include <benchmark/benchmark.h>

#include "flockSim_gpu.h"

#include "Flock.h"



//static void CPU_Flock( benchmark::State& state )
//{
//    for( auto _ : state)
//        benchmark::DoNotOptimize( Flock() );
//}
//BENCHMARK(CPU_Flock);
