#include <QCoreApplication>
#include <benchmark/benchmark.h>

#include "libflock_gpu.h"

#include "Flock.h"



static void CPU_Flock_20( benchmark::State& state )
{
    for( auto _ : state)
        benchmark::DoNotOptimize( Flock(20) );
}
BENCHMARK(CPU_Flock_20);

static void GPU_Flock_20( benchmark::State& state )
{
    for( auto _ : state)
        benchmark::DoNotOptimize( libFlock(20) );
}
BENCHMARK(GPU_Flock_20);

static void CPU_Flock_100( benchmark::State& state )
{
    for( auto _ : state)
        benchmark::DoNotOptimize( Flock(100) );
}
BENCHMARK(CPU_Flock_100);

static void GPU_Flock_100( benchmark::State& state )
{
    for( auto _ : state)
        benchmark::DoNotOptimize( libFlock(100) );
}
BENCHMARK(GPU_Flock_100);

static void CPU_Flock_Update_20( benchmark::State& state )
{


    Flock flock_cpu(20);

    for ( auto _ : state )
    {
      flock_cpu.update();
    }
}

BENCHMARK(CPU_Flock_Update_20);

static void GPU_Flock_Update_20( benchmark::State& state )
{
    libFlock flock_gpu(20);

    for ( auto _ : state )
    {
      flock_gpu.updateFlock();
    }
}
BENCHMARK(GPU_Flock_Update_20);

static void CPU_Flock_Update_100( benchmark::State& state )
{


    Flock flock_cpu(100);

    for ( auto _ : state )
    {
      flock_cpu.update();
    }
}

BENCHMARK(CPU_Flock_Update_100);

static void GPU_Flock_Update_100( benchmark::State& state )
{
    libFlock flock_gpu(100);

    for ( auto _ : state )
    {
      flock_gpu.updateFlock();
    }
}
BENCHMARK(GPU_Flock_Update_100);

static void CPU_Flock_Update_200( benchmark::State& state )
{


    Flock flock_cpu(200);

    for ( auto _ : state )
    {
      flock_cpu.update();
    }
}

BENCHMARK(CPU_Flock_Update_200);

static void GPU_Flock_Update_200( benchmark::State& state )
{
    libFlock flock_gpu(200);

    for ( auto _ : state )
    {
      flock_gpu.updateFlock();
    }
}
BENCHMARK(GPU_Flock_Update_200);





BENCHMARK_MAIN();
