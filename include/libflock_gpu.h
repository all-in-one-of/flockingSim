#ifndef LIBFLOCK_GPU_H
#define LIBFLOCK_GPU_H


class Flock_GPU;

class libFlock
{
public:

  libFlock(int _numBoids);

  void updateFlock();

  void dumpGeo(int _numFrame);

  void flock_gpu(int _numBoids);

  Flock_GPU * m_flock;

};


#endif //LIBFLOCK_GPU
