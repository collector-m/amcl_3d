#pragma once
#include <climits>
#include <random>

class XorShift128
{
  private:
    unsigned int x_;
    unsigned int y_;
    unsigned int z_;
    unsigned int w_;
    unsigned int random();

  public:
    static constexpr unsigned int min() { return 0u; }
    static constexpr unsigned int max() { return UINT_MAX; }
    unsigned int operator()();
    XorShift128();
};
