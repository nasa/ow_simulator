// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef MOVING_MAX_FILTER
#define MOVING_MAX_FILTER

#include <deque>

class MovingMaxFilter
{
public:
  MovingMaxFilter(size_t window_width);

  MovingMaxFilter(const MovingMaxFilter&) = default;
  MovingMaxFilter& operator=(const MovingMaxFilter&) = default;
  ~MovingMaxFilter() = default;

  MovingMaxFilter() = delete;

  void addDatum(double datum);

  double evaluate() const {
    return m_max;
  };

  void clear();

private:
  size_t const m_width;
  double m_max;
  std::deque<double> m_data;
};

#endif // MOVING_MAX_FILTER
