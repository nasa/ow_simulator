// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <iterator>
#include <algorithm>
#include <limits>

#include <MovingMaxFilter.h>

MovingMaxFilter::MovingMaxFilter(size_t window_width) : m_width(window_width)
{
  clear();
}

void MovingMaxFilter::addDatum(double datum)
{
  // shift container by removing from the beginning and adding to the end
  double popped = m_data.front();
  m_data.pop_front();
  m_data.push_back(datum);
  // update max value if needed
  if (datum > m_max) // datum replaces max if it's greater than
    m_max = datum;
  else if (popped == m_max) // find new max if popped value was the max
    m_max = *std::max_element(std::begin(m_data), std::end(m_data));
}

void MovingMaxFilter::clear()
{
  static constexpr auto fill = -std::numeric_limits<double>::infinity();
  m_data = std::deque<double>(m_width, fill);
  m_max = fill;
}
