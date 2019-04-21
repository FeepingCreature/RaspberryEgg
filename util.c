#include "util.h"

unit unit_rebalance(unit unit)
{
  while (unit.substep >= 1.0)
  {
    unit.step += 1;
    unit.substep -= 1.0;
  }
  while (unit.substep < 0.0)
  {
    unit.step -= 1;
    unit.substep += 1.0;
  }
  return unit;
}

unit unit_add(unit unit, float f)
{
  unit.substep += f;
  return unit_rebalance(unit);
}

coordinate coord_advance(coordinate from, float egg, float pen, float servo)
{
  return (coordinate) {
    .egg = unit_add(from.egg, egg),
    .pen = unit_add(from.pen, pen),
    .servo = servo,
  };
}

float unitf(unit unit)
{
  return (float) unit.step + unit.substep;
}

float unit_diff_f(unit from, unit to)
{
  to.step -= from.step;
  to.substep -= from.substep;
  to = unit_rebalance(to);
  return (float) to.step + to.substep;
}
