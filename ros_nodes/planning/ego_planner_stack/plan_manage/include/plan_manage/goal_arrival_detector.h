#ifndef EGO_PLANNER_GOAL_ARRIVAL_DETECTOR_H
#define EGO_PLANNER_GOAL_ARRIVAL_DETECTOR_H

#include <cmath>

namespace ego_planner
{

  class GoalArrivalDetector
  {
  public:
    void configure(double radius, double distance_increase_threshold)
    {
      radius_ = radius;
      distance_increase_threshold_ = distance_increase_threshold;
      reset();
    }

    void reset()
    {
      have_closest_distance_ = false;
      closest_distance_ = 0.0;
      reached_ = false;
    }

    bool update(double distance)
    {
      if (reached_ || !std::isfinite(distance))
        return reached_;

      if (distance <= radius_ && (!have_closest_distance_ || distance < closest_distance_))
      {
        closest_distance_ = distance;
        have_closest_distance_ = true;
      }

      reached_ = have_closest_distance_ &&
                 distance - closest_distance_ >= distance_increase_threshold_;
      return reached_;
    }

    bool reached() const
    {
      return reached_;
    }

    double closestDistance() const
    {
      return closest_distance_;
    }

  private:
    double radius_{0.5};
    double distance_increase_threshold_{0.2};
    bool have_closest_distance_{false};
    double closest_distance_{0.0};
    bool reached_{false};
  };

} // namespace ego_planner

#endif
