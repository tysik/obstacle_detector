/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#pragma once

#include <vector>

#include "obstacle_detector/utilities/point.h"
#include "obstacle_detector/utilities/point_set.h"

namespace obstacle_detector
{

class Segment
{
public:
  Segment(const Point& p1 = Point(), const Point& p2 = Point()) {
    // Swap if not counter-clockwise
    if (p1.cross(p2) > 0.0)
      first_point = p1, last_point = p2;
    else
      first_point = p2, last_point = p1;
  }

  double length() const {
    return (last_point - first_point).length();
  }

  double lengthSquared() const {
    return (last_point - first_point).lengthSquared();
  }

  Point normal() const {
    return (last_point - first_point).perpendicular().normalized();
  }

  Point projection(const Point& p) const {
    Point a = last_point - first_point;
    Point b = p - first_point;
    return first_point + a.dot(b) * a / a.lengthSquared();
  }

  Point trueProjection(const Point& p) const {
    Point a = last_point - first_point;
    Point b = p - first_point;
    Point c = p - last_point;

    double t = a.dot(b) / a.lengthSquared();

    if (t < 0.0)
      return (first_point);
    else if (t > 1.0)
      return (last_point);
    else
      return first_point + a.dot(b) * a / a.lengthSquared();
  }

  double distanceTo(const Point& p) const {
    return (p - projection(p)).length();
  }

  double trueDistanceTo(const Point& p) const {
    Point a = last_point - first_point;
    Point b = p - first_point;
    Point c = p - last_point;

    double t = a.dot(b) / a.lengthSquared();

    if (t < 0.0)
      return b.length();
    else if (t > 1.0)
      return c.length();

    Point projection = first_point + t * a;
    return (p - projection).length();
  }


  friend std::ostream& operator<<(std::ostream& out, const Segment& s) {
    out << "p1: " << s.first_point << ", p2: " << s.last_point;
    return out;
  }

  Point first_point;
  Point last_point;
  std::vector<PointSet> point_sets;
};

} // namespace obstacle_detector
