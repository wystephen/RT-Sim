#include "ray.h"

int Ray::beacon_id() const { return beacon_id_; }

void Ray::setBeacon_id(int beacon_id) { beacon_id_ = beacon_id; }

double Ray::detect_intersection(const LineSeg &l, Point &intersect_point) {
  // A x = g represent the two line intersection condition.
  // elements of A.
  double a(cur_vec.x * -1.0);
  double b(l.ori_vec.x);
  double c(cur_vec.y * -1.0);
  double d(l.ori_vec.y);

  double Adet =
      a * d - b * c;  // det(A), if det(A) >0, the eqation is invertible.
  if (std::abs(Adet) > 0.0) {
    double Astar = 1.0 / Adet;

    // element of Inverse(A).
    double inv_a(Astar * d);
    double inv_b(Astar * -1.0 * b);
    double inv_c(Astar * -1.0 * c);
    double inv_d(Astar * a);

    // g = [g1 g2]^T
    double g1 = cur_point.x - l.start_point.x;
    double g2 = cur_point.y - l.start_point.y;

    // A\times x = g ==> x = Inverse(A) \times g.
    double ray_fac = inv_a * g1 + inv_b * g2;
    double line_fac = inv_c * g1 + inv_d * g2;

    // line fac >0 and < 1, because l is from start_point to start_point +
    // ori_vec. ray fac > 0 it's ray rather line.
    if (ray_fac >= 0.0 && line_fac >= 0.0 && line_fac <= 1.0) {
      // TODO:higher performance of copy, construct and move of Point and Line
      // is needed.
      Point p_ray(cur_point.x + cur_vec.x * ray_fac,
                  cur_point.y + cur_vec.y * ray_fac);

      // cur_point + cur_vec * ray_fac;
      Point p_line(l.start_point.x + l.ori_vec.x * line_fac,
                   l.start_point.y + l.ori_vec.y * line_fac);
      if (p_ray.isClose(
              p_line)) {  // recheck p_ray and p_line represent same point.
        intersect_point.x = p_ray.x;
        intersect_point.y = p_ray.y;
        return cur_vec.len() * ray_fac;
      } else {
        std::cout << "there are some error:"
                  << "p ray :" << p_ray.x << "," << p_ray.y
                  << "p line:" << p_line.x << "," << p_line.y << std::endl;
        return -1.0;
      }
    } else {
      return -1.0;
    }
  } else {
    return -1.0;
  }
  // Avoid some unexpected error when coding. as signal of error happend.
  std::cout << "l:" << l.start_point.x << "," << l.start_point.y
            << " l v:" << l.ori_vec.x << "," << l.ori_vec.y << std::endl;
  std::cout << "cur p:" << cur_point.x << "," << cur_point.y
            << " cur vec:" << cur_vec.x << "," << cur_vec.y << std::endl;
  std::cout << "Adet:" << Adet << std::endl;

  return -1.0;
}

bool Ray::reflection(Point p, Vector norm_vec) {
  if (cur_vec.cos(norm_vec) < 0.0) {
    norm_vec.x *= -1.0;
    norm_vec.y *= -1.0;
  }
  if (norm_vec.len() < EPS) {
    std::cout << "som error of reflection using error norm vector:"
              << norm_vec.x << "," << norm_vec.y << " which is illegal!"
              << std::endl;
    return false;
  }

  double idotn2 = (norm_vec.x * cur_vec.x + norm_vec.y * cur_vec.y);
  double x = cur_vec.x;
  double y = cur_vec.y;
  if (std::abs(idotn2) < EPS) {
    std::cout << "some error of idotn2:" << idotn2 << std::endl;
    std::cout << "ERROR:" << __FILE__ << ":" << __LINE__ << std::endl;
  }

  //  LineSeg l_ray(Point(cur_point.x, cur_point.y),
  //                Vector(p.x - cur_point.x, p.y - cur_point.y));
  //  line_list.push_back(l_ray);

  Vector reflec_vec(x - idotn2 * norm_vec.x * 2.0,
                    y - idotn2 * norm_vec.y * 2.0);

  updateRay(p, reflec_vec);

  return true;
}

bool Ray::reachedPoint(Point p, double max_dis) {
  if (max_dis < 0.0) {
    std::cout << "some error  max dis::" << max_dis << std::endl;
  }
  Vector pv_vec = Vector(p.x - cur_point.x, p.y - cur_point.y);
  double cos_theta = pv_vec.cos(cur_vec);
  if (sqrt(1.0 - cos_theta * cos_theta) * pv_vec.len() < pl_dis_threshold &&
      pv_vec.len() < max_dis && cos_theta > 0.9) {
    updateRay(p, Vector(cur_vec.x, cur_vec.y));

    reached_flag = true;
    return true;
  } else {
    //    std::cout << "ERROR:" << __FILE__ << ":" << __LINE__ << std::endl;
    return false;
  }
}

bool Ray::updateRay(const Point &new_p, const Vector &new_ori) {
  double distance = std::sqrt(std::pow(new_p.x - cur_point.x, 2.0) +
                              std::pow(new_p.y - cur_point.y, 2.0));
  assert(std::isfinite(distance) && "distance calculation with some problem");
  total_distance_ += distance;

  line_list.push_back(
      LineSeg(cur_point, Vector(new_p.x - cur_point.x, new_p.y - cur_point.y)));

  cur_point.x = new_p.x;
  cur_point.y = new_p.y;
  cur_vec.x = new_ori.x;
  cur_vec.y = new_ori.y;
  //  cur_vec = new_ori.normalize();
  return true;
}

std::string Ray::toString() {
  std::string final_str;
  final_str += "{\"type\":\"Ray\"";
  final_str += ",\"start_point\":";
  final_str += start_point.toString();
  final_str += ",\"start_vec\":";
  final_str += start_vec.toString();
  final_str += ",\"dis\":";
  final_str += std::to_string(total_distance_);
  final_str += ",\"Lines\":[";
  for (int i = 0; i < line_list.size(); ++i) {
    final_str += line_list[i].toString();
    if (i < line_list.size() - 1) {
      final_str += ",";

    } else {
      final_str += "]";
    }
  }
  final_str += "}";
  return final_str;
}
