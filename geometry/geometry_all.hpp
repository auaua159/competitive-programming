#pragma once
#include "line.hpp"

namespace geometry {
  struct Segment : Line {
    Segment() = default;

    using Line::Line;
  };

  using Segments = vector< Segment >;
}

#include "point.hpp"
#include "line.hpp"
#include "projection.hpp"

namespace geometry {
  // http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_1_B
  Point reflection(const Line &l, const Point &p) {
    return p + (projection(l, p) - p) * 2;
  }
}

#pragma once

#include "point.hpp"
#include "line.hpp"

namespace geometry {
  // http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_1_A
  Point projection(const Line &l, const Point &p) {
    auto t = dot(p - l.a, l.a - l.b) / norm(l.a - l.b);
    return l.a + (l.a - l.b) * t;
  }
}

#pragma once

#include "point.hpp"

namespace geometry {
  using Polygon = vector< Point >;
  using Polygons = vector< Polygon >;
}

#pragma once
#include "base.hpp"

namespace geometry {
  using Point = complex< Real >;

  istream &operator>>(istream &is, Point &p) {
    Real a, b;
    is >> a >> b;
    p = Point(a, b);
    return is;
  }

  ostream &operator<<(ostream &os, const Point &p) {
    return os << real(p) << " " << imag(p);
  }

  Point operator*(const Point &p, const Real &d) {
    return Point(real(p) * d, imag(p) * d);
  }

  // rotate point p counterclockwise by theta rad
  Point rotate(Real theta, const Point &p) {
    return Point(cos(theta) * real(p) - sin(theta) * imag(p), sin(theta) * real(p) + cos(theta) * imag(p));
  }

  Real cross(const Point &a, const Point &b) {
    return real(a) * imag(b) - imag(a) * real(b);
  }

  Real dot(const Point &a, const Point &b) {
    return real(a) * real(b) + imag(a) * imag(b);
  }

  bool compare_x(const Point &a, const Point &b) {
    return equals(real(a), real(b)) ? imag(a) < imag(b) : real(a) < real(b);
  }

  bool compare_y(const Point &a, const Point &b) {
    return equals(imag(a), imag(b)) ? real(a) < real(b) : imag(a) < imag(b);
  }

  using Points = vector< Point >;
}

#pragma once
#include "point.hpp"

namespace geometry {
  struct Line {
    Point a, b;

    Line() = default;

    Line(const Point &a, const Point &b) : a(a), b(b) {}

    Line(const Real &A, const Real &B, const Real &C) { // Ax+By=C
      if(equals(A, 0)) {
        assert(!equals(B, 0));
        a = Point(0, C / B);
        b = Point(1, C / B);
      } else if(equals(B, 0)) {
        a = Point(C / A, 0);
        b = Point(C / A, 1);
      } else {
        a = Point(0, C / B);
        b = Point(C / A, 0);
      }
    }

    friend ostream &operator<<(ostream &os, Line &l) {
      return os << l.a << " to " << l.b;
    }

    friend istream &operator>>(istream &is, Line &l) {
      return is >> l.a >> l.b;
    }
  };

  using Lines = vector< Line >;
}

#include "point.hpp"
#include "line.hpp"

namespace geometry {
  // http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_2_A
  bool is_parallel(const Line &a, const Line &b) {
    return equals(cross(a.b - a.a, b.b - b.a), 0.0);
  }
}

#include "point.hpp"
#include "line.hpp"

namespace geometry {
  // http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_2_A
  bool is_orthogonal(const Line &a, const Line &b) {
    return equals(dot(a.a - a.b, b.a - b.b), 0.0);
  }
}

#include "point.hpp"
#include "ccw.hpp"
#include "segment.hpp"


namespace geometry {
  bool is_intersect_ss(const Segment &s, const Segment &t) {
    return ccw(s.a, s.b, t.a) * ccw(s.a, s.b, t.b) <= 0 &&
           ccw(t.a, t.b, s.a) * ccw(t.a, t.b, s.b) <= 0;
  }
}

#include "point.hpp"
#include "segment.hpp"
#include "ccw.hpp"

namespace geometry {
  bool is_intersect_sp(const Segment &s, const Point &p) {
    return ccw(s.a, s.b, p) == ON_SEGMENT;
  }
}

#include "base.hpp"
#include "point.hpp"
#include "line.hpp"
#include "segment.hpp"

namespace geometry {
  bool is_intersect_ls(const Line &l, const Segment &s) {
    return sign(cross(l.b - l.a, s.a - l.a)) * sign(cross(l.b - l.a, s.b - l.a)) <= 0;
  }
}

#include "point.hpp"
#include "line.hpp"
#include "ccw.hpp"

namespace geometry {
  bool is_intersect_lp(const Line &l, const Point &p) {
    return abs(ccw(l.a, l.b, p)) != 1;
  }
}

#include "line.hpp"
#include "is_parallel.hpp"

namespace geometry {
  bool is_intersect_ll(const Line &l, const Line &m) {
    Real A = cross(l.b - l.a, m.b - m.a);
    Real B = cross(l.b - l.a, l.b - m.a);
    if(equals(abs(A), 0) && equals(abs(B), 0)) return true;
    return !is_parallel(l, m);
  }
}

#include "base.hpp"
#include "point.hpp"
#include "segment.hpp"
#include "circle.hpp"
#include "projection.hpp"

namespace geometry {
  int is_intersect_cs(const Circle &c, const Segment &l) {
    Point h = projection(l, c.p);
    if(sign(norm(h - c.p) - norm(c.r)) > 0) return 0;
    auto d1 = abs(c.p - l.a), d2 = abs(c.p - l.b);
    if(sign(c.r - d1) >= 0 && sign(c.r - d2) >= 0) return 0;
    if(sign(c.r - d1) < 0 && sign(d2 - c.r) > 0 || sign(d1 - c.r) > 0 && sign(c.r - d2) < 0) return 1;
    if(sign(dot(l.a - h, l.b - h)) < 0) return 2;
    return 0;
  }
}

#include "base.hpp"
#include "point.hpp"
#include "circle.hpp"

namespace geometry {
  bool is_intersect_cp(const Circle &c, const Point &p) {
    return equals(abs(p - c.p) - c.r, 0);
  }
}

#include "base.hpp"
#include "line.hpp"
#include "circle.hpp"
#include "distance_lp.hpp"

namespace geometry {
  bool is_intersect_cl(const Circle &c, const Line &l) {
    return sign(c.r - distance_lp(l, c.p)) >= 0;
  }
}

#include "point.hpp"
#include "ccw.hpp"
#include "polygon.hpp"

namespace geometry {
  // http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_3_B
  bool is_convex_polygon(const Polygon &p) {
    int n = (int) p.size();
    for(int i = 0; i < n; i++) {
      if(ccw(p[(i + n - 1) % n], p[i], p[(i + 1) % n]) == CLOCKWISE) return false;
    }
    return true;
  }
}

#include "segment.hpp"
#include "is_intersect_ss.hpp"
#include "distance_sp.hpp"

namespace geometry {
  // http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_2_D
  Real distance_ss(const Segment &a, const Segment &b) {
    if(is_intersect_ss(a, b)) return 0;
    return min({distance_sp(a, b.a), distance_sp(a, b.b), distance_sp(b, a.a), distance_sp(b, a.b)});
  }
}

#include "point.hpp"
#include "segment.hpp"
#include "projection.hpp"
#include "is_intersect_sp.hpp"

namespace geometry {
  Real distance_sp(const Segment &s, const Point &p) {
    Point r = projection(s, p);
    if(is_intersect_sp(s, r)) return abs(r - p);
    return min(abs(s.a - p), abs(s.b - p));
  }
}

#include "point.hpp"

namespace geometry {
  Real distance(const Point &a, const Point &b) {
    return abs(a - b);
  }
}

#include "line.hpp"
#include "projection.hpp"

namespace geometry {
  Real distance_lp(const Line &l, const Point &p) {
    return abs(p - projection(l, p));
  }
}

#include "line.hpp"
#include "is_intersect_ll.hpp"
#include "distance_lp.hpp"

namespace geometry {
  Real distance_ll(const Line &l, const Line &m) {
    return is_intersect_ll(l, m) ? 0 : distance_lp(l, m.a);
  }
}
#include "base.hpp"
#include "line.hpp"

namespace geometry {
  Point cross_point_ll(const Line &l, const Line &m) {
    Real A = cross(l.b - l.a, m.b - m.a);
    Real B = cross(l.b - l.a, l.b - m.a);
    if(equals(abs(A), 0) && equals(abs(B), 0)) return m.a;
    return m.a + (m.b - m.a) * B / A;
  }
}
#include "point.hpp"
#include "segment.hpp"
#include "circle.hpp"
#include "is_intersect_cs.hpp"
#include "cross_point_cl.hpp"

namespace geometry {
  Points cross_point_cs(const Circle &c, const Segment &s) {
    int num = is_intersect_cs(c, s);
    if(num == 0) return {};
    if(num == 2) return cross_point_cl(c, s);
    auto ret = cross_point_cl(c, s);
    if(dot(s.a - ret[0], s.b - ret[0]) > 0) swap(ret[0], ret[1]);
    return {ret[0]};
  }
}
#include "base.hpp"
#include "point.hpp"
#include "line.hpp"
#include "circle.hpp"
#include "projection.hpp"

namespace geometry {
  // http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_7_D
  Points cross_point_cl(const Circle &c, const Line &l) {
    Point pr = projection(l, c.p);
    if(equals(abs(pr - c.p), c.r)) return {pr};
    Point e = (l.b - l.a) / abs(l.b - l.a);
    auto k = sqrt(norm(c.r) - norm(pr - c.p));
    return {pr - e * k, pr + e * k};
  }
}
#include "base.hpp"
#include "point.hpp"
#include "circle.hpp"

namespace geometry {
  // http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_7_E
  Points cross_point_cc(const Circle &c1, const Circle &c2) {
    Real d = abs(c1.p - c2.p), r = c1.r + c2.r;
    if(sign(d - r) > 0 or sign(d + c1.r - c2.r) < 0) return {};
    Real a = acos((norm(c1.r) - norm(c2.r) + norm(d)) / (2 * c1.r * d));
    Real t = arg(c2.p - c1.p);
    Point p = c1.p + polar(c1.r, t + a);
    Point q = c1.p + polar(c1.r, t - a);
    if(equals(real(p), real(q)) && equals(imag(p), imag(q))) return {p};
    return {p, q};
  }
}
#include "point.hpp"
#include "polygon.hpp"

namespace geometry {
  // http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_4_B
  pair< int, int > convex_polygon_diameter(const Polygon &p) {
    int N = (int) p.size();
    int is = 0, js = 0;
    for(int i = 1; i < N; i++) {
      if(imag(p[i]) > imag(p[is])) is = i;
      if(imag(p[i]) < imag(p[js])) js = i;
    }
    Real maxdis = norm(p[is] - p[js]);

    int maxi, maxj, i, j;
    i = maxi = is;
    j = maxj = js;
    do {
      if(cross(p[(i + 1) % N] - p[i], p[(j + 1) % N] - p[j]) >= 0) {
        j = (j + 1) % N;
      } else {
        i = (i + 1) % N;
      }
      if(norm(p[i] - p[j]) > maxdis) {
        maxdis = norm(p[i] - p[j]);
        maxi = i;
        maxj = j;
      }
    } while(i != is || j != js);
    return minmax(maxi, maxj);
  }
}
#include "base.hpp"
#include "point.hpp"
#include "line.hpp"
#include "polygon.hpp"
#include "cross_point_ll.hpp"

namespace geometry {
  // http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_4_C
  // cut with a straight line l and return a convex polygon on the left
  Polygon convex_polygon_cut(const Polygon &U, const Line &l) {
    Polygon ret;
    for(int i = 0; i < U.size(); i++) {
      const Point &now = U[i];
      const Point &nxt = U[(i + 1) % U.size()];
      auto cf = cross(l.a - now, l.b - now);
      auto cs = cross(l.a - nxt, l.b - nxt);
      if(sign(cf) >= 0) {
        ret.emplace_back(now);
      }
      if(sign(cf) * sign(cs) < 0) {
        ret.emplace_back(cross_point_ll(Line(now, nxt), l));
      }
    }
    return ret;
  }
}
#include "base.hpp"
#include "point.hpp"
#include "polygon.hpp"

namespace geometry {
  int convex_polygon_contains(const Polygon &Q, const Point &p) {
    int N = (int) Q.size();
    Point g = (Q[0] + Q[N / 3] + Q[N * 2 / 3]) / 3.0;
    if(equals(imag(g), imag(p)) && equals(real(g), imag(g))) return IN;
    Point gp = p - g;
    int l = 0, r = N;
    while(r - l > 1) {
      int mid = (l + r) / 2;
      Point gl = Q[l] - g;
      Point gm = Q[mid] - g;
      if(cross(gl, gm) > 0) {
        if(cross(gl, gp) >= 0 && cross(gm, gp) <= 0) r = mid;
        else l = mid;
      } else {
        if(cross(gl, gp) <= 0 && cross(gm, gp) >= 0) l = mid;
        else r = mid;
      }
    }
    r %= N;
    Real v = cross(Q[l] - p, Q[r] - p);
    return sign(v) == 0 ? ON : sign(v) == -1 ? OUT : IN;
  }
}
#include "base.hpp"
#include "point.hpp"
#include "polygon.hpp"

namespace geometry {
  // http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_4_A
  Polygon convex_hull(Polygon &p, bool strict = true) {
    int n = (int) p.size(), k = 0;
    if(n <= 2) return p;
    sort(begin(p), end(p), compare_x);
    vector< Point > ch(2 * n);
    auto check = [&](int i) {
      return sign(cross(ch[k - 1] - ch[k - 2], p[i] - ch[k - 1])) <= -1 + strict;
    };
    for(int i = 0; i < n; ch[k++] = p[i++]) {
      while(k >= 2 && check(i)) --k;
    }
    for(int i = n - 2, t = k + 1; i >= 0; ch[k++] = p[i--]) {
      while(k >= t && check(i)) --k;
    }
    ch.resize(k - 1);
    return ch;
  }
}
#include "base.hpp"
#include "point.hpp"
#include "polygon.hpp"

namespace geometry {
  // http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_3_C
  int contains(const Polygon &Q, const Point &p) {
    bool in = false;
    for(int i = 0; i < Q.size(); i++) {
      Point a = Q[i] - p, b = Q[(i + 1) % Q.size()] - p;
      if(imag(a) > imag(b)) swap(a, b);
      if(sign(imag(a)) <= 0 && 0 < sign(imag(b)) && sign(cross(a, b)) < 0) in = !in;
      if(equals(cross(a, b), 0) && sign(dot(a, b)) <= 0) return ON;
    }
    return in ? IN : OUT;
  }
}
#include "base.hpp"
#include "point.hpp"
#include "polygon.hpp"
#include "distance_sp.hpp"
#include "cross_point_cl.hpp"
#include "is_intersect_cs.hpp"

namespace geometry {
  // http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_7_H
  Real ca_cp_impl(const Circle &c, const Point &a, const Point &b) {
    auto va = c.p - a, vb = c.p - b;
    Real f = cross(va, vb), ret = 0;
    if(sign(f) == 0) return ret;
    if(sign(max(abs(va), abs(vb)) - c.r) <= 0) return f;
    if(sign(distance_sp(Segment(a, b), c.p) - c.r) >= 0) return norm(c.r) * arg(vb * conj(va));
    auto tot = cross_point_cl(c, Line(a, b));
    if(is_intersect_cs(c, Segment(a, b)) != 2 and dot(a - tot[0], b - tot[0]) < 0) {
      swap(tot[0], tot[1]);
    }
    tot.emplace(begin(tot), a);
    tot.emplace_back(b);
    for(int i = 1; i < (int) tot.size(); i++) {
      ret += ca_cp_impl(c, tot[i - 1], tot[i]);
    }
    return ret;
  }

  Real common_area_cp(const Circle &c, const Polygon &p) {
    if(p.size() < 3) return 0;
    Real A = 0;
    for(int i = 0; i < p.size(); i++) {
      A += ca_cp_impl(c, p[i], p[(i + 1) % p.size()]);
    }
    return A * 0.5;
  }
}
#pragma once
#include "point.hpp"

namespace geometry {
  struct Circle {
    Point p;
    Real r{};

    Circle() = default;

    Circle(const Point &p, const Real &r) : p(p), r(r) {}
  };

  using Circles = vector< Circle >;
}
#pragma once
#include "point.hpp"

namespace geometry {
  // http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_1_C
  constexpr int COUNTER_CLOCKWISE = +1;
  constexpr int CLOCKWISE = -1;
  constexpr int ONLINE_BACK = +2; // c-a-b
  constexpr int ONLINE_FRONT = -2; // a-b-c
  constexpr int ON_SEGMENT = 0; // a-c-b
  int ccw(const Point &a, Point b, Point c) {
    b = b - a, c = c - a;
    if(sign(cross(b, c)) == +1) return COUNTER_CLOCKWISE;
    if(sign(cross(b, c)) == -1) return CLOCKWISE;
    if(sign(dot(b, c)) == -1) return ONLINE_BACK;
    if(norm(b) < norm(c)) return ONLINE_FRONT;
    return ON_SEGMENT;
  }
}
#pragma once

namespace geometry {
  using Real = double;
  const Real EPS = 1e-8;
  const Real PI = acos(static_cast< Real >(-1));

  enum {
    OUT, ON, IN
  };

  inline int sign(const Real &r) {
    return r <= -EPS ? -1 : r >= EPS ? 1 : 0;
  }

  inline bool equals(const Real &a, const Real &b) {
    return sign(a - b) == 0;
  }
}
#include "point.hpp"
#include "polygon.hpp"

namespace geometry {
  // http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_3_A
  Real area(const Polygon &p) {
    int n = (int) p.size();
    Real A = 0;
    for(int i = 0; i < n; ++i) {
      A += cross(p[i], p[(i + 1) % n]);
    }
    return A * 0.5;
  }
}
#include "point.hpp"

namespace geometry {
  Real radian_to_degree(const Real &theta) {
    return theta * 180.0 / PI;
  }

  Real degree_to_radian(const Real &deg) {
    return deg * PI / 180.0;
  }

  // smaller angle of the a-b-c
  Real get_smaller_angle(const Point &a, const Point &b, const Point &c) {
    const Point v(a - b), w(c - b);
    auto alpha = atan2(imag(v), real(v));
    auto beta = atan2(imag(w), real(w));
    if(alpha > beta) swap(alpha, beta);
    Real theta = (beta - alpha);
    return min(theta, 2 * PI - theta);
  }
}
