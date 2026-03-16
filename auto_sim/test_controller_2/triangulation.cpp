#include "triangulation.hpp"

void Triangulation::legalize_edge(const size_t new_cone, const size_t cone_a,
                                  const size_t cone_b) {}

bool Triangulation::on_edge(const Cone &cone, const size_t a,
                            const size_t b) const {
  // check if cone lies on the edge (a,b)
  const Cone &cone_a = existing_cones[a];
  const Cone &cone_b = existing_cones[b];
  // check if cone is collinear with cone_a and cone_b
  if (const double cross_product = (cone.y - cone_a.y) * (cone_b.x - cone_a.x) -
                                   (cone.x - cone_a.x) * (cone_b.y - cone_a.y);
      std::abs(cross_product) > 1e-6) {
    return false;
  }
  // check if cone is within the bounding box of cone_a and cone_b
  const double dot_product = (cone.x - cone_a.x) * (cone_b.x - cone_a.x) +
                             (cone.y - cone_a.y) * (cone_b.y - cone_a.y);
  if (dot_product < 0) {
    return false;
  }
  const double squared_length_ab =
      (cone_b.x - cone_a.x) * (cone_b.x - cone_a.x) +
      (cone_b.y - cone_a.y) * (cone_b.y - cone_a.y);
  if (dot_product > squared_length_ab) {
    return false;
  }
  return true;
}

bool Triangulation::edge_detect(const Cone &cone, size_t &a, size_t &b,
                                size_t &c) const {
  if (on_edge(cone, a, b)) {
    return true;
  }
  if (on_edge(cone, b, c)) {
    std::swap(a, c);
    return true;
  }
  if (on_edge(cone, a, c)) {
    std::swap(b, c);
    return true;
  }
  return false;
}

void Triangulation::insert(const Cone &cone, const size_t cone_id) {
  // find smallest triangle which contains the cone
  const std::shared_ptr<TrianglePyramid> t =
      root.find_triangle(cone, existing_cones);
  size_t a = t->t.a, b = t->t.b, c = t->t.c;
  if (bool on_edge = edge_detect(cone, a, b, c)) {
    // if on_edge, then cone is known to be on edge (a,b)
    // very annoying
    throw std::runtime_error(
        "Edge splitting not implemented yet"); // inshallah we will never need
                                               // to implement this
  }
  // split triangle a,b,c into three triangles a,b,cone_id;
  // b,c,cone_id; a,c,cone_id
  adj[cone_id] = {a, b, c};
  adj[a].push_back(cone_id);
  adj[b].push_back(cone_id);
  adj[c].push_back(cone_id);
  t->children.push_back(std::make_shared<TrianglePyramid>(a, b, cone_id));
  t->children.push_back(std::make_shared<TrianglePyramid>(b, c, cone_id));
  t->children.push_back(std::make_shared<TrianglePyramid>(a, c, cone_id));

  // legalize the edges
  legalize_edge(cone_id, a, b);
  legalize_edge(cone_id, b, c);
  legalize_edge(cone_id, a, c);
}

/**
 * Triangulates the cones. Populates the adj with the edges of the
 * triangulation.
 * @param cones The vector of cones to triangulate.
 * @note This function assumes that the input vector of cones is an add-only
 * array whose order is preserved.
 */
void Triangulation::update(const std::vector<Cone> &cones) {
  static size_t last_seen_cone = 0;
  for (; last_seen_cone < cones.size(); ++last_seen_cone) {
    const Cone &cone = cones[last_seen_cone];
    // process the new cone
    const size_t cone_id =
        last_seen_cone + 3; // cones 0,1,2 are reserved for the super triangle
    const std::span<const Cone> s{cones};
    existing_cones.set_cones(s.first(last_seen_cone));
    insert(cone, cone_id);
  }
}