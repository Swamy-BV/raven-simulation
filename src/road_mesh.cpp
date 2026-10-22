// SPDX-License-Identifier: AGPL-3.0-only
#include "road_mesh.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace raven::simulation {
namespace {
using Json = nlohmann::json;
constexpr double kPi = 3.14159265358979323846;

std::string Base64(const std::vector<std::uint8_t>& bytes) {
  constexpr char table[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string result;
  for (std::size_t i = 0; i < bytes.size(); i += 3) {
    const unsigned a = bytes[i], b = i + 1 < bytes.size() ? bytes[i + 1] : 0,
                   c = i + 2 < bytes.size() ? bytes[i + 2] : 0;
    result += table[a >> 2];
    result += table[((a & 3) << 4) | (b >> 4)];
    result += i + 1 < bytes.size() ? table[((b & 15) << 2) | (c >> 6)] : '=';
    result += i + 2 < bytes.size() ? table[c & 63] : '=';
  }
  return result;
}

std::vector<std::uint8_t> ReadTexture(const std::string& name) {
  if (std::filesystem::path(name).filename() != name) {
    throw std::runtime_error("Texture must be a cache filename");
  }
#ifdef _WIN32
  char* value = nullptr;
  std::size_t length = 0;
  if (_dupenv_s(&value, &length, "RAVEN_SIM_ASSETS") != 0) {
    throw std::runtime_error("Cannot read scene texture environment");
  }
  const std::unique_ptr<char, decltype(&std::free)> storage(value, &std::free);
  const char* assets = storage.get();
#else
  const char* assets = std::getenv("RAVEN_SIM_ASSETS");
#endif
  if (!assets) {
    throw std::runtime_error(
        "Set RAVEN_SIM_ASSETS to the verified scene texture cache");
  }
  std::ifstream file(std::filesystem::path(assets) / name, std::ios::binary);
  if (!file) {
    throw std::runtime_error("Missing scene texture: " + name);
  }
  return {std::istreambuf_iterator<char>(file),
          std::istreambuf_iterator<char>()};
}

struct Vertex {
  std::array<float, 3> p;
  std::array<float, 3> normal;
  std::array<float, 2> uv;
};

struct Surface {
  std::vector<Vertex> vertices;
  std::vector<std::uint32_t> indices;

  void Quad(const RoadPoint& a, const RoadPoint& b, const RoadPoint& c,
            const RoadPoint& d, double height, double scale = 4) {
    const auto first = static_cast<std::uint32_t>(vertices.size());
    for (const auto& p : std::array<RoadPoint, 4>{a, b, c, d}) {
      // glTF Y-up maps to Unreal Z-up. NED east is the local glTF Z axis.
      vertices.push_back({{static_cast<float>(p[0]), static_cast<float>(height),
                           static_cast<float>(p[1])},
                          {0, 1, 0},
                          {static_cast<float>(p[0] / scale),
                           static_cast<float>(p[1] / scale)}});
    }
    const double cross =
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    if (cross < 0) {
      for (auto i : {0U, 1U, 2U, 0U, 2U, 3U}) {
        indices.push_back(first + i);
      }
    } else {
      for (auto i : {0U, 2U, 1U, 0U, 3U, 2U}) {
        indices.push_back(first + i);
      }
    }
  }
};

struct Opening {
  RoadPoint center;
  double radius;
};

std::vector<Opening> Openings(const Json& course) {
  std::vector<Opening> result;
  for (const auto& item : course.value("junctions", Json::array())) {
    Opening opening{item.at("center_xy_m").get<RoadPoint>(),
                    item.at("radius_m").get<double>()};
    if (!std::isfinite(opening.radius) || opening.radius < 5 ||
        opening.radius > 50 || !std::isfinite(opening.center[0]) ||
        !std::isfinite(opening.center[1])) {
      throw std::runtime_error("Invalid road junction opening");
    }
    result.push_back(opening);
  }
  return result;
}

void Strip(Surface& mesh, const std::vector<RoadPoint>& points, double left,
           double right, double height, bool dashed = false,
           const std::vector<Opening>& openings = {}) {
  const auto a = OffsetPath(points, left), b = OffsetPath(points, right);
  double distance = 0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const double step = std::hypot(points[i + 1][0] - points[i][0],
                                   points[i + 1][1] - points[i][1]);
    const RoadPoint center{(a[i][0] + b[i][0] + a[i + 1][0] + b[i + 1][0]) / 4,
                           (a[i][1] + b[i][1] + a[i + 1][1] + b[i + 1][1]) / 4};
    const bool open = std::any_of(
        openings.begin(), openings.end(), [&](const Opening& opening) {
          return std::hypot(center[0] - opening.center[0],
                            center[1] - opening.center[1]) < opening.radius;
        });
    if (!open && (!dashed || std::fmod(distance, 8.0) < 4.0)) {
      mesh.Quad(a[i], b[i], b[i + 1], a[i + 1], height);
    }
    distance += step;
  }
}

std::vector<RoadPoint> Circle(const RoadPoint& centre, double radius) {
  std::vector<RoadPoint> result;
  const int count = static_cast<int>(std::ceil(2 * kPi * radius));
  for (int i = 0; i <= count; ++i) {
    const double a = 2 * kPi * i / count;
    result.push_back(
        {centre[0] + radius * std::cos(a), centre[1] + radius * std::sin(a)});
  }
  result.back() = result.front();
  return result;
}

Json Roundabouts(const Json& course) {
  auto result = course.value("roundabouts", Json::array());
  if (course.contains("roundabout")) {
    result.push_back(course.at("roundabout"));
  }
  if (!result.is_array() || result.size() > 8) {
    throw std::runtime_error("Invalid roundabout list");
  }
  return result;
}

struct RoadLink {
  RoadPoint start;
  RoadPoint end;
  double width;
};

std::vector<RoadLink> RoadLinks(const Json& course) {
  std::vector<RoadLink> result;
  for (const auto& item : course.value("crossroads", Json::array())) {
    RoadLink link{item.at("start_xy_m").get<RoadPoint>(),
                  item.at("end_xy_m").get<RoadPoint>(),
                  item.value("width_m", 8.0)};
    const double length =
        std::hypot(link.end[0] - link.start[0], link.end[1] - link.start[1]);
    if (!std::isfinite(length) || length < 10 || length > 500 ||
        !std::isfinite(link.width) || link.width < 4 || link.width > 14) {
      throw std::runtime_error("Invalid crossing road");
    }
    for (const auto& point : {link.start, link.end}) {
      for (const double value : point) {
        if (!std::isfinite(value) || std::abs(value) > 10000) {
          throw std::runtime_error("Invalid crossing coordinate");
        }
      }
    }
    result.push_back(link);
  }
  return result;
}

bool ClearOfLinks(const RoadPoint& point, const std::vector<RoadLink>& links,
                  const double margin) {
  for (const auto& link : links) {
    const double dx = link.end[0] - link.start[0],
                 dy = link.end[1] - link.start[1];
    const double t = std::clamp(
        ((point[0] - link.start[0]) * dx + (point[1] - link.start[1]) * dy) /
            (dx * dx + dy * dy),
        0.0, 1.0);
    if (std::hypot(point[0] - link.start[0] - t * dx,
                   point[1] - link.start[1] - t * dy) <
        link.width / 2 + margin) {
      return false;
    }
  }
  return true;
}

void CrossingRoad(std::array<Surface, 5>& surfaces, const RoadLink& link,
                  const std::vector<Opening>& openings) {
  const double dx = link.end[0] - link.start[0],
               dy = link.end[1] - link.start[1], length = std::hypot(dx, dy);
  const RoadPoint right{dy / length, -dx / length};
  const int count = static_cast<int>(std::ceil(length));
  const auto point = [&](const int index, const double offset) {
    const double t = static_cast<double>(index) / count;
    return RoadPoint{link.start[0] + t * dx + right[0] * offset,
                     link.start[1] + t * dy + right[1] * offset};
  };
  for (int i = 0; i < count; ++i) {
    const double half = link.width / 2;
    surfaces[1].Quad(point(i, -half), point(i, half), point(i + 1, half),
                     point(i + 1, -half), .02);
    const auto center = point(i, 0);
    const bool opening =
        std::any_of(openings.begin(), openings.end(), [&](const Opening& item) {
          return std::hypot(center[0] - item.center[0],
                            center[1] - item.center[1]) < item.radius;
        });
    if (!opening) {
      for (const double sign : {-1.0, 1.0}) {
        surfaces[3].Quad(point(i, sign * (half - .3)),
                         point(i, sign * (half - .15)),
                         point(i + 1, sign * (half - .15)),
                         point(i + 1, sign * (half - .3)), .025);
      }
    }
  }
}

Json Material(const std::string& name, const Json& color) {
  return {{"name", name},
          {"doubleSided", true},
          {"pbrMetallicRoughness",
           {{"baseColorFactor", color},
            {"metallicFactor", 0},
            {"roughnessFactor", 1}}}};
}
}  // namespace

void WriteRoadMesh(const std::vector<RoadPoint>& points, const Json& course,
                   const std::filesystem::path& directory,
                   const bool include_textures) {
  const double height = course.at("surface_z_m").get<double>();
  if (!std::isfinite(height) || std::abs(height) > 10000) {
    throw std::runtime_error("Invalid course surface height");
  }
  const double half = course.value("road_width_m", 14.0) / 2;
  if (!std::isfinite(half) || half < 4 || half > 12) {
    throw std::runtime_error("Continuous road width must be 8 to 24 metres");
  }
  std::array<Surface, 5>
      surfaces;  // grass, asphalt, sidewalk, white markings, yellow centre line
  RoadPoint low = points.front(), high = low;
  for (const auto& p : points) {
    for (std::size_t j = 0; j < 2; ++j) {
      low[j] = std::min(low[j], p[j]);
      high[j] = std::max(high[j], p[j]);
    }
  }
  surfaces[0].Quad({low[0] - 100, low[1] - 100}, {low[0] - 100, high[1] + 100},
                   {high[0] + 100, high[1] + 100},
                   {high[0] + 100, low[1] - 100}, 0, 5);
  const auto openings = Openings(course);
  Strip(surfaces[1], points, -half, half, .02);
  for (double sign : {-1.0, 1.0}) {
    Strip(surfaces[2], points, sign * (half + .2), sign * (half + 2), .12,
          false, openings);
    Strip(surfaces[3], points, sign * (half - .4), sign * (half - .25), .025,
          false, openings);
  }
  if (course.value("centerline", true)) {
    Strip(surfaces[4], points, -.07, .07, .026, true, openings);
  }
  for (const auto& mark : course.value("lane_markings_m", Json::array())) {
    const double offset = mark.get<double>();
    if (!std::isfinite(offset) || std::abs(offset) > half - .5) {
      throw std::runtime_error("Lane marking is outside the road");
    }
    Strip(surfaces[3], points, offset - .06, offset + .06, .026, true,
          openings);
  }
  for (const auto& link : RoadLinks(course)) {
    CrossingRoad(surfaces, link, openings);
  }
  for (const auto& roundabout : Roundabouts(course)) {
    const auto center = roundabout.at("center_xy_m").get<RoadPoint>();
    for (const auto value : center) {
      if (!std::isfinite(value) || std::abs(value) > 10000) {
        throw std::runtime_error("Invalid roundabout centre");
      }
    }
    const double radius = roundabout.at("radius_m").get<double>();
    if (!std::isfinite(radius) || radius < 16 || radius > 200) {
      throw std::runtime_error("Invalid roundabout radius");
    }
    const auto ring = Circle(center, radius);
    Strip(surfaces[1], ring, -half, half, .04);
    Strip(surfaces[3], ring, -half + .25, -half + .4, .045, false, openings);
    Strip(surfaces[3], ring, half + .05, half + .25, .15);
    const auto island = Circle(center, radius - half - .3);
    for (std::size_t i = 0; i + 1 < island.size(); ++i) {
      surfaces[0].Quad(center, island[i], island[i + 1], center, .15, 5);
    }
  }
  Json gltf = {
      {"asset", {{"version", "2.0"}, {"generator", "Raven road geometry"}}},
      {"scene", 0},
      {"scenes", Json::array({{{"nodes", {0}}}})},
      {"nodes", Json::array({{{"mesh", 0}}})},
      {"bufferViews", Json::array()},
      {"accessors", Json::array()},
      {"images", Json::array()},
      {"textures", Json::array()},
      {"samplers", Json::array({{{"wrapS", 10497}, {"wrapT", 10497}}})}};
  Json materials = Json::array({Material("Grass", {1, 1, 1, 1}),
                                Material("Asphalt", {1, 1, 1, 1}),
                                Material("Sidewalk", {.5, .5, .48, 1}),
                                Material("Edge marking", {.85, .85, .8, 1}),
                                Material("Centre marking", {.85, .62, .1, 1})});
  for (int material = 0; include_textures && material < 2; ++material) {
    const std::string base =
        course.at(material == 0 ? "grass_texture" : "asphalt_texture")
            .get<std::string>();
    for (int channel = 0; channel < 2; ++channel) {
      const auto image = ReadTexture(
          base + (channel == 0 ? "_diff_1k.jpg" : "_nor_gl_1k.jpg"));
      const auto texture = gltf["textures"].size();
      gltf["images"].push_back(
          {{"uri", "data:image/jpeg;base64," + Base64(image)}});
      gltf["textures"].push_back({{"source", texture}, {"sampler", 0}});
      if (channel == 0) {
        materials[material]["pbrMetallicRoughness"]["baseColorTexture"] = {
            {"index", texture}};
      } else {
        materials[material]["normalTexture"] = {{"index", texture},
                                                {"scale", .45}};
      }
    }
  }
  if (!include_textures) {
    gltf.erase("images");
    gltf.erase("textures");
    gltf.erase("samplers");
  }
  gltf["materials"] = materials;
  std::vector<std::uint8_t> bytes;
  const auto append = [&](const auto& values, const int type,
                          const int components, const char* shape) {
    const auto offset = bytes.size();
    const auto* data = reinterpret_cast<const std::uint8_t*>(values.data());
    bytes.insert(bytes.end(), data, data + values.size() * sizeof(values[0]));
    const auto view = gltf["bufferViews"].size(),
               accessor = gltf["accessors"].size();
    gltf["bufferViews"].push_back({{"buffer", 0},
                                   {"byteOffset", offset},
                                   {"byteLength", bytes.size() - offset}});
    gltf["accessors"].push_back({{"bufferView", view},
                                 {"componentType", type},
                                 {"count", values.size() / components},
                                 {"type", shape}});
    return accessor;
  };
  Json primitives = Json::array();
  for (std::size_t material = 0; material < surfaces.size(); ++material) {
    const auto& mesh = surfaces[material];
    if (mesh.vertices.empty()) {
      continue;
    }
    std::vector<float> p, n, uv;
    std::array<float, 3> minimum{std::numeric_limits<float>::max(),
                                 std::numeric_limits<float>::max(),
                                 std::numeric_limits<float>::max()},
        maximum{-minimum[0], -minimum[1], -minimum[2]};
    for (const auto& v : mesh.vertices) {
      p.insert(p.end(), v.p.begin(), v.p.end());
      n.insert(n.end(), v.normal.begin(), v.normal.end());
      uv.insert(uv.end(), v.uv.begin(), v.uv.end());
      for (std::size_t j = 0; j < 3; ++j) {
        minimum[j] = std::min(minimum[j], v.p[j]);
        maximum[j] = std::max(maximum[j], v.p[j]);
      }
    }
    const auto position = append(p, 5126, 3, "VEC3");
    gltf["accessors"][position]["min"] = minimum;
    gltf["accessors"][position]["max"] = maximum;
    const auto normal = append(n, 5126, 3, "VEC3"),
               tex = append(uv, 5126, 2, "VEC2"),
               indices = append(mesh.indices, 5125, 1, "SCALAR");
    primitives.push_back(
        {{"attributes",
          {{"POSITION", position}, {"NORMAL", normal}, {"TEXCOORD_0", tex}}},
         {"indices", indices},
         {"material", material}});
  }
  gltf["meshes"] = Json::array({{{"primitives", primitives}}});
  gltf["buffers"] = Json::array(
      {{{"byteLength", bytes.size()},
        {"uri", "data:application/octet-stream;base64," + Base64(bytes)}}});
  std::ofstream output(directory / "road.gltf", std::ios::binary);
  if (!output || !(output << gltf.dump())) {
    throw std::runtime_error("Cannot write continuous road mesh");
  }
}

Json RoadScenery(const std::vector<RoadPoint>& points, const Json& course) {
  Json result = Json::array();
  double distance = 0;
  unsigned count = 0;
  const auto roundabouts = Roundabouts(course);
  const auto links = RoadLinks(course);
  const double setback = course.value("building_offset_m", 32.0);
  if (!std::isfinite(setback) || setback < 24 || setback > 80) {
    throw std::runtime_error("Invalid building setback");
  }
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    distance += std::hypot(points[i][0] - points[i - 1][0],
                           points[i][1] - points[i - 1][1]);
    if (distance < 45) {
      continue;
    }
    distance = 0;
    const double yaw = std::atan2(points[i + 1][1] - points[i][1],
                                  points[i + 1][0] - points[i][0]);
    const double side = count % 2 ? 1 : -1;
    const RoadPoint p{points[i][0] - side * std::sin(yaw) * setback,
                      points[i][1] + side * std::cos(yaw) * setback};
    // Avoid placing houses inside the island or across a nearby road section.
    bool clear = ClearOfLinks(p, links, 15);
    for (const auto& island : roundabouts) {
      const auto center = island.at("center_xy_m").get<RoadPoint>();
      if (std::hypot(p[0] - center[0], p[1] - center[1]) <
          island.at("radius_m").get<double>() + 24) {
        clear = false;
      }
    }
    for (const auto& road : points) {
      if (std::hypot(p[0] - road[0], p[1] - road[1]) < 24) {
        clear = false;
        break;
      }
    }
    ++count;
    if (!clear) {
      continue;
    }
    const bool tree = !course.contains("trees") && count % 3 == 0;
    result.push_back(
        {{"name", "Roadside" + std::to_string(count)},
         {"asset",
          course.at(tree ? "tree_asset"
                         : (count % 2 ? "small_house_asset" : "house_asset"))},
         {"position", {p[0], p[1], course.at("surface_z_m")}},
         {"yaw", yaw + side * kPi / 2},
         {"scale",
          tree ? Json::array({.65, .65, .65}) : Json::array({.75, .75, .75})}});
  }
  unsigned island_index = 0;
  for (const auto& island : roundabouts) {
    const auto center = island.at("center_xy_m");
    result.push_back(
        {{"name", "IslandTree" + std::to_string(island_index++)},
         {"asset", course.at("tree_asset")},
         {"position",
          {center[0], center[1], course.at("surface_z_m").get<double>() - .15}},
         {"yaw", 0},
         {"scale", {.55, .55, .55}}});
  }
  if (course.contains("trees")) {
    const auto& trees = course.at("trees");
    const double spacing = trees.at("spacing_m").get<double>();
    const double offset = trees.at("offset_m").get<double>();
    const double scale = trees.value("scale", .6);
    if (!std::isfinite(spacing) || spacing < 24 || spacing > 100 ||
        !std::isfinite(offset) || offset < 16 || offset > 30 ||
        !std::isfinite(scale) || scale < .25 || scale > 1) {
      throw std::runtime_error("Invalid roadside tree rows");
    }
    unsigned index = 0;
    for (const double side : {-1.0, 1.0}) {
      const auto row = OffsetPath(points, side * offset);
      double travelled = spacing;
      for (std::size_t i = 0; i + 1 < points.size(); ++i) {
        if (i > 0) {
          travelled += std::hypot(points[i][0] - points[i - 1][0],
                                  points[i][1] - points[i - 1][1]);
        }
        if (travelled < spacing) {
          continue;
        }
        travelled = 0;
        bool clear = ClearOfLinks(row[i], links, 8);
        for (const auto& road : points) {
          if (std::hypot(row[i][0] - road[0], row[i][1] - road[1]) <
              course.value("road_width_m", 14.0) / 2 + 8) {
            clear = false;
            break;
          }
        }
        if (!clear) {
          continue;
        }
        result.push_back(
            {{"name", "TreeRow" + std::to_string(++index)},
             {"asset", course.at("tree_asset")},
             {"position", {row[i][0], row[i][1], course.at("surface_z_m")}},
             {"yaw", static_cast<double>(index % 7) * .8},
             {"scale", {scale, scale, scale}}});
      }
    }
  }
  return result;
}
}  // namespace raven::simulation
