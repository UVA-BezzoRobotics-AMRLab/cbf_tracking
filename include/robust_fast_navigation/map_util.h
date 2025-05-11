#pragma once

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

namespace map_util
{

// struct mimicing nav_msgs::OccupancyGrid
struct occupancy_grid
{
  unsigned char *data;
  int width;
  int height;
  double resolution;
  double origin_x;
  double origin_y;

  std::vector<unsigned char> occupied_values;

  occupancy_grid ()
  {
    width = 0;
    height = 0;
    resolution = 0.0;
    origin_x = 0.0;
    origin_y = 0.0;
    data = nullptr;
  }

  occupancy_grid (int w, int h, double res, double ox, double oy,
                  unsigned char *d, std::vector<unsigned char> ov)
  {
    width = w;
    height = h;
    resolution = res;
    origin_x = ox;
    origin_y = oy;
    data = d;
    occupied_values = ov;
  }

  // define these functions with vectors so we can pybind them more easily
  std::vector<unsigned int>
  world_to_map (double x, double y) const
  {
    if (x < origin_x || y < origin_y)
      throw std::invalid_argument (
          "[world_to_map] x or y is less than origin");

    unsigned int mx = (int)((x - origin_x) / resolution);
    unsigned int my = (int)((y - origin_y) / resolution);

    if (mx >= width || my >= height)
      throw std::invalid_argument (
          "[world_to_map] mx or my is greater than width or height");

    return { mx, my };
  }

  std::vector<double>
  map_to_world (unsigned int mx, unsigned int my) const
  {
    if (mx > width || my > height)
      throw std::invalid_argument (
          "[map_to_world] mx or my is greater than width or height");

    double x = (mx + .5) * resolution + origin_x;
    double y = (my + .5) * resolution + origin_y;

    return { x, y };
  }

  std::vector<unsigned int>
  index_to_cells (unsigned int index) const
  {
    if (index > width * height)
      throw std::invalid_argument (
          "[index_to_cells] index is greater than width * height");

    unsigned int mx = index % width;
    unsigned int my = index / width;

    return { mx, my };
  }

  unsigned int
  cells_to_index (unsigned int mx, unsigned int my) const
  {
    if (mx > width || my > height)
      throw std::invalid_argument (
          "[cells_to_index] mx or my is greater than width or height");

    return my * width + mx;
  }

  unsigned char *
  get_data () const
  {
    return data;
  }

  unsigned char
  get_cost (unsigned int mx, unsigned int my) const
  {
    return data[cells_to_index (mx, my)];
  }

  unsigned char
  get_cost (unsigned int index) const
  {
    return data[index];
  }

  bool
  is_occupied (double x, double y) const
  {
    std::vector<unsigned int> cells = world_to_map (x, y);
    return is_occupied (cells[0], cells[1]);
  }

  bool
  is_occupied (unsigned int mx, unsigned int my) const
  {
    return std::find (occupied_values.begin (), occupied_values.end (),
                      data[cells_to_index (mx, my)])
           != occupied_values.end ();
  }

  bool
  is_occupied (unsigned int index) const
  {
    return std::find (occupied_values.begin (), occupied_values.end (),
                      data[index])
           != occupied_values.end ();
  }

  void
  raycast (unsigned int sx, unsigned int sy, unsigned int ex, unsigned int ey,
           double &x, double &y, const std::vector<unsigned char> &test_val,
           unsigned int max_range = 1e6)
  {
    unsigned int size_x = width;

    int dx = ex - sx;
    int dy = ey - sy;

    unsigned int abs_dx = abs (dx);
    unsigned int abs_dy = abs (dy);

    int offset_dx = dx > 0 ? 1 : -1;
    int offset_dy = (dy > 0 ? 1 : -1) * size_x;

    unsigned int offset = sy * size_x + sx;

    double dist = hypot (dx, dy);
    double scale = (dist == 0.0) ? 1.0 : std::min (1.0, max_range / dist);

    unsigned int term;
    if (abs_dx >= abs_dy)
      {
        int error_y = abs_dx / 2;
        bresenham (abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset,
                   (unsigned int)(scale * abs_dx), term, test_val);
      }
    else
      {
        int error_x = abs_dy / 2;
        bresenham (abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset,
                   (unsigned int)(scale * abs_dy), term, test_val);
      }

    // convert costmap index to world coordinates
    unsigned int mx, my;
    std::vector<unsigned int> cells = index_to_cells (term);
    mx = cells[0];
    my = cells[1];

    std::vector<double> world = map_to_world (mx, my);
    x = world[0];
    y = world[1];
  }

  // following bresenham / raycast method from
  // https://docs.ros.org/en/api/costmap_2d/html/costmap__2d_8h_source.html
  void
  bresenham (unsigned int abs_da, unsigned int abs_db, int error_b,
             int offset_a, int offset_b, unsigned int offset,
             unsigned int max_range, unsigned int &term,
             const std::vector<unsigned char> &test_val)
  {

    unsigned int end = std::min (max_range, abs_da);
    unsigned int mx, my;
    for (unsigned int i = 0; i < end; ++i)
      {
        offset += offset_a;
        error_b += abs_db;

        std::vector<unsigned int> cells = index_to_cells (offset);
        mx = cells[0];
        my = cells[1];

        unsigned char cost = get_cost (mx, my);
        if (std::find (test_val.begin (), test_val.end (), cost)
            != test_val.end ())
          {
            break;
          }

        if ((unsigned int)error_b >= abs_da)
          {
            offset += offset_b;
            error_b -= abs_da;
          }

        cells = index_to_cells (offset);
        mx = cells[0];
        my = cells[1];

        cost = get_cost (mx, my);
        if (std::find (test_val.begin (), test_val.end (), cost)
            != test_val.end ())
          {
            break;
          }
      }

    term = offset;
  }
};
typedef struct occupancy_grid occupancy_grid_t;

inline occupancy_grid_t
costmap_to_occgrid (const costmap_2d::Costmap2D &costmap)
{
  int w = costmap.getSizeInCellsX ();
  int h = costmap.getSizeInCellsY ();
  double res = costmap.getResolution ();
  double ox = costmap.getOriginX ();
  double oy = costmap.getOriginY ();
  std::vector<unsigned char> ov = { costmap_2d::LETHAL_OBSTACLE,
                                    costmap_2d::INSCRIBED_INFLATED_OBSTACLE };

  occupancy_grid_t occ_grid (w, h, res, ox, oy, costmap.getCharMap (), ov);

  return occ_grid;
}

} // end namespace map_util
