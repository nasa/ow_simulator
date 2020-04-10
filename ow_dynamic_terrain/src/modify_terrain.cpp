#include "modify_terrain.h"
#include <gazebo/rendering/Conversions.hh>

using namespace std;
using namespace Ogre;
using namespace gazebo;
using namespace rendering;

void ModifyTerrain::modify(Heightmap* heightmap,
        Vector3 terrain_position,
        double outside_radius, double inside_radius, double weight,
        const string& op,
        function<float (long, long)> get_height_value,
        function<void (long, long, float)> set_height_value)
{
    auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);

    if (!terrain)
    {
        gzerr << "DynamicTerrain: Heightmap has no associated terrain object!" << endl;
        return;
    }

    Vector3 heightmap_position;
    terrain->getTerrainPosition(terrain_position, &heightmap_position);

    auto size = static_cast<int>(terrain->getSize());
    auto left = max(int((heightmap_position.x - outside_radius) * size), 0);
    auto top = max(int((heightmap_position.y - outside_radius) * size), 0);
    auto right = min(int((heightmap_position.x + outside_radius) * size), size);
    auto bottom = min(int((heightmap_position.y + outside_radius) * size), size);

    auto average_height = op == "flatten" || op == "smooth" ?
        heightmap->AvgHeight(Conversions::ConvertIgn(heightmap_position), outside_radius) :
        0.0;

    for (auto y = top; y <= bottom; ++y)
        for (auto x = left; x <= right; ++x)
        {
            auto ts_x_dist = x / static_cast<double>(size) - heightmap_position.x;
            auto ts_y_dist = y / static_cast<double>(size)  - heightmap_position.y;
            auto dist = sqrt(ts_y_dist * ts_y_dist + ts_x_dist * ts_x_dist);

            auto inner_weight = 1.0;
            if (dist > inside_radius)
            {
                inner_weight = ignition::math::clamp(dist / outside_radius, 0.0, 1.0);
                inner_weight = 1.0 - (inner_weight * inner_weight);
            }

            auto added_height = inner_weight * weight;
            auto new_height = get_height_value(x, y);

            if (op == "raise")
                new_height += added_height;
            else if (op == "lower")
                new_height -= added_height;
            else if (op == "flatten")
            {
                if (new_height < average_height)
                    new_height += added_height;
                else
                    new_height -= added_height;
            }
            else if (op == "smooth")
            {
                if (new_height < average_height)
                    new_height += added_height;
                else
                    new_height -= added_height;
            }
            else
                gzerr << "Unknown terrain operation[" << op << "]" << endl;

            set_height_value(x, y, new_height);
        }
}