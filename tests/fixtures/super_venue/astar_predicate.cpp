    bool Astar::insideLocalMap(const rog_map::Vec3i &id_g) const {
        rog_map::Vec3i delta = id_g - md_.local_map_center_id_g;
        if (fabs(delta.x()) > cfg_.map_size_i.x() ||
            fabs(delta.y()) > cfg_.map_size_i.y() ||
            fabs(delta.z()) > cfg_.map_size_i.z()) {
            return false;
        }
        return true;
    }

