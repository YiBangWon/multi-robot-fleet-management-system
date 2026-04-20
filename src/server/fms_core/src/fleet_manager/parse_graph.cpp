// Graph parser for loading topological maps and annotated lane networks from YAML.

#include <fleet_manager/parse_graph.hpp>


rmf::GraphRMF rmf::parse_graph(const std::string& graph_file){

    const YAML::Node graph_config = YAML::LoadFile(graph_file);
    if (!graph_config) throw std::runtime_error("error1");
    const YAML::Node levels = graph_config["levels"];
    if (!levels) throw std::runtime_error("error2");
    if (!levels.IsMap()) throw std::runtime_error("error3");

    rmf::GraphRMF graph;
    std::size_t vnum = 0;  // To increment lane endpoint ids

    for (const auto& level : levels){

        const std::string& map_name = level.first.as<std::string>();
        std::size_t vnum_temp = 0;

        const YAML::Node& vertices = level.second["vertices"];
        std::size_t iter_wp = 0;
        for (const auto& vertex : vertices){

            const Eigen::Vector2d location{vertex[0].as<double>(), vertex[1].as<double>()};

            rmf::GraphRMF::Node wp;
            wp.index = iter_wp; iter_wp++;
            wp.map = map_name;
            wp.location = location;

            const YAML::Node& options = vertex[2];
            const YAML::Node& name_option = options["name"];
            if (name_option){
                const std::string& name = name_option.as<std::string>();
                if (!name.empty()){
                    wp.name = name;
                }
            }
            vnum_temp ++;

            wp.zone = -1;
            const YAML::Node& zone_option = options["zone"];
            if (zone_option){
                const int zone = zone_option.as<int>();
                wp.zone = zone;
            }
            graph.nodes.push_back(wp);
        }

        std::size_t iter_ln;
        const YAML::Node& lanes = level.second["lanes"];
        for (const auto& lane : lanes){
            const YAML::Node& options = lane[2];
            std::size_t begin = lane[0].as<std::size_t>() + vnum;
            std::size_t end = lane[1].as<std::size_t>() + vnum;

            rmf::GraphRMF::Lane ln;
            ln.index = iter_ln; iter_ln++;
            ln.entry_node = lane[0].as<std::size_t>();
            ln.exit_node = lane[1].as<std::size_t>();

            graph.lanes.push_back(ln);
        }
        vnum += vnum_temp;
    }

    return graph;
    // return rmf::GraphRMF();
}


void rmf::GraphRMF::save_graph(std::string file_name){

    double scale = 2.0;
    std::ofstream out(file_name);
    out << "building_name: building" << std::endl;
    out << "levels:"                 << std::endl;
    out << "  L1:"                   << std::endl;
    out << "    lanes:"              << std::endl;
    for (int i = 0; i < lanes.size(); i++){
        out << "    - - " << lanes[i].entry_node << std::endl;
        out << "      - " << lanes[i].exit_node  << std::endl;
        out << "      - {}"                      << std::endl;
    }
    out << "    vertices:"              << std::endl;
    for (int i = 0; i < nodes.size(); i++){
        out << "    - - " << nodes[i].location[0] * scale << std::endl;
        out << "      - " << nodes[i].location[1] * scale << std::endl;
        out << "      - {name: '" << nodes[i].name << "', zone: " << nodes[i].zone << "}" << std::endl;
    }
    out.close();

    int iter = 0;
    for (int i = 0; i < nodes.size(); i++){
        if (nodes[i].name.find("ChargingStation") != std::string::npos){
            double loc_x = nodes[i].location[0] * scale;
            double loc_y = nodes[i].location[1] * scale;
            std::cout << "pioneer2dx_no_sonar" << std::endl;
            std::cout << "(" << std::endl;
            std::cout << "# can refer to the robot by this name" << std::endl;
            std::cout << "name \"robot_" << iter << "\"" << std::endl;
            std::cout << "pose [ " << loc_x << " " << loc_y <<" 0 0 ]" << std::endl;
            std::cout << ")" << std::endl;
            std::cout << std::endl;
            iter++;
        }
    }
}
