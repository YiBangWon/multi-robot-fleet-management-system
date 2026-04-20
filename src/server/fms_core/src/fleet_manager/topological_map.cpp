// Topological map utilities for node lookup, connectivity, and geometric queries.

#include <fleet_manager/topological_map.h>

namespace FMS
{

/* Load topological map */
bool TopologicalMap::loadTopologicalMapFile(std::string filename_graph, bool is_directed_graph_, bool remove_stations_, double unit_dist_){

    // Set map parameters
    is_directed_graph = is_directed_graph_;     // True: directed graph
    remove_stations = remove_stations_;         // remove station nodes
    unit_dist = unit_dist_;                     // Minimum distance b/w nodes

    std::size_t pos = filename_graph.find("topological_map.yaml");
    std::string dir_name = filename_graph.substr (0, pos);
    std::string file_name_nodes = dir_name + "extended_nodes.yaml";     // Extended nodes in topological map
    std::string file_name_lanes = dir_name + "extended_lanes.yaml";     // Extended edges in topological map
    bool files_exist = std::filesystem::exists(file_name_nodes) && std::filesystem::exists(file_name_lanes);
    if (files_exist){                                                   // If the extended nodes and edges are already computed,
        std::cout << "Load Extended Topological Map" << std::endl;      // load those files
        loadExtendedTopologicalMap(file_name_nodes, file_name_lanes);
    }
    else {
        std::cout << "Generate Extended Topological Map" << std::endl;  // If we only have an original topological map file
        extendTopologicalMap(filename_graph);                           // Generate the extended nodes and edges from the original topological map
        saveExtendedTopologicalMap(file_name_nodes, file_name_lanes);   // Save the extended topological map
    }
    // Set the node labels
    setNodeStation();

    return true;
}

/* Generate the extended nodes and edges from the original topological map */
bool TopologicalMap::extendTopologicalMap(std::string filename_graph){

    // 1) Read rmf graph file
    graph = rmf::parse_graph(filename_graph);

    // 2) Delete duplicated lanes
    std::vector<bool> select(graph.lanes.size(), false);
    std::vector<bool> is_bidirect(graph.lanes.size(), false);
    for (size_t i = 0; i < graph.lanes.size(); i++){
        if (select[i]) continue;
        std::size_t entry_node = graph.lanes[i].entry_node;
        std::size_t exit_node  = graph.lanes[i].exit_node;
        for (size_t j = 0 ; j < graph.lanes.size() ; j++){
            if (graph.lanes[j].entry_node == exit_node &&   // check if they are duplicated
                graph.lanes[j].exit_node  == entry_node){
                    select[j] = true;
                    is_bidirect[i] = true;
                    is_bidirect[j] = true;
                }
        }
    }
    std::vector<rmf::GraphRMF::Lane> lanes_rmf; // filtered graph lanes
    for (size_t i = 0 ; i < graph.lanes.size() ; i++){
        if (select[i]) continue;
        if (!is_directed_graph || is_bidirect[i])
            graph.lanes[i].bidirect = true;
        else
            graph.lanes[i].bidirect = false;
        lanes_rmf.push_back(graph.lanes[i]);
    }

    // 3) Expand graph nodes according to unit distance (unit_dist)
    std::vector<Eigen::Vector2d> entry_nodes;       // xy-positions of expanded entry nodes
    std::vector<Eigen::Vector2d> exit_nodes;        // xy-positions of expanded exit nodes
    std::vector<bool> lanes_bidirect;               // bi-direction for each lane
    for (size_t i = 0 ; i < lanes_rmf.size() ; i++){
        std::size_t entry_idx = lanes_rmf[i].entry_node;
        std::size_t exit_idx  = lanes_rmf[i].exit_node;
        bool bidirect = lanes_rmf[i].bidirect;
        Eigen::Vector2d entry_(graph.nodes[entry_idx].location[0], graph.nodes[entry_idx].location[1]);
        Eigen::Vector2d exit_(graph.nodes[exit_idx].location[0], graph.nodes[exit_idx].location[1]);
        double dist = (entry_ - exit_).norm();      // distance b/w entry and exit nodes
        double div = std::round(dist/unit_dist);    // num of expanded lanes
        double div_dist = dist/div;                 // unit distance of each lane
        Eigen::Vector2d dir = exit_ - entry_;
        Eigen::Vector2d dir_norm = dir/dir.norm();  // normaized direction from entry to exit nodes

        if (div <= 4){                              // if it has only one lane, just input original nodes
            entry_nodes.push_back(entry_);
            exit_nodes.push_back(exit_);
            lanes_bidirect.push_back(bidirect);
        }
        else {
            for (double s = 1.0 ; s <= div ; s = s + 1.0){  // for each step s, expand nodes from entry to exit
                entry_nodes.push_back( entry_ + (s - 1.0)*div_dist*dir_norm );
                exit_nodes.push_back( entry_ + s*div_dist*dir_norm );
                lanes_bidirect.push_back(bidirect);
            }
        }
    }

    // 4) Set the unique nodes from expanded graph
    std::vector<Eigen::Vector2d> nodes_loc;
    for (size_t i = 0 ; i < entry_nodes.size() ; i++){
        Eigen::Vector2d en_ = entry_nodes[i];
        Eigen::Vector2d ex_ = exit_nodes[i];
        bool found_en = false;
        bool found_ex = false;
        for (size_t j = 0 ; j < nodes_loc.size() ; j++){    // check if the entry node is alrady inserted
            if (isEqual(en_, nodes_loc[j])) {found_en = true; break;}
        }
        for (size_t j = 0 ; j < nodes_loc.size() ; j++){    // check if the exit node is alrady inserted
            if (isEqual(ex_, nodes_loc[j])) {found_ex = true; break;}
        }
        if (!found_en) {nodes_loc.push_back(en_);}   // insert the entry node
        if (!found_ex) {nodes_loc.push_back(ex_);}   // insert the exit node
    }

    // 5) Compose lanes from node ids
    std::vector<std::size_t> entry_ids;
    std::vector<std::size_t> exit_ids;
    for (std::size_t i = 0 ; i < entry_nodes.size() ; i++){
        Eigen::Vector2d et_ = entry_nodes[i];
        Eigen::Vector2d ex_ = exit_nodes[i];
        for (std::size_t j = 0 ; j < nodes_loc.size() ; j++){
            Eigen::Vector2d n_ = nodes_loc[j];
            if (isEqual(n_, et_))
                entry_ids.push_back(j);
            else if (isEqual(n_, ex_))
                exit_ids.push_back(j);
        }
    }

    // 6) Set the nodes and lanes
    for (int i = 0; i < nodes_loc.size(); i++){
        TopologicalMap::Node node;
        node.loc    = nodes_loc[i];
        nodes.push_back(node); // Set Nodes
    }

    for (int i = 0; i < entry_ids.size(); i++){
        // Reject duplicated lanes
        bool found = false;
        for (int j = 0; j < lanes.size(); j++){
            if (entry_ids[i] == lanes[j].entry && exit_ids[i] == lanes[j].exit){
                found = true;   break;
            }
        }
        if (found) continue;

        TopologicalMap::Lane lane;
        lane.entry    = entry_ids[i];
        lane.exit     = exit_ids[i];
        lane.bidirect = lanes_bidirect[i];
        lanes.push_back(lane);
    }
    num_nodes = nodes.size();
    num_lanes = lanes.size();

    // 7)  Set the node names
    setNodeName();

    return true;
}


/* Get the node id from pose location */
int TopologicalMap::getNodeId(Eigen::Vector3d pose){
    Eigen::Vector2d loc = pose.head(2);
    return getNodeId(loc);
}

/* Get the node id from pose location */
int TopologicalMap::getNodeId(Eigen::Vector2d loc){
    // Simply find the closet node
    int node_id = 0;
    double dist_min  = 9999;
    for (int i = 0 ; i < num_nodes ; i++) {
        Eigen::Vector2d node = nodes[i].loc;
        double dist = (loc - node).norm();
        if (dist < dist_min) {
            dist_min = dist;
            node_id = i;
        }
    }

    return node_id;
}

/* Get the nearest node id in global path from pose location */
int TopologicalMap::getRouteNodeId(int robot_id, std::vector<std::vector<Eigen::Vector3d>> global_path, Eigen::Vector2d loc)
{
    // Simply find the closet node in path
    int node_id = 0;
    double dist_min  = 9999;
    for (int i = 0 ; i < global_path[robot_id].size(); i++) {
        Eigen::Vector2d node(global_path[robot_id][i][0], global_path[robot_id][i][1]);
        double dist = (loc - node).norm();
        if (dist < dist_min) {
            dist_min = dist;
            node_id = global_path[robot_id][i][2];
        }
    }
    return node_id;
}

/* Get the station id from a node name */
int TopologicalMap::getStationId(std::string node_name){
    for (int i = 0 ; i < stations.size() ; i++) {
        if (node_name == stations[i].first)
            return i;
    }
    return -1;
}

/* Set the node name */
void TopologicalMap::setNodeName(){
    for (int i = 0; i < graph.nodes.size(); i++) {
        if (graph.nodes[i].name.empty()) continue;
        Eigen::Vector2d location = graph.nodes[i].location;
        std::string name = graph.nodes[i].name;
        for (int j = 0; j < num_nodes; j++){
            if (isEqual(nodes[j].loc, location)){
                nodes[j].name = name;
                break;
            }
        }
    }
}


/* For each robot, set the station node from xy_location*/
void TopologicalMap::setNodeStation(){

    std::vector<int> station_nodes;
    std::vector<Eigen::Vector2d> docks_loc;
    std::vector<int> station_count(5,0);
    for (int i = 0; i < num_nodes; i++) {
        /*************************************************************/
        /*   Get the labels                                          */
        /*   1) Inbound, 2) Outbound,        3) I/O Bound,           */
        /*   4) Rack,    5) ChargingStation, 6)Dock                  */
        /*************************************************************/
        std::string station_name;
        if (nodes[i].name.find("Inbound") != std::string::npos)               station_name = "I" + nodes[i].name.substr(7);
        else if (nodes[i].name.find("Outbound") != std::string::npos)         station_name = "O" + nodes[i].name.substr(8);
        else if (nodes[i].name.find("Inoutbound") != std::string::npos)       station_name = "IO" + nodes[i].name.substr(10);
        else if (nodes[i].name.find("Rack") != std::string::npos)             station_name = "R" + nodes[i].name.substr(4);
        else if (nodes[i].name.find("ChargingStation") != std::string::npos)  station_name = "C" + nodes[i].name.substr(15);
        else if (nodes[i].name.find("Dock") != std::string::npos)             station_name = "D" + nodes[i].name.substr(4);
        else if (nodes[i].name.find("Waypoint") != std::string::npos)         station_name = "W" + nodes[i].name.substr(8);
        else                                                                  continue;

        Eigen::Vector2d loc = nodes[i].loc;
        stations.push_back(std::make_pair(station_name, loc));

        if (station_name.find("C") != std::string::npos){
            charging_station_index.push_back(stations.size()-1);
            nodes[i].is_waypoint = true;
        }

        if (station_name.find("D") != std::string::npos || station_name.find("C") != std::string::npos){
            docks_loc.push_back(loc);
            nodes[i].is_waypoint = true;
        }
        else {
            station_nodes.push_back(getNodeId(loc));
            if (station_name.find("W") != std::string::npos || station_name.find("I") != std::string::npos
                || station_name.find("O") != std::string::npos|| station_name.find("IO") != std::string::npos|| station_name.find("R") != std::string::npos){
                nodes[i].is_waypoint = true;
            }
        }
    }

    if (remove_stations)
        removeNodes(station_nodes); // Remove station node (except ChargingStation & Dock) in Topological map

    // Set dock nodes
    for (int i = 0; i < docks_loc.size(); i++){
        nodes_dock.push_back( getNodeId(docks_loc[i]) );
    }

}

/* Remove station (I/O Bound and Rack) Nodes */
void TopologicalMap::removeNodes(std::vector<int> station_nodes){

    Eigen::Vector2d del_node(9999, 9999);
    for (int i = 0; i < station_nodes.size(); i++){
        nodes[station_nodes[i]].loc = del_node;
    }
    int iter = 0;
    std::vector<int> node_new_index;
    std::vector<TopologicalMap::Node> node_new;
    for (int i = 0; i < nodes.size(); i++){
        if (nodes[i].loc == del_node){
            node_new_index.push_back(-1);
        }
        else {
            node_new_index.push_back(iter);
            node_new.push_back(nodes[i]);
            iter++;
        }
    }
    std::vector<TopologicalMap::Lane> lanes_new;
    for (int i = 0; i < lanes.size(); i++){
        int en = lanes[i].entry;
        int ex = lanes[i].exit;
        int en_n = node_new_index[en];
        int ex_n = node_new_index[ex];
        if (en_n >= 0 && ex_n >= 0){
            TopologicalMap::Lane lane = lanes[i];
            lane.entry = en_n;
            lane.exit = ex_n;
            lanes_new.push_back(lane);
        }
    }
    nodes.clear();
    lanes.clear();
    nodes = node_new;
    lanes = lanes_new;
    num_nodes = nodes.size();
    num_lanes = lanes.size();
}


/* Disable nodes in bounding box region */
void TopologicalMap::disableRegion(Eigen::Vector2d bbx_min, Eigen::Vector2d bbx_max){
    for (int i = 0 ; i < num_nodes ; i++) {
        Eigen::Vector2d node = nodes[i].loc;
        if (bbx_min[0] <= node[0] && bbx_max[0] >= node[0] &&
            bbx_min[1] <= node[1] && bbx_max[1] >= node[1]){
                nodes[i].disabled = true;
            }
    }

    for (int i = 0 ; i < num_lanes ; i++){
        if (nodes[lanes[i].entry].disabled || nodes[lanes[i].exit].disabled){
            lanes[i].disabled = true;
        }
    }
}

/* Assign duck and charging station */
void TopologicalMap::assignDuckAndChargingStation(std::vector<Eigen::Vector3d> poses){

    int num_duck = nodes_dock.size();
    int num_robot = poses.size();
    std::vector<std::pair<int, Eigen::Vector2d>> ducks(num_duck);   // index, location
    for (int i = 0; i < num_duck; i++){
        ducks[i].first = i;
        ducks[i].second = nodes[nodes_dock[i]].loc;
    }

    for (int i = 0; i < num_robot; i++){
        int min_index = 0;
        double min_dist = 9999;
        for (int j = i; j < num_duck; j++){
            double dist = (poses[i].head(2) - ducks[j].second).norm();
            if (dist < min_dist){
                min_index = j;
                min_dist = dist;
            }
        }
        // Swap
        std::pair<int, Eigen::Vector2d> temp;
        temp = ducks[i];
        ducks[i] = ducks[min_index];
        ducks[min_index] = temp;
    }
    std::vector<int> charging_station_index_t(num_duck);
    std::vector<int> nodes_dock_t(num_duck);
    for (int i = 0; i < num_duck; i++){
        charging_station_index_t[i] = charging_station_index[ducks[i].first];
        nodes_dock_t[i] = nodes_dock[ducks[i].first];
    }
    charging_station_index = charging_station_index_t;
    nodes_dock = nodes_dock_t;
}

/* Enable all nodes */
void TopologicalMap::enableAllNodes(){
    for (int i = 0 ; i < num_nodes ; i++) {
        nodes[i].disabled = false;
    }
    for (int i = 0 ; i < num_lanes ; i++){
        lanes[i].disabled = false;
    }
}

/* Is tow location equal?  */
bool TopologicalMap::isEqual(Eigen::Vector2d loc1, Eigen::Vector2d loc2){
    double dist = (loc1 - loc2).norm();
    if (dist < 0.01) { return true;  }
    else             { return false; }
}

/* Save the extended topolgical map  */
void TopologicalMap::saveExtendedTopologicalMap(std::string file_name_nodes, std::string file_name_lanes){
    YAML::Emitter nodes_out;
    std::ofstream nodes_fout;
    nodes_fout.open(file_name_nodes.c_str());
    for (int i = 0; i < num_nodes; i++){
        nodes_out << YAML::BeginMap;
        nodes_out << YAML::Key << "loc_x";
        nodes_out << YAML::Value << nodes[i].loc[0];
        nodes_out << YAML::Key << "loc_y";
        nodes_out << YAML::Value << nodes[i].loc[1];
        nodes_out << YAML::Key << "name";
        nodes_out << YAML::Value << nodes[i].name;
        nodes_out << YAML::EndMap;
    }
    nodes_fout << nodes_out.c_str();
    nodes_fout.close();

    YAML::Emitter lanes_out;
    std::ofstream lanes_fout;
    lanes_fout.open(file_name_lanes.c_str());
    for (int i = 0; i < num_lanes; i++){
        lanes_out << YAML::BeginMap;
        lanes_out << YAML::Key << "entry";
        lanes_out << YAML::Value << lanes[i].entry;
        lanes_out << YAML::Key << "exit";
        lanes_out << YAML::Value << lanes[i].exit;
        lanes_out << YAML::Key << "bidirect";
        lanes_out << YAML::Value << lanes[i].bidirect;
        lanes_out << YAML::EndMap;
    }
    lanes_fout << lanes_out.c_str();
    lanes_fout.close();
}

/* Load the extended topolgical map  */
void TopologicalMap::loadExtendedTopologicalMap(std::string file_name_nodes, std::string file_name_lanes){
    std::vector<YAML::Node> yaml_nodes = YAML::LoadAllFromFile(file_name_nodes);
    for (int i = 0; i < yaml_nodes.size(); i++){
       Node node;
       node.loc[0] = yaml_nodes[i]["loc_x"].as<double>();
       node.loc[1] = yaml_nodes[i]["loc_y"].as<double>();
       node.name   = yaml_nodes[i]["name"].as<std::string>();
       nodes.push_back(node);
    }
    num_nodes = nodes.size();

    std::vector<YAML::Node> yaml_lanes = YAML::LoadAllFromFile(file_name_lanes);
    for (int i = 0; i < yaml_lanes.size(); i++){
       Lane lane;
       lane.entry      = yaml_lanes[i]["entry"].as<int>();
       lane.exit       = yaml_lanes[i]["exit"].as<int>();
       lane.bidirect   = yaml_lanes[i]["bidirect"].as<bool>();
       lanes.push_back(lane);
    }
    num_lanes = lanes.size();
}

void TopologicalMap::setZone(){

    Eigen::Vector4d bbx_zone_1(23.0, 1.0, 41.0, 23.0);  // [min_x, min_y, max_x, max_y]
    Eigen::Vector4d bbx_zone_2(1.0, 1.0, 19.0, 23.0);   // [min_x, min_y, max_x, max_y]
    std::vector<Eigen::Vector4d> bbx_zones;
    bbx_zones.push_back(bbx_zone_1);
    bbx_zones.push_back(bbx_zone_2);
    for (int i = 0; i < bbx_zones.size(); i++){
        Eigen::Vector4d bbx = bbx_zones[i];
        for (int j = 0; j < nodes.size(); j++){
            Eigen::Vector2d loc = nodes[j].loc;
            if (loc[0] > bbx[0] && loc[0] < bbx[2] && loc[1] > bbx[1] && loc[1] < bbx[3]){
                nodes[j].zone = i;
            }
        }
    }
    /*
    for (int j = 0; j < nodes.size(); j++){
        Eigen::Vector2d loc = nodes[j].loc;
        std::cout << loc[0] << ", " << loc[1] << ", " << nodes[j].zone << std::endl;
    }
    */
}

}
