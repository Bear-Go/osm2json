from cgi import print_directory
from ctypes import wstring_at
import os
import sys
from sys import platform
import argparse
from collections import defaultdict
import sympy
from mpmath import degrees, radians
import copy
import math
import json

if platform == "linux" or platform == "linux2":
    try:
        import traci
        import traci.constants as tc
        import sumolib
        from sumolib.net import Connection
    except ImportError:
        if "SUMO_HOME" in os.environ:
            print(os.path.join(os.environ["SUMO_HOME"], "tools"))
            sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
            import traci
            import traci.constants as tc
            import sumolib
            from sumolib.net import Connection
        else:
            raise EnvironmentError("Please set SUMO_HOME environment variable or install traci as python module!")
elif platform =='darwin':
    os.environ['SUMO_HOME'] = "/Users/{0}/sumo/".format(os.environ.get('USER'))
    print(os.environ['SUMO_HOME'])
    try:
        import traci
        import traci.constants as tc
        import sumolib
        from sumolib.net import Connection
    except ImportError:
        if "SUMO_HOME" in os.environ:
            print(os.path.join(os.environ["SUMO_HOME"], "tools"))
            sys.path.append(
                os.path.join(os.environ["SUMO_HOME"], "tools")
            )
            import traci
            import traci.constants as tc
            import sumolib
            from sumolib.net import Connection
        else:
            raise EnvironmentError("Please set SUMO_HOME environment variable or install traci as python module!")
else:
    sys.exit("platform error")

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sumonet", type=str,default='atlanta_sumo.net.xml')
    parser.add_argument("--cityflownet", type=str,default='atlanta_cityflow.json')
    return parser.parse_args()

def get_direction_fron_connection(connection):
    _map = {
        Connection.LINKDIR_STRAIGHT: "go_straight",
        Connection.LINKDIR_TURN: "turn_u",
        Connection.LINKDIR_LEFT: "turn_left",
        Connection.LINKDIR_RIGHT: "turn_right",
        Connection.LINKDIR_PARTLEFT: "turn_left",
        Connection.LINKDIR_PARTRIGHT: "turn_right",
    }
    return _map[connection.getDirection()]

def process_edge(edge):
    lanes = []
    for inx,lane in enumerate(reversed(edge.getLanes())):
        outgoing_list = lane.getOutgoing()
        for outgoing in outgoing_list:
            new_lane = copy.copy(lane)
            direction = get_direction_fron_connection(outgoing)
            to_lane = outgoing.getToLane()
            new_lane._cityflow_lane_id = f'{lane.getID()}|{to_lane.getID()}|{direction}'
            new_lane._cityflow_lane_inx = inx
            new_lane._direction = direction
            lanes.append(new_lane)
        if len(outgoing_list) == 0:
            new_lane = copy.copy(lane)
            new_lane._cityflow_lane_id = f'{lane.getID()}'
            new_lane._cityflow_lane_inx = inx
            new_lane._direction = 'go_end'
            lanes.append(new_lane)
    edge._cityflow_lanes = lanes[::-1]
    return edge

def _cityflow_get_lane_index_in_edge_cor(lane, edge):
    for i, _lane in enumerate(edge._cityflow_lanes):
        if _lane._cityflow_lane_id == lane._cityflow_lane_id:
            return _lane._cityflow_lane_inx
    raise Exception('lane in edge not found')


def point_tuple_to_dict(point_tuple):
    return {"x": point_tuple[0], "y": point_tuple[1]}


def _is_node_virtual(node):
    n = node
    edges = [edge for edge in n.getIncoming() + n.getOutgoing()]
    ids = list(set([e.getFromNode().getID() for e in edges] + [e.getToNode().getID() for e in edges]))
    if len(ids)<=2:
        return True
    else:
        return False


def group_connections_by_start_end(connections):
    connection_group_result = defaultdict(list)
    for connection in connections:
        start_road = connection.getFrom()
        end_road = connection.getTo()
        direction = get_direction_fron_connection(connection)
        key = "{}|{}|{}".format(start_road.getID(), end_road.getID(), direction)
        connection_group_result[key].append(connection)
    return connection_group_result


def process_intersection_simple_phase(intersection):
    if intersection['virtual']:
        return intersection

    all_green = {
        "time": 30,
        "availableRoadLinks": intersection['trafficLight']['roadLinkIndices']
    }
    all_red = {
        "time": 30,
        "availableRoadLinks": []
    }
    lightphases = [all_green]
    intersection['trafficLight']['lightphases'] = lightphases
    return intersection


all_phase_dict = {}
node_outgoing_dict = {}
tot_roads = []

def node_to_intersection(node,tls_dict,edge_dict):
    node_type = node.getType()
    node_coord = node.getCoord()
    node_x = node_coord[0]
    node_y = node_coord[1]
    intersection = {
        "id": node.getID(),
        "point": {"x": node_x, "y": node_y},
        "width": 0,
        "roads": [edge.getID() for edge in node.getIncoming() + node.getOutgoing()],
        "roadLinks": [],
        "trafficLight": {
            "roadLinkIndices": [],
            "lightphases": []
        },
        "virtual": _is_node_virtual(node)
    }


    connections_group = group_connections_by_start_end(node.getConnections())
    roadLinks = intersection['roadLinks']
    for k, v in connections_group.items():
        connection_template = v[0]
        start_road = connection_template.getFrom()
        end_road = connection_template.getTo()
        raw_roadlink_type = get_direction_fron_connection(connection_template)
        roadLink = {
            "type": raw_roadlink_type,
            "startRoad": start_road.getID(),
            "endRoad": end_road.getID(),
            "direction": 0,
            "laneLinks": []
        }
        if roadLink["type"] == "turn_u":
            continue


        for start_lane in reversed(start_road._cityflow_lanes):
            if start_lane._direction != raw_roadlink_type:
                continue
            for end_inx,end_lane in enumerate(reversed(end_road._lanes)):
                start_point = start_lane.getShape()[-1]
                start_point = point_tuple_to_dict(start_point)
                end_point = end_lane.getShape()[0]
                end_point = point_tuple_to_dict(end_point)
                path = {
                    "startLaneIndex": _cityflow_get_lane_index_in_edge_cor(start_lane, start_road),
                    "endLaneIndex": end_inx,
                    "points": [start_point, end_point]
                }
                roadLink["laneLinks"].append(path)
        roadLinks.append(roadLink)


    for i, _ in enumerate(intersection["roadLinks"]):
        intersection["trafficLight"]["roadLinkIndices"].append(i)

    if node_type in ['dead_end']:
        pass
    if node_type in ['priority']:
        pass
    if node_type in ['right_before_left']:
        pass
    if node_type in ['dead_end','priority','right_before_left']:
        intersection = process_intersection_simple_phase(intersection)


    if node_type in ['traffic_light']:
        print(node.getID())
        for edge in node.getIncoming() + node.getOutgoing():
            tot_roads.append(edge.getID())
        
        all_phase = []
        nodeid = node.getID()
        all_phase_dict[nodeid] = []
        G_to_lane_dict = {}
        for connec in tls_dict[nodeid]._connections:
            G_to_lane_dict[connec[-1]] = connec[0].getID()


        turn_right_index_list = []
        for index, roadLink in enumerate(roadLinks):
            if (roadLink["type"] == "turn_right"):
                turn_right_index_list += [index]
            
        ewwe_turn_left_index_list = []
        snns_turn_left_index_list = []
        for index, roadLink in enumerate(roadLinks):
            if (roadLink["type"] == "turn_left"):
                x_diff = roadLink["laneLinks"][0]["points"][0]["x"] - roadLink["laneLinks"][0]["points"][1]["x"]
                y_diff = roadLink["laneLinks"][0]["points"][0]["y"] - roadLink["laneLinks"][0]["points"][1]["y"]
                if (x_diff * y_diff > 0):
                    ewwe_turn_left_index_list += [index]
                else:
                    snns_turn_left_index_list += [index]

        ewwe_straight_index_list = []
        snns_straight_index_list = []
        for index, roadLink in enumerate(roadLinks):
            if (roadLink["type"] == "go_straight"):
                x_diff = abs(roadLink["laneLinks"][0]["points"][0]["x"] - roadLink["laneLinks"][0]["points"][1]["x"])
                y_diff = abs(roadLink["laneLinks"][0]["points"][0]["y"] - roadLink["laneLinks"][0]["points"][1]["y"])
                if (x_diff > y_diff):
                    ewwe_straight_index_list += [index]
                else:
                    snns_straight_index_list += [index]

        we_index_list = []
        ew_index_list = []
        ns_index_list = []
        sn_index_list = []
        for index, roadLink in enumerate(roadLinks):
            x_diff = roadLink["laneLinks"][0]["points"][1]["x"] - roadLink["laneLinks"][0]["points"][0]["x"]
            y_diff = roadLink["laneLinks"][0]["points"][1]["y"] - roadLink["laneLinks"][0]["points"][0]["y"]
            if (roadLink["type"] == "go_straight"):
                if abs(x_diff) > abs(y_diff):
                    if x_diff > 0:
                        we_index_list += [index]
                    else:
                        ew_index_list += [index]
                else:
                    if y_diff > 0:
                        sn_index_list += [index]
                    else:
                        ns_index_list += [index]
            elif (roadLink["type"] == "turn_left"):
                if (x_diff > 0 and y_diff > 0):
                    we_index_list += [index]
                elif (x_diff > 0 and y_diff < 0):
                    ns_index_list += [index]
                elif (x_diff < 0 and y_diff > 0):
                    sn_index_list += [index]
                else:
                    ew_index_list += [index]


        phase_dict = {
            'availableRoadLinks': list(set(turn_right_index_list)),
            'time': 5
        }
        all_phase.append(phase_dict)
        phase_dict = {
            'availableRoadLinks': list(set(ewwe_straight_index_list + turn_right_index_list)),
            'time': 30
        }
        all_phase.append(phase_dict)
        phase_dict = {
            'availableRoadLinks': list(set(snns_straight_index_list + turn_right_index_list)),
            'time': 30
        }
        all_phase.append(phase_dict)
        phase_dict = {
            'availableRoadLinks': list(set(ewwe_turn_left_index_list + turn_right_index_list)),
            'time': 30
        }
        all_phase.append(phase_dict)
        phase_dict = {
            'availableRoadLinks': list(set(snns_turn_left_index_list + turn_right_index_list)),
            'time': 30
        }
        all_phase.append(phase_dict)
        
        phase_dict = {
            'availableRoadLinks': list(set(we_index_list + turn_right_index_list)),
            'time': 30
        }
        all_phase.append(phase_dict)
        phase_dict = {
            'availableRoadLinks': list(set(ew_index_list + turn_right_index_list)),
            'time': 30
        }
        all_phase.append(phase_dict)
        phase_dict = {
            'availableRoadLinks': list(set(ns_index_list + turn_right_index_list)),
            'time': 30
        }
        all_phase.append(phase_dict)
        phase_dict = {
            'availableRoadLinks': list(set(sn_index_list + turn_right_index_list)),
            'time': 30
        }
        all_phase.append(phase_dict)
        intersection["trafficLight"]["lightphases"] = all_phase

        outgoing_lane_list = []
        edge_list_ = [edge_.getID() for edge_ in node.getOutgoing()]
        for edge in edge_list_:
            for i in range(len(edge_dict[edge])):
                outgoing_lane_list.append(edge+'_'+str(i))
        node_outgoing_dict[nodeid] = outgoing_lane_list

        exiting_lane_list = []
        for edge in node.getOutgoing():
            exiting_lane_list.extend([lane.getID() for lane in edge.getLanes()])

    return intersection

def get_final_intersections(net,tls_dict,edge_dict):

    final_intersections = []
    net_nodes = net.getNodes()
    net_nodes_sorted = sorted(net_nodes,key=lambda n:n.getID())
    nodes = [(index,node) for index,node in enumerate(net_nodes_sorted)]
    nodes = nodes[:]
    for obj in nodes:

        index = obj[0]
        node = obj[1]

        intersection = node_to_intersection(node,tls_dict,edge_dict)
        if (intersection["roads"] != []):
            if (intersection["roadLinks"] == [] \
                or intersection["trafficLight"]["roadLinkIndices"] == [] \
                or len(intersection["trafficLight"]["roadLinkIndices"]) <= 1 \
                or len(intersection["trafficLight"]["lightphases"]) != 9):
                intersection["virtual"] = bool(1)
            final_intersections.append(intersection)

    return final_intersections

def get_final_roads(net):
    edges = net.getEdges()
    final_roads = []
    for edge in edges:
        start_intersection = edge.getFromNode()
        start_coord = start_intersection.getCoord()
        end_intersection = edge.getToNode()
        end_coord = end_intersection.getCoord()
        road = {
            "id": edge.getID(),
            "points": [
                {
                    "x": start_coord[0],
                    "y": start_coord[1],
                },
                {
                    "x": end_coord[0],
                    "y": end_coord[1],
                }
            ],
            "lanes": [
            ],
            "startIntersection": start_intersection.getID(),
            "endIntersection": end_intersection.getID(),
        }
        lane_template = {
            "width": 4,
            "maxSpeed": 11.111
        }
        for _v in edge._lanes:
            road["lanes"].append(lane_template)
        final_roads.append(road)
    return final_roads



def main(args):
    print("Converting...",args.sumonet)
    net = sumolib.net.readNet(os.path.join(os.getcwd(),args.sumonet), withPrograms=True)

    for edge in net.getEdges():
        process_edge(edge)

    tls_dict = {}
    for tls in net.getTrafficLights():
        tls_dict[tls.getID()] = tls

    print('Have '+str(len(tls_dict))+" traffic lights")
    edge_dict = {}
    for edge_ in net.getEdges():
        edge_dict[edge_.getID()] = edge_._lanes

    final_intersections = get_final_intersections(net,tls_dict,edge_dict)

    for intersection in final_intersections:
        if intersection['virtual']:
            intersection['roadLinks'] = []

    final_roads = get_final_roads(net)

    result = {
        "intersections": final_intersections,
        "roads": final_roads
    }

    f = open(args.cityflownet, 'w')
    json.dump(result, f, indent=2)
    f.close()


if __name__ == '__main__':
    args = parse_args()
    main(args)
    print("Convert successfully!")
