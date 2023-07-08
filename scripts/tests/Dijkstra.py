def dijkstra(self, start_node, all_nodes):
        N = all_nodes.size()
        closed = set()
        start_node.distance = 0
        for _ in range(N):
            min_index = self.find_min(all_nodes, closed)
            current_node = all_nodes.nodes[min_index]
            closed.add(current_node)
            if current_node.has_free():
                path = self.make_path(current_node)
                self.cleanup(all_nodes)
                return path
            for neighbour in \
                list(current_node.get_neighbours().values()):
                if neighbour not in closed:
                    new_distance = current_node.distance + \
                        current_node.distance_to(neighbour)
                    if new_distance < neighbour.distance:
                        neighbour.distance = new_distance
                        neighbour.previous = current_node
                        
        raise Exception("No path found, No more paths to explore!!")