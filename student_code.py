from expand import expand
import heapq

def dequeue_priorityq(priority_queue):
    if not priority_queue:
        return None
    return heapq.heappop(priority_queue)

def a_star_search (dis_map, time_map, start, end):
    priority_queue = [(start, dis_map[start][end], 0, [start])]
    visited_n = set()
    ans_found = False

    while priority_queue:
        dequeue_index = dequeue_priorityq(priority_queue)
        curr_node = priority_queue[dequeue_index]
        
        del priority_queue[dequeue_index]
        if curr_node[0] in visited_n:
            continue
        
        visited_n.add(curr_node[0])
        if curr_node[0] == end:
            if ans_found and ans[1] > curr_node[1]:
                ans = curr_node
            else:
                ans = curr_node
                ans_found = True
        if ans_found and ans != curr_node and ans[1] <= curr_node[1]:
                    break
        if curr_node[0] != end:
            neighbour_nodes = expand(curr_node[0], time_map)
            for node in neighbour_nodes:
                if node not in visited_n:
                    priority_queue.append(
									(node, 
									curr_node[2] + time_map[curr_node[0]][node] + dis_map[node][end], 
									curr_node[2] + time_map[curr_node[0]][node],
									curr_node[3] + [node])
								)
    if ans_found:
       return ans[-1]

def dfs(time_map, start, visited_n=None):
    if visited_n is None:
        visited_n = set()
    visited_n.add(start)

    print(start)

    for next_node in time_map[start]:
        if next_node not in visited_n:
            dfs(time_map, next_node, visited_n)
    return visited_n

def depth_first_search(time_map, start, end):
    visited_n = set()

    path = dfs(time_map, start, visited_n)

    if end in path:
        return path
    else:
        return "No path found"
	

def breadth_first_search(time_map, start, end):
	visited_n = set()
	queue_bfs = [(start, start)]

	paths= {start: [[start]]}

	for node in queue_bfs:

		if node[1] in visited_n:
			continue
		else:
			visited_n.add(node[1])
			if node[1] in paths.keys():
				paths[node[1]].append(paths[node[0]][-1] + [node[1]])
			else:
				paths[node[1]] = [paths[node[0]][-1] + [node[1]]]

		if node[1] == end:
			
			return paths[node[1]][-1][1:]

		adj_nodes = expand(node[1], time_map)
		for adj in adj_nodes:
			queue_bfs.append((node[1], adj))
	

	return "No path found"