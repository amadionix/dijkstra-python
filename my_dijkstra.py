import sys
import unittest
import time
import heapq

class Graph:
    def __init__(self, num):
        self.adjList = {}                                     # u -> (v1, w1) -> (v2, w2) -> ...
        self.num_nodes = num   
        self.dist = []

    def add_edge(self, u, v, w):
        if u in self.adjList.keys():
            self.adjList[u].append((v, w))
        else:
            self.adjList[u] = [(v, w)]

    def show_graph(self):
        for u in sorted(self.adjList):
            print(u, '->', ' -> '.join(str("{}({})".format(v, w))
                                       for v, w in self.adjList[u]))

    def dijkstra_dec(self, src):
        Q = PriorityQueue()
        self.dist = [sys.maxsize] * (self.num_nodes + 1)
        self.dist[src] = 0
        Q.insert(src, 0)  
        while Q.heap:                                        
            heap_node = Q.extract_min()                      
            u = heap_node[0]
            self.dist[u] = heap_node[1]  
            if u not in self.adjList.keys():                 # sometimes I visit nodes without an entry 
                continue                                     # in the adjList -> skippo otherwise keyError
            for v, w in self.adjList[u]:
                if self.dist[u] + w < self.dist[v]:
                    if self.dist[v] == sys.maxsize:
                        Q.insert(v, self.dist[u] + w)
                    else:
                        Q.decrease_key(v, self.dist[u] + w)
                    self.dist[v] = self.dist[u] + w
        # show_distances(src, self.dist)

    def dijkstra_no_dec(self, src):
        Q = PriorityQueue()
        self.dist = [sys.maxsize] * (self.num_nodes + 1)
        self.dist[src] = 0
        Q.insert(src, 0)  
        while Q.heap:
            heap_node = Q.extract_min()        
            if heap_node[1] > self.dist[heap_node[0]]:       # if dist_new > dist_old, no upgrade needed
                continue
            u = heap_node[0]
            self.dist[u] = heap_node[1] 
            if u not in self.adjList.keys():                 # sometimes I visit nodes without an entry
                continue                                     # in the adjList -> skippo otherwise keyError
            for v, w in self.adjList[u]:
                if self.dist[u] + w < self.dist[v]:
                    Q.insert(v, self.dist[u] + w)
                    self.dist[v] = self.dist[u] + w
        # show_distances(src, self.dist)

    def dijkstra_no_dec_2(self, src):
        heap = [(0, src)]  
        self.dist = [sys.maxsize] * (self.num_nodes + 1)
        self.dist[src] = 0
        while heap:
            (cost, u) = heapq.heappop(heap)
            if cost > self.dist[u]:                          # if dist_new > dist_old, no upgrade needed
                continue
            self.dist[u] = cost
            if u not in self.adjList.keys():                 # sometimes I visit nodes without an entry
                continue    
            for v, w in self.adjList[u]:
                if self.dist[u] + w < self.dist[v]:
                    heapq.heappush(heap, (self.dist[u] + w, v))
                    self.dist[v] = self.dist[u] + w


class PriorityQueue:
    def __init__(self):
        self.heap = []
        self.pos = {}   

    def insert(self, graph_node, dist):
        self.pos[graph_node] = len(self.heap)
        self.heap.append([graph_node, dist])
        self.sift_up(len(self.heap) - 1)
            
    def is_empty(self):
        return len(self.heap) == 0

    def sift_up(self, child):
        if child == 0:
            return
        par = (child - 1) // 2
        if self.heap[par][1] > self.heap[child][1]:
            self.swap(par, child)
            self.sift_up(par)
     
    def swap(self, i, j):
        self.pos[self.heap[i][0]], self.pos[self.heap[j][0]] = j, i
        temp = self.heap[i]
        self.heap[i] = self.heap[j]
        self.heap[j] = temp

    def extract_min(self):
        self.swap(0, len(self.heap) - 1)
        min_heap = self.heap.pop()
        self.sift_down(0)
        del self.pos[min_heap[0]]
        return min_heap

    def sift_down(self, idx):
        lc = self.left(idx)
        rc = self.right(idx)
        if lc < len(self.heap) and self.heap[lc][1] < self.heap[idx][1]:
            smallest = lc
        else:
            smallest = idx
        if rc < len(self.heap) and self.heap[rc][1] < self.heap[smallest][1]:
            smallest = rc
        if smallest != idx:
            self.swap(idx, smallest)
            self.sift_down(smallest)

    def left(self, i):
        return 2 * i + 1

    def right(self, i):
        return 2 * i + 2        

    def decrease_key(self, graph_node, dist):
        idx = self.pos[graph_node]
        self.heap[idx] = [graph_node, dist]
        self.sift_up(idx)


def show_distances(src, dist_vect):
    print("Distance from node: {}".format(src))
    for u in range(len(dist_vect)):
        if dist_vect[u] == sys.maxsize:
            d = "not reached from src"
        else:
            d = dist_vect[u]
        print('Node {} has distance: {}'.format(u, d))


def get_graph(path):
    graph = {}
    with open(path) as f:
        vertices_num = int(f.readline())
        graph = Graph(vertices_num)
        edge = f.readline()
        while edge != "-1":
            ar = edge.split()
            u = int(ar[0])
            v = int(ar[1])
            w = float(ar[2])
            graph.add_edge(u, v, w)
            edge = f.readline()
    return graph


class Test_Dijkstra_Dec(unittest.TestCase):
    def graph_1_from_1(self):
        graph = get_graph('Randgraph/graph_1')
        graph.dijkstra_dec(1)
        graph_1_dist = [2.286603,0,sys.maxsize,sys.maxsize,sys.maxsize,0.781672,2.8084290000000003,
                        sys.maxsize,sys.maxsize,2.531511,2.471927,sys.maxsize,sys.maxsize,2.873569,
                        1.612932,sys.maxsize,3.025985,sys.maxsize,sys.maxsize,1.571332]
        self.assertEqual(graph.dist, graph_1_dist)

class Test_Dijkstra_No_Dec(unittest.TestCase):
    def graph_1_from_1(self):
        graph = get_graph('Randgraph/graph_1')
        graph.dijkstra_no_dec_2(1)
        graph_1_dist = [2.286603,0,sys.maxsize,sys.maxsize,sys.maxsize,0.781672,2.8084290000000003,
                        sys.maxsize,sys.maxsize,2.531511,2.471927,sys.maxsize,sys.maxsize,2.873569,
                        1.612932,sys.maxsize,3.025985,sys.maxsize,sys.maxsize,1.571332]
        self.assertEqual(graph.dist, graph_1_dist)


def get_graph_specs(path):
        edges_num = 0
        with open(path) as f:
            vertices_num = int(f.readline())
            edge = f.readline()
            while edge != "-1":
                edges_num += 1
                edge = f.readline()
        avg_degree = (edges_num * 2) / vertices_num
        return (avg_degree, vertices_num)


def display_experiment_results(graph_name, path):
    graph = get_graph(path)
    (avg_degree, vertices_num) = get_graph_specs(path)
    t = t2 = ratio = 0
    for i in [0,3,6,9]:                               # average results over
        start = time.time()                           # 4 different src points
        graph.dijkstra_no_dec_2(i)     
        t += time.time() - start
        start = time.time()
        graph.dijkstra_no_dec(i)
        t2 += time.time() - start
        ratio += t / t2
    t_avg = t / 4
    t2_avg = t2 / 4
    ratio_avg = ratio / 4
    print("{}, average degree: {}, vertices number: {}: \n with decrease_key: \
        {},\n without: \t\t    {}\n ratio: \t\t\t{}".format(graph_name, \
            avg_degree, vertices_num, t_avg*1000, t2_avg*1000, ratio_avg))


if __name__ == "__main__":
     # unittest.main()

    # # TRANCHE 1

    # # avg_d = 2,  n = 10^5
    # path = 'Randgraph/graph_1'
    # name = "Graph 1"
    # display_experiment_results(name, path)     

    # # avg_d = 2,  n = 10^6
    # path = 'Randgraph/graph_2'
    # name = "Graph 2"
    # display_experiment_results(name, path)

    # # avg_d = 2,  n = 10^7
    # path = 'Randgraph/graph_3'
    # name = "Graph 3"
    # display_experiment_results(name, path)

    # # avg_d = 3,  n = 10
    # path = 'Randgraph/graph_4'
    # name = "Graph 4"
    # display_experiment_results(name, path)

    # # avg_d = 3,  n = 10^3
    # path = 'Randgraph/graph_5'
    # name = "Graph 5"
    # display_experiment_results(name, path)

    # # avg_d = 3,  n = 10^5
    # path = 'Randgraph/graph_6'
    # name = "Graph 6"
    # display_experiment_results(name, path)

    # # avg_d = 3,  n = 10^7
    # path = 'Randgraph/graph_7'
    # name = "Graph 7"
    # display_experiment_results(name, path)


    # # TRANCHE 2

    # # avg_d = 3,  n = 3*(10^7)
    # path = 'Randgraph/graph_7b'
    # name = "Graph 7b"
    # display_experiment_results(name, path)


    # TRANCHE 3

    # avg_d = 8,  n = 10^3
    path = 'Randgraph/graph_8'
    name = "Graph 8"
    display_experiment_results(name, path)

    # avg_d = 8,  n = 10^5
    path = 'Randgraph/graph_9'
    name = "Graph 9"
    display_experiment_results(name, path)  

    # avg_d = 8,  n = 10^6
    path = 'Randgraph/graph_9b'
    name = "Graph 9b"
    display_experiment_results(name, path)  

    # avg_d = 4,  n = 10^7
    path = 'Randgraph/graph_10'
    name = "Graph 10"
    display_experiment_results(name, path)

    # TRANCHE 4

    # avg_d = 4,  n = 1.25*(10^7)
    # path = 'Randgraph/graph_10b'
    # name = "Graph 10b"
    # display_experiment_results(name, path)    

    # # avg_d = 5.5,  n = 2.6*(10^5)
    # path = 'Randgraph/USA-road-t.NY.gr'
    # name = "NY roads"
    # display_experiment_results(name, path)

    # # avg_d = 4.8,  n = 6.2*(10^6)
    # path = 'Randgraph/USA-road-t.W.gr'
    # name = "Western USA roads"
    # display_experiment_results(name, path)