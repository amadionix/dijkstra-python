# dijkstra-python
Dijsktra's algorithm in Python, using a Binary Heap to implement the Priority Queue and covering both the version that uses the decrease-key method and the one that doesn't use it. It also contains an implementation of the no-decrease-key version based on the [heapq](https://docs.python.org/3.7/library/heapq.html) library.

Project Work for the course in Combinatorial Optimization, MSc Computer Science, University of Milan, AY 2019/2020.

Repo structure:

-in the .py file you have all the source code: including implementation of two variants of Dijkstra's algorithm, a Binary Heap of tuples, an Adjacency List implementation of a graph and some methods used to test and compare performances of the two Dijkstra's implementations provided.

-in the test-graphs folder I added one graph you can test the algorithms with. 
If you want to generate your own graphs or download real routing graphs from US roard network, I suggest you to check this page out: [http://users.diag.uniroma1.it/challenge9/download.shtml](http://users.diag.uniroma1.it/challenge9/download.shtml)

-the .pdf file contains the report of the project work: it includes a theoretical explaination of the topic, a walk-through of our experiments and some personal final considerations
