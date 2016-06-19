CC := g++
CXX_FlAGS := -Wall -Wextra -O0 -std=c++11
SRCS := descartes_graph.cpp main.cpp dijkstras_search.cpp
BIN_NAME := graph

.PHONY all: graph

graph:
	$(CC) $(CXX_FlAGS) $(SRCS) -o $(BIN_NAME)

clean:
	rm $(BIN_NAME)
